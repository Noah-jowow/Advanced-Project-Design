import numpy as np
import os
import base64


# =============================================================================
# AIRFOIL PROFILE GENERATION (shared by both preview and full mesh)
# =============================================================================

def _generate_airfoil_profile(naca_type, customCoords='', n_pts=50):
    """
    Generate a closed-loop airfoil profile as a list of (x, y) tuples,
    normalized to chord = 1. The loop runs: LE → TE (upper) → TE → LE (lower).
    """
    if naca_type == 'Custom' and customCoords:
        pts = []
        for line in customCoords.split('\n'):
            line = line.strip()
            if not line or line[0].isalpha():
                continue
            parts = line.split()
            if len(parts) >= 2:
                try:
                    pts.append((float(parts[0]), float(parts[1])))
                except ValueError:
                    pass
        return pts

    if len(naca_type) != 4 or not naca_type.isdigit():
        naca_type = '0012'

    m = int(naca_type[0]) / 100.0
    p = int(naca_type[1]) / 10.0
    t = int(naca_type[2:]) / 100.0

    upper, lower = [], []
    for i in range(n_pts):
        x = 0.5 * (1.0 - np.cos(i / (n_pts - 1) * np.pi))
        yt = 5 * t * (0.2969 * np.sqrt(x) - 0.126 * x
                       - 0.3516 * x**2 + 0.2843 * x**3 - 0.1036 * x**4)

        if p == 0:
            upper.append((x, yt))
            lower.append((x, -yt))
        else:
            if x <= p:
                yc  = (m / (p**2)) * (2 * p * x - x**2)
                dyc = (2 * m / (p**2)) * (p - x)
            else:
                yc  = (m / ((1 - p)**2)) * (1 - 2 * p + 2 * p * x - x**2)
                dyc = (2 * m / ((1 - p)**2)) * (p - x)

            theta = np.arctan(dyc)
            upper.append((x - yt * np.sin(theta), yc + yt * np.cos(theta)))
            lower.append((x + yt * np.sin(theta), yc - yt * np.cos(theta)))

    # Closed loop: upper (LE→TE) then lower reversed (TE→LE), skip duplicate LE
    return upper + lower[-1:0:-1]


# =============================================================================
# TIER 1: PURE NUMPY FAST PREVIEW  (no Gmsh dependency)
# =============================================================================

def generate_preview(params):
    """
    Pure-NumPy geometry preview.
    Returns (vertices_Nx3, triangles_Mx3) as numpy arrays.
    Runs in < 20ms — suitable for live debounced previews.
    """
    rootChord   = float(params.get('rootChord', 1.0))
    span        = float(params.get('span', 5.0))
    sweepOffset = float(params.get('sweepOffset', 0.0))
    tipScale    = float(params.get('tipScale', 1.0))
    wingtip     = params.get('wingtip', 'flat')
    naca_type   = params.get('nacaType', '0012')
    customCoords = params.get('customCoords', '')

    # --- 1. Airfoil profile (chord = 1, 2D) ---
    profile_2d = _generate_airfoil_profile(naca_type, customCoords)
    n_prof = len(profile_2d)
    if n_prof < 4:
        # Fallback to NACA 0012 if custom import is broken
        profile_2d = _generate_airfoil_profile('0012')
        n_prof = len(profile_2d)

    # Convert to numpy array for vectorized ops
    prof = np.array(profile_2d)  # shape (n_prof, 2)

    # --- 2. Spanwise stations ---
    n_span = 20
    verts = []
    for s in range(n_span + 1):
        t = s / n_span
        z = t * span
        x_off = t * sweepOffset
        chord = rootChord * (1.0 - t * (1.0 - tipScale))

        xs = prof[:, 0] * chord + x_off
        ys = prof[:, 1] * chord
        zs = np.full(n_prof, z)
        verts.append(np.column_stack([xs, ys, zs]))

    vertices = np.vstack(verts)

    # --- 3. Wing surface triangulation ---
    tris = []
    for s in range(n_span):
        for p in range(n_prof):
            p_next = (p + 1) % n_prof
            i00 = s * n_prof + p
            i10 = (s + 1) * n_prof + p
            i01 = s * n_prof + p_next
            i11 = (s + 1) * n_prof + p_next
            tris.append([i00, i10, i11])
            tris.append([i00, i11, i01])

    # --- 4. Root cap (flat fan from centroid at z=0) ---
    root_center = vertices[:n_prof].mean(axis=0)
    root_ci = len(vertices)
    vertices = np.vstack([vertices, root_center.reshape(1, 3)])
    for p in range(n_prof):
        p_next = (p + 1) % n_prof
        tris.append([root_ci, p_next, p])

    # --- 5. Tip cap ---
    tip_start = n_span * n_prof
    tip_ring = vertices[tip_start:tip_start + n_prof]
    tip_cx = tip_ring[:, 0].mean()
    tip_cy = tip_ring[:, 1].mean()

    if wingtip == 'flat':
        tip_ci = len(vertices)
        vertices = np.vstack([vertices, [[tip_cx, tip_cy, span]]])
        for p in range(n_prof):
            p_next = (p + 1) % n_prof
            tris.append([tip_ci, tip_start + p, tip_start + p_next])

    elif wingtip == 'semi-elliptical':
        t_val = int(naca_type[2:]) / 100.0 if naca_type.isdigit() else 0.12
        tip_chord = rootChord * tipScale
        cap_depth = tip_chord * t_val * 1.5

        n_cap = 10
        prev_start = tip_start
        for r in range(1, n_cap + 1):
            theta = (r / n_cap) * (np.pi / 2)
            shrink = np.cos(theta)
            z_off = span + np.sin(theta) * cap_depth

            ring_start = len(vertices)
            ring = np.empty((n_prof, 3))
            ring[:, 0] = tip_cx + (tip_ring[:, 0] - tip_cx) * shrink
            ring[:, 1] = tip_cy + (tip_ring[:, 1] - tip_cy) * shrink
            ring[:, 2] = z_off
            vertices = np.vstack([vertices, ring])

            for p in range(n_prof):
                p_next = (p + 1) % n_prof
                tris.append([prev_start + p, ring_start + p, ring_start + p_next])
                tris.append([prev_start + p, ring_start + p_next, prev_start + p_next])
            prev_start = ring_start

        # Close with pole
        pole_idx = len(vertices)
        vertices = np.vstack([vertices, [[tip_cx, tip_cy, span + cap_depth]]])
        last_start = pole_idx - n_prof
        for p in range(n_prof):
            p_next = (p + 1) % n_prof
            tris.append([pole_idx, last_start + p, last_start + p_next])

    elif wingtip == 'hoerner':
        tip_chord = rootChord * tipScale
        cap_depth = tip_chord * 0.08
        tip_y_max = tip_ring[:, 1].max()

        n_cap = 8
        prev_start = tip_start
        for r in range(1, n_cap + 1):
            frac = r / n_cap
            z_off = span + frac * cap_depth
            shrink_x = 1.0 - frac * 0.7

            ring_start = len(vertices)
            ring = np.empty((n_prof, 3))
            ring[:, 0] = tip_cx + (tip_ring[:, 0] - tip_cx) * shrink_x

            # Hoerner effect: lower surface rises, upper shrinks slightly
            for i in range(n_prof):
                vy = tip_ring[i, 1]
                if vy < tip_cy:
                    ring[i, 1] = tip_cy + (vy - tip_cy) * (1.0 - frac)
                else:
                    ring[i, 1] = tip_cy + (vy - tip_cy) * (1.0 - frac * 0.3)
            ring[:, 2] = z_off
            vertices = np.vstack([vertices, ring])

            for p in range(n_prof):
                p_next = (p + 1) % n_prof
                tris.append([prev_start + p, ring_start + p, ring_start + p_next])
                tris.append([prev_start + p, ring_start + p_next, prev_start + p_next])
            prev_start = ring_start

        # Close with pole
        pole_idx = len(vertices)
        last_start = pole_idx - n_prof
        last_ring = vertices[last_start:last_start + n_prof]
        vertices = np.vstack([vertices, [[last_ring[:, 0].mean(),
                                           last_ring[:, 1].mean(),
                                           span + cap_depth]]])
        for p in range(n_prof):
            p_next = (p + 1) % n_prof
            tris.append([pole_idx, last_start + p, last_start + p_next])

    return vertices, np.array(tris, dtype=np.int32)


# =============================================================================
# TIER 2: FULL GMSH OCC SOLID  (used for domain subtraction & CFD meshing)
# =============================================================================

def generate_mesh(params, log_queue=None):
    """
    Generates a watertight OCC solid using Gmsh for boolean operations.
    Only called on 'Confirm & Continue' or domain/full mesh stages.
    Returns: (coords_Nx3, triangles_or_tets_Mx3or4) as numpy arrays.
    """
    import gmsh
    import multiprocessing

    def qlog(msg):
        if log_queue is not None:
            log_queue.put(msg)

    mode = params.get("mode", "full")
    qlog("[Mesher] Initializing Gmsh...")
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    
    # Enable multi-threading and advanced algorithms for massive speedup
    num_threads = multiprocessing.cpu_count()
    qlog(f"[Mesher] Utilizing {num_threads} CPU cores for parallel meshing.")
    gmsh.option.setNumber("General.NumThreads", num_threads)
    gmsh.option.setNumber("Mesh.Algorithm", 6)   # Frontal-Delaunay for 2D surfaces
    gmsh.option.setNumber("Mesh.Algorithm3D", 10) # HXT parallel Delaunay for 3D volumes

    try:
        gmsh.model.add("CFD_Domain")

        domain_shape = params.get('domainShape', 'block')

        rootChord = float(params.get('rootChord', 1.0))
        span      = float(params.get('span', 5.0))

        domain_tag = None

        if domain_shape == 'cylinder':
            cylRadiusMult     = float(params.get('cylRadiusMult', 20.0))
            cylUpMult         = float(params.get('cylUpMult', 20.0))
            cylDownMult       = float(params.get('cylDownMult', 40.0))
            cylAxis           = params.get('cylAxis', 'X')
            
            R_far = cylRadiusMult * rootChord
            L_far_upstream = cylUpMult * rootChord
            L_far_downstream = cylDownMult * rootChord
            L_total = L_far_upstream + span + L_far_downstream

            if cylAxis == 'X':
                domain_tag = gmsh.model.occ.addCylinder(-L_far_upstream, 0, 0,
                                                  L_total, 0, 0, R_far)
            elif cylAxis == 'Y':
                domain_tag = gmsh.model.occ.addCylinder(0, -L_far_upstream, 0,
                                                  0, L_total, 0, R_far)
            else: # Z
                domain_tag = gmsh.model.occ.addCylinder(0, 0, -L_far_upstream,
                                                  0, 0, L_total, R_far)
        else: # block
            blockXMinMult = float(params.get('blockXMinMult', 20.0))
            blockXMaxMult = float(params.get('blockXMaxMult', 40.0))
            blockYMinMult = float(params.get('blockYMinMult', 20.0))
            blockYMaxMult = float(params.get('blockYMaxMult', 20.0))
            blockZMinMult = float(params.get('blockZMinMult', 0.0))
            blockZMaxMult = float(params.get('blockZMaxMult', 10.0))
            
            x_min = -blockXMinMult * rootChord
            x_max = blockXMaxMult * rootChord
            y_min = -blockYMinMult * rootChord
            y_max = blockYMaxMult * rootChord
            z_min = -blockZMinMult * span
            z_max = blockZMaxMult * span
            
            domain_tag = gmsh.model.occ.addBox(x_min, y_min, z_min,
                                         x_max - x_min, y_max - y_min, z_max - z_min)

        stl_base64 = params.get('stl_base64', '')
        stl_path   = params.get('stl_path', '')
        wing_tag   = None

        if stl_base64:
            import tempfile
            import uuid
            encoded = stl_base64.split(",", 1)[-1] if "," in stl_base64 else stl_base64
            temp_stl = os.path.join(tempfile.gettempdir(), f'custom_upload_{uuid.uuid4().hex}.stl')
            with open(temp_stl, "wb") as f:
                f.write(base64.b64decode(encoded))
            try:
                gmsh.merge(temp_stl)
            finally:
                if os.path.exists(temp_stl):
                    os.remove(temp_stl)
        elif stl_path and os.path.exists(stl_path):
            gmsh.merge(stl_path)

        if not wing_tag:
            naca_type    = params.get('nacaType', '0012')
            wingtip      = params.get('wingtip', 'flat')
            sweepOffset  = float(params.get('sweepOffset', 0.0))
            tipScale     = float(params.get('tipScale', 1.0))
            customCoords = params.get('customCoords', '')

            profile = _generate_airfoil_profile(naca_type, customCoords)
            pts = [(p[0], p[1], 0.0) for p in profile]
            scale = rootChord

            def create_wire(points, z_offset, x_offset, t_scale=1.0):
                # Clean duplicate points
                clean_pts = [points[0]]
                for pt in points[1:]:
                    if np.linalg.norm(np.array(pt) - np.array(clean_pts[-1])) > 1e-6:
                        clean_pts.append(pt)
                if len(clean_pts) > 1 and np.linalg.norm(np.array(clean_pts[-1]) - np.array(clean_pts[0])) < 1e-6:
                    clean_pts.pop() # Remove last point if it duplicates first
                
                tags = []
                for pt in clean_pts:
                    tags.append(gmsh.model.occ.addPoint(
                        pt[0] * scale * t_scale + x_offset,
                        pt[1] * scale * t_scale,
                        pt[2] + z_offset))
                lines = []
                for i in range(len(tags) - 1):
                    lines.append(gmsh.model.occ.addLine(tags[i], tags[i + 1]))
                lines.append(gmsh.model.occ.addLine(tags[-1], tags[0]))
                return gmsh.model.occ.addWire(lines)

            wire_root = create_wire(pts, 0.0, 0.0, 1.0)
            wire_tip  = create_wire(pts, span, sweepOffset, tipScale)

            ext = gmsh.model.occ.addThruSections(
                [wire_root, wire_tip], makeSolid=True, makeRuled=True)
            wing_tag = ext[0][1]

            if wingtip == 'semi-elliptical':
                t_val = int(naca_type[2:]) / 100.0 if naca_type.isdigit() else 0.12
                local_chord = scale * tipScale
                local_thick = local_chord * t_val

                sphere = gmsh.model.occ.addSphere(0, 0, 0, 1.0)
                sx, sy, sz = local_chord / 2.0, local_thick / 2.0, local_thick / 2.0
                tx, ty, tz = sweepOffset + sx, 0.0, span

                gmsh.model.occ.affineTransform([(3, sphere)], [
                    sx, 0, 0, tx,
                    0, sy, 0, ty,
                    0, 0, sz, tz,
                    0, 0, 0, 1
                ])
                fused = gmsh.model.occ.fuse([(3, wing_tag)], [(3, sphere)])
                wing_tag = fused[0][0][1]

            elif wingtip == 'hoerner':
                local_chord = scale * tipScale
                cap_z = span + 0.05 * local_chord
                max_y = max(pt[1] for pt in pts)

                tags = []
                for pt in pts:
                    x_c = pt[0] * local_chord * 0.1 + sweepOffset + 0.02 * local_chord
                    y_c = (max_y * local_chord) - (max_y - pt[1]) * local_chord * 0.05
                    tags.append(gmsh.model.occ.addPoint(x_c, y_c, cap_z))

                lines = []
                for i in range(len(tags) - 1):
                    lines.append(gmsh.model.occ.addLine(tags[i], tags[i + 1]))
                lines.append(gmsh.model.occ.addLine(tags[-1], tags[0]))
                wire_cap = gmsh.model.occ.addWire(lines)

                cap = gmsh.model.occ.addThruSections(
                    [wire_tip, wire_cap], makeSolid=True, makeRuled=True)
                fused = gmsh.model.occ.fuse([(3, wing_tag)], [(3, cap[0][1])])
                wing_tag = fused[0][0][1]

        # --- Geometry-only surface mesh ---
        if mode == "geometry":
            gmsh.model.occ.remove([(3, domain_tag)], recursive=True)
            gmsh.model.occ.synchronize()
            gmsh.model.mesh.generate(2)

            nodeTags, nodeCoords, _ = gmsh.model.mesh.getNodes()
            coords = np.array(nodeCoords).reshape(-1, 3)
            node_map = {tag: i for i, tag in enumerate(nodeTags)}

            elemTypes, _, elemNodeTags = gmsh.model.mesh.getElements(dim=2)
            tris = []
            for i, etype in enumerate(elemTypes):
                if etype == 2:
                    nt = elemNodeTags[i].reshape(-1, 3)
                    for j in range(nt.shape[0]):
                        tris.append([node_map[nt[j, 0]],
                                     node_map[nt[j, 1]],
                                     node_map[nt[j, 2]]])
            return coords, np.array(tris, dtype=np.int32)

        # --- Domain boolean cut ---
        gmsh.model.occ.cut([(3, domain_tag)], [(3, wing_tag)],
                           removeObject=True, removeTool=True)
        gmsh.model.occ.synchronize()

        # --- Boundary Condition Tagging ---
        surfaces = gmsh.model.getEntities(2)
        groups = {}
        # Pre-initialize possible tags based on params
        for t in ['Velocity Inlet', 'Pressure Outlet', 'Symmetry', 'Freestream', 'Wall']:
            groups[t] = []
            
        for dim, tag in surfaces:
            bb = gmsh.model.getBoundingBox(dim, tag)
            xmin, ymin, zmin, xmax, ymax, zmax = bb
            is_far = False
            
            if domain_shape == 'block':
                # Tolerate slight numerical offsets in OCC bounding boxes
                tol = 1e-2
                if abs(zmin - z_min) < tol and abs(zmax - z_min) < tol:
                    groups['Symmetry'].append(tag)
                    is_far = True
                elif abs(xmin - x_min) < tol or abs(xmax - x_max) < tol or abs(ymin - y_min) < tol or abs(ymax - y_max) < tol or abs(zmax - z_max) < tol:
                    groups['Freestream'].append(tag)
                    is_far = True
            else: # cylinder
                tol = 1e-2
                if cylAxis == 'X':
                    if abs(xmin - (-L_far_upstream)) < tol or abs(xmax - (L_total - L_far_upstream)) < tol or abs(ymax - R_far) < tol or abs(ymin - (-R_far)) < tol:
                        groups['Freestream'].append(tag)
                        is_far = True
                elif cylAxis == 'Y':
                    if abs(ymin - (-L_far_upstream)) < tol or abs(ymax - (L_total - L_far_upstream)) < tol or abs(xmax - R_far) < tol or abs(xmin - (-R_far)) < tol:
                        groups['Freestream'].append(tag)
                        is_far = True
                elif cylAxis == 'Z':
                    if abs(zmin - (-L_far_upstream)) < tol and abs(zmax - (-L_far_upstream)) < tol:
                        groups['Symmetry'].append(tag)
                        is_far = True
                    elif abs(zmax - (L_total - L_far_upstream)) < tol or abs(xmax - R_far) < tol or abs(xmin - (-R_far)) < tol:
                        groups['Freestream'].append(tag)
                        is_far = True
                        
            if not is_far:
                groups['Wall'].append(tag)
                
        # Register Physical Groups in Gmsh
        idx = 1
        for name, tags in groups.items():
            if tags:
                gmsh.model.addPhysicalGroup(2, tags, tag=idx, name=name)
                idx += 1
                
        vols = gmsh.model.getEntities(3)
        if vols:
            gmsh.model.addPhysicalGroup(3, [vols[0][1]], tag=idx, name="Fluid")

        if mode == "domain":
            qlog("[Mesher] Generating 2D surface boundary mesh...")
            gmsh.model.mesh.generate(2)
            nodeTags, nodeCoords, _ = gmsh.model.mesh.getNodes()
            coords = np.array(nodeCoords).reshape(-1, 3)
            node_map = {tag: i for i, tag in enumerate(nodeTags)}

            enum_map = {
                "Wall": 0,
                "Velocity Inlet": 1,
                "Pressure Outlet": 2,
                "Symmetry": 3,
                "Freestream": 4
            }
            pgs = gmsh.model.getPhysicalGroups(2)
            
            tris = []
            tri_colors = []
            
            for dim, tag in pgs:
                name = gmsh.model.getPhysicalName(dim, tag)
                enum_val = enum_map.get(name, 0)
                entities = gmsh.model.getEntitiesForPhysicalGroup(dim, tag)
                for e in entities:
                    elemTypes, elemTags, elemNodeTags = gmsh.model.mesh.getElements(dim, e)
                    for i, etype in enumerate(elemTypes):
                        if etype == 2:
                            nt = elemNodeTags[i].reshape(-1, 3)
                            for j in range(nt.shape[0]):
                                tris.append([node_map[nt[j, 0]],
                                             node_map[nt[j, 1]],
                                             node_map[nt[j, 2]]])
                                tri_colors.append(enum_val)
                                
            return coords, np.array(tris, dtype=np.int32), np.array(tri_colors, dtype=np.int32)

        # --- Parse User Parameters ---
        meshSizeMax = float(params.get('meshSizeMax', 5.0))
        meshSizeMin = float(params.get('meshSizeMin', 0.5))
        growthRate = float(params.get('growthRate', 1.2))
        
        blEnabled = params.get('blEnabled', True)
        blFirstLayer = float(params.get('blFirstLayer', 0.05))
        blGrowthRate = float(params.get('blGrowthRate', 1.2))
        blNumLayers = int(params.get('blNumLayers', 5))
        
        wakeEnabled = params.get('wakeEnabled', True)
        wakeLength = float(params.get('wakeLength', 10.0))
        wakeSize = float(params.get('wakeSize', 0.5))

        # --- Full 3D volume mesh ---
        # --- Full 3D volume mesh ---
        # Enable lightning-fast internal PDE solver for sizing interpolation
        gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 1)
        gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 1)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)

        # 1. Apply max size to farfield bounding box faces
        farfield_surfaces = []
        wall_surfaces = groups.get('Wall', [])
        
        all_surfaces = gmsh.model.getEntities(2)
        if not wall_surfaces:
            # If nothing was tagged, assume everything except the very outer boundary is wall
            wall_surfaces = [s[1] for s in all_surfaces]
            
        for dim, tag in all_surfaces:
            if tag not in wall_surfaces:
                farfield_surfaces.append((dim, tag))
                
        if farfield_surfaces:
            gmsh.model.mesh.setSize(farfield_surfaces, meshSizeMax)
            
        # 2. Apply min size (or boundary layer size) to the wing
        wing_size = blFirstLayer if blEnabled else meshSizeMin
        wing_entities = [(2, tag) for tag in wall_surfaces]
        if wing_entities:
            gmsh.model.mesh.setSize(wing_entities, wing_size)

        # 3. Wake Refinement (Density Box)
        if wakeEnabled:
            gmsh.model.mesh.field.add("Box", 1)
            gmsh.model.mesh.field.setNumber(1, "VIn", wakeSize)
            gmsh.model.mesh.field.setNumber(1, "VOut", meshSizeMax)
            # Center the box roughly behind the trailing edge
            root_chord = float(params.get('rootChord', 1.0))
            gmsh.model.mesh.field.setNumber(1, "XMin", root_chord * 0.5) 
            gmsh.model.mesh.field.setNumber(1, "XMax", root_chord + wakeLength)
            gmsh.model.mesh.field.setNumber(1, "YMin", -10.0) 
            gmsh.model.mesh.field.setNumber(1, "YMax", 10.0)
            gmsh.model.mesh.field.setNumber(1, "ZMin", -2.0)
            gmsh.model.mesh.field.setNumber(1, "ZMax", float(params.get('span', 5.0)) + 2.0)
            gmsh.model.mesh.field.setNumber(1, "Thickness", 2.0)
            
            gmsh.model.mesh.field.setAsBackgroundMesh(1)

        qlog("[Mesher] Launching 3D HXT Volume Meshing (This may take several minutes depending on density)...")
        gmsh.model.mesh.generate(3)
        qlog("[Mesher] Volume mesh generation complete. Extracting topology maps...")

        nodeTags, nodeCoords, _ = gmsh.model.mesh.getNodes()
        coords = np.array(nodeCoords).reshape(-1, 3)
        node_map = {tag: i for i, tag in enumerate(nodeTags)}

        elemTypes, _, elemNodeTags = gmsh.model.mesh.getElements(dim=3)
        tets = []
        for i, etype in enumerate(elemTypes):
            if etype == 4:
                nt = elemNodeTags[i].reshape(-1, 4)
                for j in range(nt.shape[0]):
                    tets.append([node_map[nt[j, 0]], node_map[nt[j, 1]],
                                 node_map[nt[j, 2]], node_map[nt[j, 3]]])
                                 
        # Extract boundary triangles and their physical group tags
        boundary_faces = []
        pgs = gmsh.model.getPhysicalGroups(2)
        enum_map = {
            "Wall": 0,
            "Velocity Inlet": 1,
            "Pressure Outlet": 2,
            "Symmetry": 3,
            "Freestream": 4
        }
        for dim, tag in pgs:
            name = gmsh.model.getPhysicalName(dim, tag)
            enum_val = enum_map.get(name, 0)
            entities = gmsh.model.getEntitiesForPhysicalGroup(dim, tag)
            for e in entities:
                bElemTypes, _, bElemNodeTags = gmsh.model.mesh.getElements(dim, e)
                for i, etype in enumerate(bElemTypes):
                    if etype == 2: # Triangle
                        nt = bElemNodeTags[i].reshape(-1, 3)
                        for j in range(nt.shape[0]):
                            boundary_faces.append([
                                node_map[nt[j, 0]],
                                node_map[nt[j, 1]],
                                node_map[nt[j, 2]],
                                enum_val
                            ])
                            
        return coords, np.array(tets, dtype=np.int32), np.array(boundary_faces, dtype=np.int32)

    finally:
        gmsh.finalize()
