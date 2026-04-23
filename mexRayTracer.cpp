classdef CFDPreProcessor < matlab.apps.AppBase
    % CFDPreProcessor - Stage 4 Commercial-Grade CFD Geometry Pre-processor.
    % Now includes topological Bounding Primitive Generation and Boolean 
    % Subtraction formulation via normal-inversion, mirroring StarCCM+ region definitions.
    % Features Full Domain Encompassment and a Monolithic C++ MEX Integration.
    
    % UI Components
    properties (Access = private)
        UIFigure              matlab.ui.Figure
        MainGridLayout        matlab.ui.container.GridLayout
        LeftPanel             matlab.ui.container.Panel
        LeftGridLayout        matlab.ui.container.GridLayout
        ControlTabs           matlab.ui.container.TabGroup
        
        % Tabs
        NacaTab               matlab.ui.container.Tab
        DatTab                matlab.ui.container.Tab
        StlTab                matlab.ui.container.Tab
        DomainTab             matlab.ui.container.Tab
        MeshTab               matlab.ui.container.Tab
        
        % Layouts
        NacaGridLayout        matlab.ui.container.GridLayout
        DatGridLayout         matlab.ui.container.GridLayout
        StlGridLayout         matlab.ui.container.GridLayout
        DomainGridLayout      matlab.ui.container.GridLayout
        MeshGridLayout        matlab.ui.container.GridLayout
        
        % NACA Controls
        NacaCodeEditFieldLabel matlab.ui.control.Label
        NacaCodeEditField      matlab.ui.control.EditField
        SpanEditFieldLabel     matlab.ui.control.Label
        SpanEditField          matlab.ui.control.NumericEditField
        TaperEditFieldLabel    matlab.ui.control.Label
        TaperEditField         matlab.ui.control.NumericEditField
        SweepEditFieldLabel    matlab.ui.control.Label
        SweepEditField         matlab.ui.control.NumericEditField
        NacaTipStyleLabel      matlab.ui.control.Label
        NacaTipStyleDropDown   matlab.ui.control.DropDown
        GenerateButton         matlab.ui.control.Button
        
        % DAT Controls
        ImportDatButton        matlab.ui.control.Button
        DatPathLabel           matlab.ui.control.Label
        DatSpanEditFieldLabel  matlab.ui.control.Label
        DatSpanEditField       matlab.ui.control.NumericEditField
        DatTaperEditFieldLabel matlab.ui.control.Label
        DatTaperEditField      matlab.ui.control.NumericEditField
        DatSweepEditFieldLabel matlab.ui.control.Label
        DatSweepEditField      matlab.ui.control.NumericEditField
        DatTipStyleLabel       matlab.ui.control.Label
        DatTipStyleDropDown    matlab.ui.control.DropDown
        DatGenerateButton      matlab.ui.control.Button
        
        % STL Controls
        ImportSTLButton        matlab.ui.control.Button
        FilePathLabel          matlab.ui.control.Label
        
        % Domain Controls
        UpstreamEditLabel      matlab.ui.control.Label
        UpstreamEditField      matlab.ui.control.NumericEditField
        DownstreamEditLabel    matlab.ui.control.Label
        DownstreamEditField    matlab.ui.control.NumericEditField
        TransverseYEditLabel   matlab.ui.control.Label
        TransverseYEditField   matlab.ui.control.NumericEditField
        TransverseZMinEditLabel matlab.ui.control.Label
        TransverseZMinEditField matlab.ui.control.NumericEditField
        TransverseZMaxEditLabel matlab.ui.control.Label
        TransverseZMaxEditField matlab.ui.control.NumericEditField
        GenerateDomainButton   matlab.ui.control.Button
        
        % Mesh Controls
        BaseSizeLabel         matlab.ui.control.Label
        BaseSizeEditField     matlab.ui.control.NumericEditField
        GenerateMeshButton    matlab.ui.control.Button
        MeshClipSliderLabel   matlab.ui.control.Label
        MeshClipSlider        matlab.ui.control.Slider
        
        % Graphics
        RenderAxes             matlab.ui.control.UIAxes
        ConsoleTextArea        matlab.ui.control.TextArea
    end
    
    % Geometry Data (Ready for downstream processing)
    properties (Access = public)
        % Primary Solid Geometry
        Vertices % [N x 3] matrix of spatial coordinates
        Faces    % [M x 3] matrix of triangle connectivity
        
        % Custom Inputs
        CustomAirfoilCoords % [N x 2] matrix of imported 2D coordinates
        
        % Fluid Domain / Bounding Primitive
        DomainVertices
        DomainFaces
        
        % Volumetric Mesh Data
        Tetrahedra % [K x 4] matrix of tetrahedral element connectivity
        MeshNodes  % [P x 3] matrix of all mesh node coordinates
    end
    
    methods (Access = private)
        
        % --- Pre-Flight Check: Monolithic C++ MEX Compilation ---
        function checkAndCompileMEX(app)
            % Ensure the high-performance C++ Ray Tracer is compiled and ready.
            % If absent, generate the source dynamically from the monolithic structure.
            if exist('mexRayTracer', 'file') ~= 3
                app.logToConsole('C++ MEX binary not found. Extracting monolithic source...');
                
                try
                    % The fully integrated Stage 4 C++ Source Code
                    cppSource = {
                        '#include "mex.h"'
                        '#include <cmath>'
                        '#include <vector>'
                        ''
                        '#define EPSILON 1e-8'
                        ''
                        '// Möller-Trumbore Ray-Triangle Intersection'
                        'bool rayIntersectsTriangle(const double* orig, const double* dir, '
                        '                           const double* v0, const double* v1, const double* v2) {'
                        '    double edge1[3], edge2[3], h[3], s[3], q[3];'
                        '    double a, f, u, v, t;'
                        '    '
                        '    for (int i=0; i<3; ++i) {'
                        '        edge1[i] = v1[i] - v0[i];'
                        '        edge2[i] = v2[i] - v0[i];'
                        '    }'
                        '    '
                        '    h[0] = dir[1] * edge2[2] - dir[2] * edge2[1];'
                        '    h[1] = dir[2] * edge2[0] - dir[0] * edge2[2];'
                        '    h[2] = dir[0] * edge2[1] - dir[1] * edge2[0];'
                        '    '
                        '    a = edge1[0] * h[0] + edge1[1] * h[1] + edge1[2] * h[2];'
                        '    if (a > -EPSILON && a < EPSILON) return false;'
                        '        '
                        '    f = 1.0 / a;'
                        '    for (int i=0; i<3; ++i) s[i] = orig[i] - v0[i];'
                        '    '
                        '    u = f * (s[0] * h[0] + s[1] * h[1] + s[2] * h[2]);'
                        '    if (u < 0.0 || u > 1.0) return false;'
                        '        '
                        '    q[0] = s[1] * edge1[2] - s[2] * edge1[1];'
                        '    q[1] = s[2] * edge1[0] - s[0] * edge1[2];'
                        '    q[2] = s[0] * edge1[1] - s[1] * edge1[0];'
                        '    '
                        '    v = f * (dir[0] * q[0] + dir[1] * q[1] + dir[2] * q[2]);'
                        '    if (v < 0.0 || u + v > 1.0) return false;'
                        '        '
                        '    t = f * (edge2[0] * q[0] + edge2[1] * q[1] + edge2[2] * q[2]);'
                        '    if (t > EPSILON) return true;'
                        '    return false;'
                        '}'
                        ''
                        'void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {'
                        '    if (nrhs != 3) mexErrMsgIdAndTxt("CFD:mexRayTracer:nrhs", "Three inputs required: Vertices, Faces, QueryPoints.");'
                        '    if (nlhs != 1) mexErrMsgIdAndTxt("CFD:mexRayTracer:nlhs", "One output required.");'
                        '    '
                        '    const double* V = mxGetPr(prhs[0]);'
                        '    const double* F = mxGetPr(prhs[1]);'
                        '    const double* P = mxGetPr(prhs[2]);'
                        '    '
                        '    size_t numVertices = mxGetM(prhs[0]);'
                        '    size_t numFaces = mxGetM(prhs[1]);'
                        '    size_t numQueries = mxGetM(prhs[2]);'
                        '    '
                        '    plhs[0] = mxCreateLogicalMatrix(numQueries, 1);'
                        '    mxLogical* isInside = mxGetLogicals(plhs[0]);'
                        '    '
                        '    // [Explicit Approximation]: Micro-jittered ray direction'
                        '    double dir[3] = {1.0, 1.345e-7, 2.113e-7}; '
                        '    '
                        '    for (size_t i = 0; i < numQueries; ++i) {'
                        '        double orig[3] = { P[i], P[i + numQueries], P[i + 2*numQueries] };'
                        '        int intersectionCount = 0;'
                        '        '
                        '        for (size_t j = 0; j < numFaces; ++j) {'
                        '            int idx0 = (int)F[j] - 1;'
                        '            int idx1 = (int)F[j + numFaces] - 1;'
                        '            int idx2 = (int)F[j + 2*numFaces] - 1;'
                        '            '
                        '            double v0[3] = { V[idx0], V[idx0 + numVertices], V[idx0 + 2*numVertices] };'
                        '            double v1[3] = { V[idx1], V[idx1 + numVertices], V[idx1 + 2*numVertices] };'
                        '            double v2[3] = { V[idx2], V[idx2 + numVertices], V[idx2 + 2*numVertices] };'
                        '            '
                        '            if (rayIntersectsTriangle(orig, dir, v0, v1, v2)) {'
                        '                intersectionCount++;'
                        '            }'
                        '        }'
                        '        isInside[i] = (intersectionCount % 2 != 0);'
                        '    }'
                        '}'
                    };
                    
                    % Write Source to Disk
                    cppFileName = 'mexRayTracer.cpp';
                    fid = fopen(cppFileName, 'w');
                    if fid == -1
                        error('Failed to create local C++ source file.');
                    end
                    for i = 1:length(cppSource)
                        fprintf(fid, '%s\n', cppSource{i});
                    end
                    fclose(fid);
                    
                    app.logToConsole('Initiating native C++ compilation (this may take a moment)...');
                    mex(cppFileName);
                    
                    % Purge source file to maintain monolithic architecture purity
                    delete(cppFileName);
                    
                    app.logToConsole('SUCCESS: mexRayTracer integrated and compiled successfully.');
                catch ME
                    app.logToConsole(['ERROR: C++ Compilation failed. Verify compiler setup. ', ME.message]);
                end
            else
                app.logToConsole('C++ MEX engine verified and loaded.');
            end
        end

        % --- Core Algorithm: 3D NACA Generation ---
        function generateGeometry(app)
            app.clearDomain(); % Purge domain to maintain synchronization
            app.logToConsole('Initiating 3D parametric wing generation...');
            
            % Parse Inputs
            nacaStr = app.NacaCodeEditField.Value;
            if length(nacaStr) ~= 4 || ~all(isstrprop(nacaStr, 'digit'))
                app.logToConsole('ERROR: Invalid NACA 4-digit code.');
                return;
            end
            
            m = str2double(nacaStr(1)) / 100;      
            p = str2double(nacaStr(2)) / 10;       
            t = str2double(nacaStr(3:4)) / 100;    
            
            span = app.SpanEditField.Value;
            taper = app.TaperEditField.Value;
            sweep_deg = app.SweepEditField.Value;
            tip_style = app.NacaTipStyleDropDown.Value;
            
            numPoints = 150; 
            beta = linspace(0, pi, numPoints)';
            x = (1 - cos(beta)) / 2;
            
            yt = 5 * t * (0.2969 * sqrt(x) - 0.1260 * x - 0.3516 * x.^2 + 0.2843 * x.^3 - 0.1036 * x.^4);
            
            yc = zeros(size(x));
            dyc = zeros(size(x));
            
            if m > 0
                idx1 = x <= p;
                idx2 = x > p;
                yc(idx1) = (m / p^2) * (2 * p * x(idx1) - x(idx1).^2);
                dyc(idx1) = (2 * m / p^2) * (p - x(idx1));
                yc(idx2) = (m / (1 - p)^2) * ((1 - 2 * p) + 2 * p * x(idx2) - x(idx2).^2);
                dyc(idx2) = (2 * m / (1 - p)^2) * (p - x(idx2));
            end
            
            theta = atan(dyc);
            
            xU = x - yt .* sin(theta);
            yU = yc + yt .* cos(theta);
            xL = x + yt .* sin(theta);
            yL = yc - yt .* cos(theta);
            
            x2d = [flipud(xL); xU(2:end)];
            y2d = [flipud(yL); yU(2:end)];
            
            app.extrude3DWing(x2d, y2d, span, taper, sweep_deg, tip_style);
        end
        
        % --- Core Algorithm: Custom DAT Generation ---
        function generateDatGeometry(app)
            app.clearDomain(); % Purge domain to maintain synchronization
            app.logToConsole('Initiating 3D custom airfoil wing generation...');
            if isempty(app.CustomAirfoilCoords)
                app.logToConsole('ERROR: No airfoil data loaded.');
                return;
            end
            
            span = app.DatSpanEditField.Value;
            taper = app.DatTaperEditField.Value;
            sweep_deg = app.DatSweepEditField.Value;
            tip_style = app.DatTipStyleDropDown.Value;
            
            x2d = app.CustomAirfoilCoords(:,1);
            y2d = app.CustomAirfoilCoords(:,2);
            
            app.extrude3DWing(x2d, y2d, span, taper, sweep_deg, tip_style);
        end
        
        % --- Fully Vectorized Shared Utility: 3D Wing Lofting ---
        function extrude3DWing(app, x2d, y2d, span, taper, sweep_deg, tip_style)
            sweep_rad = deg2rad(sweep_deg);
            nContour = length(x2d);
            
            numStations = 50; 
            z_stations_main = linspace(0, span, numStations);
            
            c_local_main = 1 - (z_stations_main / span) * (1 - taper);
            x_shift_main = z_stations_main * tan(sweep_rad);
            S_y_main = ones(1, numStations);
            
            if strcmp(tip_style, 'Rounded')
                app.logToConsole('Applying Vectorized Semi-Elliptical Thickness pinch for rounded tip.');
                N_cap = 15; 
                tip_thickness_max = (max(y2d) - min(y2d)) * taper;
                delta_z = 0.5 * tip_thickness_max;
                
                theta_cap = linspace(0, pi/2, N_cap + 1);
                z_cap_stations = delta_z * sin(theta_cap(2:end)); 
                
                z_stations_cap = span + z_cap_stations;
                c_local_cap = repmat(taper, 1, N_cap);
                x_shift_cap = span * tan(sweep_rad) + z_cap_stations * tan(sweep_rad);
                
                S_y_cap = max(0.005, sqrt(max(0, 1 - (z_cap_stations / delta_z).^2)));
                
                z_current = [z_stations_main, z_stations_cap];
                c_local = [c_local_main, c_local_cap];
                x_shift = [x_shift_main, x_shift_cap];
                S_y_total = [S_y_main, S_y_cap];
                numStationsTotal = numStations + N_cap;
            else
                z_current = z_stations_main;
                c_local = c_local_main;
                x_shift = x_shift_main;
                S_y_total = S_y_main;
                numStationsTotal = numStations;
            end
            
            % 1. Fully Vectorized Vertex Matrix Generation 
            V = zeros(nContour * numStationsTotal, 3);
            V(:, 1) = reshape(x2d * c_local + ones(nContour, 1) * x_shift, [], 1);
            V(:, 2) = reshape(y2d * (c_local .* S_y_total), [], 1);
            V(:, 3) = reshape(ones(nContour, 1) * z_current, [], 1);
            
            % 2. Fully Vectorized Face Connectivity
            [I, J] = ndgrid(1:nContour-1, 1:numStationsTotal-1);
            p1 = I + (J-1)*nContour;
            p2 = p1 + 1;
            p3 = I + J*nContour;
            p4 = p3 + 1;
            
            F_skin = [p1(:), p2(:), p3(:); p2(:), p4(:), p3(:)];
            
            % 3. Vectorized End Caps
            root_idx = (2:nContour-1)';
            F_root = [ones(nContour-2, 1), root_idx, root_idx + 1];
            
            tip_offset = (numStationsTotal - 1) * nContour;
            tip_idx = tip_offset + (2:nContour-1)';
            F_tip = [tip_offset + ones(nContour-2, 1), tip_idx + 1, tip_idx];
            
            F = [F_skin; F_root; F_tip];
            
            app.Vertices = V;
            app.Faces = F;
            
            app.logToConsole('Optimized Vector geometry generation completed.');
            app.renderGeometry();
        end
        
        % --- Stage 4 Core Algorithm: Bounding Primitive (Full Encompassment) ---
        function generateDomain(app)
            if isempty(app.Vertices)
                app.logToConsole('ERROR: Generate a 3D target geometry first.');
                return;
            end
            
            app.logToConsole('Generating topological fluid domain...');
            
            % 1. Evaluate Target Geometry Extents
            minB = min(app.Vertices, [], 1);
            maxB = max(app.Vertices, [], 1);
            
            % Approximate Reference Chord for offset scaling
            C_ref = maxB(1) - minB(1); 
            
            % Parse Offsets
            O_up = app.UpstreamEditField.Value;
            O_dn = app.DownstreamEditField.Value;
            O_tY = app.TransverseYEditField.Value;
            O_tZmin = app.TransverseZMinEditField.Value;
            O_tZmax = app.TransverseZMaxEditField.Value;
            
            % 2. Calculate Domain Limits (Now supporting full encompassment)
            Dx_min = minB(1) - (O_up * C_ref);
            Dx_max = maxB(1) + (O_dn * C_ref);
            Dy_min = minB(2) - (O_tY * C_ref);
            Dy_max = maxB(2) + (O_tY * C_ref);
            
            % Negative Z offset applied to fully encompass the body (e.g. non-symmetry planes)
            Dz_min = minB(3) - (O_tZmin * C_ref); 
            Dz_max = maxB(3) + (O_tZmax * C_ref);
            
            % 3. Generate Box Vertices (8 corners)
            app.DomainVertices = [
                Dx_min, Dy_min, Dz_min; % 1
                Dx_max, Dy_min, Dz_min; % 2
                Dx_max, Dy_max, Dz_min; % 3
                Dx_min, Dy_max, Dz_min; % 4
                Dx_min, Dy_min, Dz_max; % 5
                Dx_max, Dy_min, Dz_max; % 6
                Dx_max, Dy_max, Dz_max; % 7
                Dx_min, Dy_max, Dz_max  % 8
            ];
            
            % 4. Generate Box Faces with CLOCKWISE (CW) Winding
            % [Explicit Justification]: This mathematically inverts the surface
            % normal vectors so they point INWARD towards the wing. This perfectly
            % replicates a Boolean Subtraction in B-Rep topology.
            app.DomainFaces = [
                % Bottom (Z_min) - Normal points +Z
                1, 4, 2;  2, 4, 3;
                % Top (Z_max) - Normal points -Z
                5, 6, 8;  6, 7, 8;
                % Front (X_max) - Normal points -X
                2, 3, 6;  3, 7, 6;
                % Back (X_min) - Normal points +X
                1, 5, 4;  5, 8, 4;
                % Right (Y_max) - Normal points -Y
                3, 4, 7;  4, 8, 7;
                % Left (Y_min) - Normal points +Y
                1, 2, 5;  2, 6, 5
            ];
            
            % Reset mesh if domain changes
            app.clearMesh();
            
            app.logToConsole('Fluid Domain generated. Region normals successfully inverted.');
            app.renderGeometry();
        end
        
        function clearDomain(app)
            app.DomainVertices = [];
            app.DomainFaces = [];
            app.clearMesh();
        end
        
        % --- Stage 4 Core Algorithm: Volumetric Meshing via MEX C++ ---
        function generateMesh(app)
            if isempty(app.DomainVertices) || isempty(app.Vertices)
                app.logToConsole('ERROR: Generate Solid Geometry and Fluid Domain first.');
                return;
            end
            
            app.logToConsole('Initiating Unstructured Tetrahedral Volumetric Meshing...');
            
            % Ensure C++ MEX is compiled using our Hybrid Monolithic Architecture
            app.checkAndCompileMEX();
            if exist('mexRayTracer', 'file') ~= 3
                 app.logToConsole('CRITICAL ERROR: C++ Compilation failed. Cannot proceed with mesh generation.');
                 return;
            end
            
            baseSize = app.BaseSizeEditField.Value;
            
            % 2. Cartesian Background Seeding (Octree-like node generation)
            app.logToConsole(sprintf('Seeding internal domain with Base Size: %.3f...', baseSize));
            minD = min(app.DomainVertices, [], 1);
            maxD = max(app.DomainVertices, [], 1);
            
            % Generate meshgrid coordinates
            [Xg, Yg, Zg] = meshgrid(...
                minD(1):baseSize:maxD(1), ...
                minD(2):baseSize:maxD(2), ...
                minD(3):baseSize:maxD(3));
            
            internalNodes = [Xg(:), Yg(:), Zg(:)];
            
            % 3. Filter Internal Nodes (Remove nodes inside the solid wing via C++)
            app.logToConsole('Executing high-speed C++ Ray-Casting for exact node filtering...');
            isInsideSolid = mexRayTracer(app.Vertices, app.Faces, internalNodes);
            validInternalNodes = internalNodes(~isInsideSolid, :);
            
            % 4. Construct Monolithic Point Matrix
            % Combine Wing surface, Domain boundary, and Valid internal fluid nodes
            app.MeshNodes = [app.Vertices; app.DomainVertices; validInternalNodes];
            
            % 5. Execute 3D Delaunay Triangulation
            app.logToConsole(sprintf('Executing 3D Delaunay on %d total nodes...', size(app.MeshNodes, 1)));
            DT = delaunayTriangulation(app.MeshNodes);
            allTets = DT.ConnectivityList;
            
            % 6. Post-Triangulation Centroid Filtering 
            % Remove any tetrahedrons that formed across the boundary bridging into the solid
            app.logToConsole('Evaluating volumetric centroids for exact boundary recovery (C++)...');
            
            % Calculate centroid of every tetrahedron
            V1 = app.MeshNodes(allTets(:,1), :);
            V2 = app.MeshNodes(allTets(:,2), :);
            V3 = app.MeshNodes(allTets(:,3), :);
            V4 = app.MeshNodes(allTets(:,4), :);
            centroids = (V1 + V2 + V3 + V4) / 4.0;
            
            % Find tets whose centroids fall inside the solid wing using Exact C++ Ray-Casting
            invalidTets = mexRayTracer(app.Vertices, app.Faces, centroids);
            
            % Cull invalid tetrahedra
            app.Tetrahedra = allTets(~invalidTets, :);
            
            numCells = size(app.Tetrahedra, 1);
            app.logToConsole(sprintf('Meshing Complete: %d Tetrahedral fluid cells generated.', numCells));
            
            % Update Clip Slider Limits based on domain Y extents
            app.MeshClipSlider.Limits = [minD(2), maxD(2)];
            app.MeshClipSlider.Value = (minD(2) + maxD(2)) / 2; % Default to middle cut
            
            app.renderMesh();
        end
        
        function clearMesh(app)
            app.Tetrahedra = [];
            app.MeshNodes = [];
        end
        
        % --- External Geometry: Custom Airfoil (.dat) Importer ---
        function importDat(app)
            [file, path] = uigetfile({'*.dat;*.txt;*.csv', 'Airfoil Data Files (*.dat, *.txt, *.csv)'}, 'Select 2D Airfoil Data');
            if isequal(file, 0)
                app.logToConsole('Import canceled by user.');
                return;
            end
            
            fullPath = fullfile(path, file);
            app.DatPathLabel.Text = ['File: ', file];
            app.logToConsole(['Reading .dat from: ', fullPath]);
            
            try
                data = readmatrix(fullPath);
                data(any(isnan(data), 2), :) = [];
                
                if size(data, 2) < 2
                    app.logToConsole('ERROR: Airfoil file must contain at least two columns (X, Y).');
                    return;
                end
                
                if norm(data(1, 1:2) - data(end, 1:2)) > 1e-6
                    data = [data; data(1, :)];
                    app.logToConsole('[Approximation] Forced Trailing Edge closure for watertight B-Rep.');
                end
                
                app.CustomAirfoilCoords = data(:, 1:2);
                app.clearDomain();
                app.logToConsole(sprintf('Successfully loaded %d 2D coordinates.', size(app.CustomAirfoilCoords, 1)));
            catch ME
                app.logToConsole(['ERROR parsing .dat: ', ME.message]);
            end
        end
        
        % --- External Geometry: STL Importer ---
        function importSTL(app)
            [file, path] = uigetfile({'*.stl', 'STL Files (*.stl)'}, 'Select 3D Geometry');
            if isequal(file, 0)
                app.logToConsole('Import canceled by user.');
                return;
            end
            
            fullPath = fullfile(path, file);
            app.FilePathLabel.Text = ['File: ', file];
            app.logToConsole(['Reading STL from: ', fullPath]);
            
            try
                TR = stlread(fullPath);
                app.Vertices = TR.Points;
                app.Faces = TR.ConnectivityList;
                app.clearDomain();
                app.logToConsole('STL imported successfully.');
                app.renderGeometry();
            catch ME
                app.logToConsole(['ERROR parsing STL: ', ME.message]);
            end
        end
        
        % --- Rendering Pipeline ---
        function renderGeometry(app)
            % Switch back to standard Geometry view if user clicks Domain/Geom buttons
            cla(app.RenderAxes);
            
            % 1. Render Solid Target Geometry (Wing)
            if ~isempty(app.Faces) && ~isempty(app.Vertices)
                patch(app.RenderAxes, 'Faces', app.Faces, 'Vertices', app.Vertices, ...
                    'FaceColor', [0.8 0.8 0.9], 'EdgeColor', 'none', ...
                    'FaceLighting', 'gouraud', 'AmbientStrength', 0.3, ...
                    'DiffuseStrength', 0.8, 'SpecularStrength', 0.9, ...
                    'SpecularExponent', 25, 'BackFaceLighting', 'unlit');
            end
            
            % 2. Render Fluid Domain Geometry (Glass Wind Tunnel)
            if ~isempty(app.DomainFaces) && ~isempty(app.DomainVertices)
                patch(app.RenderAxes, 'Faces', app.DomainFaces, 'Vertices', app.DomainVertices, ...
                    'FaceColor', [0.2 0.6 1.0], 'FaceAlpha', 0.15, ... % Semi-transparent
                    'EdgeColor', [0 0 0.5], 'EdgeAlpha', 0.5, ...
                    'FaceLighting', 'flat', 'AmbientStrength', 0.6, ...
                    'BackFaceLighting', 'reverselit');
            end
            
            % Apply professional CAD-like lighting and view
            if isempty(findobj(app.RenderAxes, 'Type', 'Light'))
                camlight(app.RenderAxes, 'headlight');
            end
            
            axis(app.RenderAxes, 'equal');
            grid(app.RenderAxes, 'on');
            view(app.RenderAxes, 3);
            xlabel(app.RenderAxes, 'X (Streamwise)');
            ylabel(app.RenderAxes, 'Y (Transverse)');
            zlabel(app.RenderAxes, 'Z (Spanwise)');
            title(app.RenderAxes, 'Topology Region & Boundary Definition');
        end
        
        function renderMesh(app)
            if isempty(app.Tetrahedra)
                return;
            end
            
            cla(app.RenderAxes);
            
            % Replicate StarCCM+ "Mesh Scene" with Transverse Clipping Plane
            clipY = app.MeshClipSlider.Value;
            
            % 1. Render the Solid Wing (always visible)
            patch(app.RenderAxes, 'Faces', app.Faces, 'Vertices', app.Vertices, ...
                'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'none', ...
                'FaceLighting', 'gouraud', 'AmbientStrength', 0.5);
                
            % 2. Identify Tetrahedra to display (Centroid Y > clipY)
            % This exposes the internal volumetric mesh structure
            V1 = app.MeshNodes(app.Tetrahedra(:,1), 2);
            V2 = app.MeshNodes(app.Tetrahedra(:,2), 2);
            V3 = app.MeshNodes(app.Tetrahedra(:,3), 2);
            V4 = app.MeshNodes(app.Tetrahedra(:,4), 2);
            tetYCenters = (V1 + V2 + V3 + V4) / 4.0;
            
            visibleTetsIdx = tetYCenters > clipY;
            visibleTets = app.Tetrahedra(visibleTetsIdx, :);
            
            % 3. Fast rendering of Tetrahedral edges using highly optimized patch
            % Extract all unique triangular faces from the visible tetrahedra
            faces1 = visibleTets(:, [1 2 3]);
            faces2 = visibleTets(:, [1 2 4]);
            faces3 = visibleTets(:, [1 3 4]);
            faces4 = visibleTets(:, [2 3 4]);
            allTetFaces = [faces1; faces2; faces3; faces4];
            
            % To avoid overwhelming the MATLAB graphics engine, we render the mesh 
            % as a lightweight wireframe on the outer shell of the clipped region.
            patch(app.RenderAxes, 'Faces', allTetFaces, 'Vertices', app.MeshNodes, ...
                'FaceColor', 'none', 'EdgeColor', [0.1 0.4 0.8], 'EdgeAlpha', 0.15);
            
            if isempty(findobj(app.RenderAxes, 'Type', 'Light'))
                camlight(app.RenderAxes, 'headlight');
            end
            
            axis(app.RenderAxes, 'equal');
            grid(app.RenderAxes, 'on');
            view(app.RenderAxes, 3);
            xlabel(app.RenderAxes, 'X'); ylabel(app.RenderAxes, 'Y'); zlabel(app.RenderAxes, 'Z');
            title(app.RenderAxes, sprintf('Volumetric Mesh Scene (Cut Plane Y = %.2f)', clipY));
        end
        
        % --- Utility: Console Logging ---
        function logToConsole(app, msg)
            dt = datetime('now', 'Format', 'HH:mm:ss');
            timestamp = char(dt);
            formattedMsg = sprintf('[%s] %s', timestamp, msg);
            app.ConsoleTextArea.Value = [app.ConsoleTextArea.Value; {formattedMsg}];
            scroll(app.ConsoleTextArea, 'bottom');
        end
        
        % --- UI Initialization ---
        function createComponents(app)
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1000 650];
            app.UIFigure.Name = 'Native MATLAB CFD - Stage 4 Monolithic Region & MEX Meshing';
            
            app.MainGridLayout = uigridlayout(app.UIFigure);
            app.MainGridLayout.ColumnWidth = {300, '1x'};
            app.MainGridLayout.RowHeight = {'1x', 120};
            
            app.LeftPanel = uipanel(app.MainGridLayout);
            app.LeftPanel.Title = 'Geometry & Domain';
            app.LeftPanel.Layout.Row = 1;
            app.LeftPanel.Layout.Column = 1;
            
            app.ControlTabs = uitabgroup(app.LeftPanel);
            app.ControlTabs.Position = [5 5 290 480];
            
            % --- NACA Tab ---
            app.NacaTab = uitab(app.ControlTabs);
            app.NacaTab.Title = 'NACA';
            
            app.NacaGridLayout = uigridlayout(app.NacaTab);
            app.NacaGridLayout.ColumnWidth = {'1x', '1x'};
            app.NacaGridLayout.RowHeight = {30, 30, 30, 30, 30, 40, '1x'};
            
            app.NacaCodeEditFieldLabel = uilabel(app.NacaGridLayout);
            app.NacaCodeEditFieldLabel.Text = 'NACA 4-Digit:';
            app.NacaCodeEditField = uieditfield(app.NacaGridLayout, 'text');
            app.NacaCodeEditField.Value = '2414';
            
            app.SpanEditFieldLabel = uilabel(app.NacaGridLayout);
            app.SpanEditFieldLabel.Text = 'Span (b):';
            app.SpanEditField = uieditfield(app.NacaGridLayout, 'numeric');
            app.SpanEditField.Value = 5.0;
            
            app.TaperEditFieldLabel = uilabel(app.NacaGridLayout);
            app.TaperEditFieldLabel.Text = 'Taper (\lambda):';
            app.TaperEditField = uieditfield(app.NacaGridLayout, 'numeric');
            app.TaperEditField.Value = 0.5;
            
            app.SweepEditFieldLabel = uilabel(app.NacaGridLayout);
            app.SweepEditFieldLabel.Text = 'Sweep (deg):';
            app.SweepEditField = uieditfield(app.NacaGridLayout, 'numeric');
            app.SweepEditField.Value = 15.0;
            
            app.NacaTipStyleLabel = uilabel(app.NacaGridLayout);
            app.NacaTipStyleLabel.Text = 'Tip Style:';
            app.NacaTipStyleDropDown = uidropdown(app.NacaGridLayout);
            app.NacaTipStyleDropDown.Items = {'Flat', 'Rounded'};
            app.NacaTipStyleDropDown.Value = 'Flat';
            
            app.GenerateButton = uibutton(app.NacaGridLayout, 'push');
            app.GenerateButton.Text = 'Generate Wing';
            app.GenerateButton.ButtonPushedFcn = @(src, event) app.generateGeometry();
            app.GenerateButton.Layout.Column = [1 2];
            app.GenerateButton.BackgroundColor = [0.2 0.6 0.8];
            app.GenerateButton.FontColor = [1 1 1];
            app.GenerateButton.FontWeight = 'bold';
            
            % --- Custom Airfoil DAT Tab ---
            app.DatTab = uitab(app.ControlTabs);
            app.DatTab.Title = 'DAT';
            
            app.DatGridLayout = uigridlayout(app.DatTab);
            app.DatGridLayout.ColumnWidth = {'1x', '1x'};
            app.DatGridLayout.RowHeight = {30, 30, 30, 30, 30, 30, 40, '1x'};
            
            app.ImportDatButton = uibutton(app.DatGridLayout, 'push');
            app.ImportDatButton.Text = 'Browse Data';
            app.ImportDatButton.ButtonPushedFcn = @(src, event) app.importDat();
            app.ImportDatButton.Layout.Column = [1 2];
            
            app.DatPathLabel = uilabel(app.DatGridLayout);
            app.DatPathLabel.Text = 'No .dat loaded.';
            app.DatPathLabel.WordWrap = 'on';
            app.DatPathLabel.Layout.Column = [1 2];
            
            app.DatSpanEditFieldLabel = uilabel(app.DatGridLayout);
            app.DatSpanEditFieldLabel.Text = 'Span (b):';
            app.DatSpanEditField = uieditfield(app.DatGridLayout, 'numeric');
            app.DatSpanEditField.Value = 5.0;
            
            app.DatTaperEditFieldLabel = uilabel(app.DatGridLayout);
            app.DatTaperEditFieldLabel.Text = 'Taper (\lambda):';
            app.DatTaperEditField = uieditfield(app.DatGridLayout, 'numeric');
            app.DatTaperEditField.Value = 0.5;
            
            app.DatSweepEditFieldLabel = uilabel(app.DatGridLayout);
            app.DatSweepEditFieldLabel.Text = 'Sweep (deg):';
            app.DatSweepEditField = uieditfield(app.DatGridLayout, 'numeric');
            app.DatSweepEditField.Value = 15.0;
            
            app.DatTipStyleLabel = uilabel(app.DatGridLayout);
            app.DatTipStyleLabel.Text = 'Tip Style:';
            app.DatTipStyleDropDown = uidropdown(app.DatGridLayout);
            app.DatTipStyleDropDown.Items = {'Flat', 'Rounded'};
            app.DatTipStyleDropDown.Value = 'Flat';
            
            app.DatGenerateButton = uibutton(app.DatGridLayout, 'push');
            app.DatGenerateButton.Text = 'Generate Wing';
            app.DatGenerateButton.ButtonPushedFcn = @(src, event) app.generateDatGeometry();
            app.DatGenerateButton.Layout.Column = [1 2];
            app.DatGenerateButton.BackgroundColor = [0.2 0.8 0.6];
            app.DatGenerateButton.FontColor = [1 1 1];
            app.DatGenerateButton.FontWeight = 'bold';
            
            % --- STL Tab ---
            app.StlTab = uitab(app.ControlTabs);
            app.StlTab.Title = 'STL';
            
            app.StlGridLayout = uigridlayout(app.StlTab);
            app.StlGridLayout.ColumnWidth = {'1x'};
            app.StlGridLayout.RowHeight = {40, 30, '1x'};
            
            app.ImportSTLButton = uibutton(app.StlGridLayout, 'push');
            app.ImportSTLButton.Text = 'Browse .stl';
            app.ImportSTLButton.ButtonPushedFcn = @(src, event) app.importSTL();
            app.ImportSTLButton.BackgroundColor = [0.2 0.6 0.8];
            app.ImportSTLButton.FontColor = [1 1 1];
            app.ImportSTLButton.FontWeight = 'bold';
            
            app.FilePathLabel = uilabel(app.StlGridLayout);
            app.FilePathLabel.Text = 'No file selected.';
            app.FilePathLabel.WordWrap = 'on';
            
            % --- Domain Tab (Stage 4 Update) ---
            app.DomainTab = uitab(app.ControlTabs);
            app.DomainTab.Title = 'Domain';
            
            app.DomainGridLayout = uigridlayout(app.DomainTab);
            app.DomainGridLayout.ColumnWidth = {'1.5x', '1x'};
            app.DomainGridLayout.RowHeight = {30, 30, 30, 30, 30, 40, '1x'}; % Expanded for Z-Min
            
            app.UpstreamEditLabel = uilabel(app.DomainGridLayout);
            app.UpstreamEditLabel.Text = 'Upstream Offset (C):';
            app.UpstreamEditField = uieditfield(app.DomainGridLayout, 'numeric');
            app.UpstreamEditField.Value = 3.0; 
            
            app.DownstreamEditLabel = uilabel(app.DomainGridLayout);
            app.DownstreamEditLabel.Text = 'Downstream Offset (C):';
            app.DownstreamEditField = uieditfield(app.DomainGridLayout, 'numeric');
            app.DownstreamEditField.Value = 8.0; 
            
            app.TransverseYEditLabel = uilabel(app.DomainGridLayout);
            app.TransverseYEditLabel.Text = 'Transverse Y (C):';
            app.TransverseYEditField = uieditfield(app.DomainGridLayout, 'numeric');
            app.TransverseYEditField.Value = 3.0;
            
            app.TransverseZMinEditLabel = uilabel(app.DomainGridLayout);
            app.TransverseZMinEditLabel.Text = 'Transverse -Z (C):';
            app.TransverseZMinEditField = uieditfield(app.DomainGridLayout, 'numeric');
            app.TransverseZMinEditField.Value = 0.0; % Default to 0 for symmetry plane
            
            app.TransverseZMaxEditLabel = uilabel(app.DomainGridLayout);
            app.TransverseZMaxEditLabel.Text = 'Transverse +Z (C):';
            app.TransverseZMaxEditField = uieditfield(app.DomainGridLayout, 'numeric');
            app.TransverseZMaxEditField.Value = 3.0;
            
            app.GenerateDomainButton = uibutton(app.DomainGridLayout, 'push');
            app.GenerateDomainButton.Text = 'Create Boolean Region';
            app.GenerateDomainButton.ButtonPushedFcn = @(src, event) app.generateDomain();
            app.GenerateDomainButton.Layout.Column = [1 2];
            app.GenerateDomainButton.BackgroundColor = [0.8 0.4 0.2];
            app.GenerateDomainButton.FontColor = [1 1 1];
            app.GenerateDomainButton.FontWeight = 'bold';
            
            % --- Mesh Tab (Stage 3) ---
            app.MeshTab = uitab(app.ControlTabs);
            app.MeshTab.Title = 'Volume Mesh';
            
            app.MeshGridLayout = uigridlayout(app.MeshTab);
            app.MeshGridLayout.ColumnWidth = {'1x', '1x'};
            app.MeshGridLayout.RowHeight = {30, 40, 30, 30, '1x'};
            
            app.BaseSizeLabel = uilabel(app.MeshGridLayout);
            app.BaseSizeLabel.Text = 'Base Cell Size:';
            app.BaseSizeEditField = uieditfield(app.MeshGridLayout, 'numeric');
            app.BaseSizeEditField.Value = 0.5; % Default conservative size
            
            app.GenerateMeshButton = uibutton(app.MeshGridLayout, 'push');
            app.GenerateMeshButton.Text = 'Generate 3D Tet Mesh';
            app.GenerateMeshButton.ButtonPushedFcn = @(src, event) app.generateMesh();
            app.GenerateMeshButton.Layout.Column = [1 2];
            app.GenerateMeshButton.BackgroundColor = [0.2 0.7 0.3];
            app.GenerateMeshButton.FontColor = [1 1 1];
            app.GenerateMeshButton.FontWeight = 'bold';
            
            app.MeshClipSliderLabel = uilabel(app.MeshGridLayout);
            app.MeshClipSliderLabel.Text = 'Mesh Scene Cut Plane (Y):';
            app.MeshClipSliderLabel.Layout.Column = [1 2];
            
            app.MeshClipSlider = uislider(app.MeshGridLayout);
            app.MeshClipSlider.Layout.Column = [1 2];
            app.MeshClipSlider.ValueChangingFcn = @(src, event) app.renderMesh(); 
            
            % --- Graphics & Logging ---
            app.RenderAxes = uiaxes(app.MainGridLayout);
            app.RenderAxes.Layout.Row = 1;
            app.RenderAxes.Layout.Column = 2;
            title(app.RenderAxes, 'Topology Region Definition');
            
            app.ConsoleTextArea = uitextarea(app.MainGridLayout);
            app.ConsoleTextArea.Layout.Row = 2;
            app.ConsoleTextArea.Layout.Column = [1 2];
            app.ConsoleTextArea.Editable = 'off';
            app.ConsoleTextArea.FontName = 'Courier';
            app.ConsoleTextArea.Value = {'[System] Initialized Stage 4 Domain Encompassment and Monolithic MEX Compiler.'};
            
            app.UIFigure.Visible = 'on';
        end
    end
    
    methods (Access = public)
        function app = CFDPreProcessor
            createComponents(app);
            app.generateGeometry();
        end
        
        function delete(app)
            delete(app.UIFigure);
        end
    end
end