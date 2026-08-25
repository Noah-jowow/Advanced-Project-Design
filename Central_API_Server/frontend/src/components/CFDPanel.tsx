import React, { useState, useEffect, useRef } from 'react';
import { useWebSocket } from '../hooks/useWebSocket';
import { PlotCard } from './PlotCard';
import type { CFDData } from '../types';

export function CFDPanel() {
  const { status, data, logs, sendCommand } = useWebSocket('aero');
  
  const [terminalExpanded, setTerminalExpanded] = useState(false);
  const terminalEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (terminalExpanded && terminalEndRef.current) {
      terminalEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [logs, terminalExpanded]);
  
  const [generating, setGenerating] = useState(false);
  const [stage, setStage] = useState(0);
  
  const [showGeometry, setShowGeometry] = useState(true);
  const [showDomain, setShowDomain] = useState(true);
  const [showMeshLines, setShowMeshLines] = useState(false);
  
  const [viewMode, setViewMode] = useState<'stage' | 'post'>('stage');
  const [showAdvancedEnv, setShowAdvancedEnv] = useState(false);
  const [postMap, setPostMap] = useState<'pressure'|'mach'|'velocity'>('pressure');
  const [params, setParams] = useState({
    nacaType: '0012', wingtip: 'flat', customCoords: '',
    rootChord: 1.0, span: 5.0, sweepDeg: 0.0, sweepOffset: 0.0, tipScale: 1.0, stl_path: '', stl_base64: '',
    domainShape: 'block', // 'block' or 'cylinder'
    cylAxis: 'X',
    cylRadiusMult: 20.0, cylUpMult: 20.0, cylDownMult: 40.0,
    blockXMinMult: 20.0, blockXMaxMult: 40.0,
    blockYMinMult: 20.0, blockYMaxMult: 20.0,
    blockZMinMult: 0.0, blockZMaxMult: 10.0,
    bcXMin: 'Velocity Inlet', bcXMax: 'Pressure Outlet',
    bcYMin: 'Freestream', bcYMax: 'Freestream',
    bcZMin: 'Symmetry', bcZMax: 'Freestream',
    bcUpstream: 'Velocity Inlet', bcDownstream: 'Pressure Outlet', bcRadial: 'Freestream',
    mesh: 'medium',
    meshSizeMax: 5.0, meshSizeMin: 0.5, growthRate: 1.2,
    blEnabled: true, blFirstLayer: 0.05, blGrowthRate: 1.2, blNumLayers: 5,
    wakeEnabled: true, wakeLength: 10.0, wakeSize: 0.5,
    mach: 0.8, aoa: 2.0, alt: 10000, cfl: 1.5, scheme: 'AUSM+', turb: 'DDES',
    maxIters: 500, epsilon: 1e-5,
    gas_constant: 287.05, T_sl: 288.15, P_sl: 101325.0, gamma: 1.4, prandtl: 0.72,
    lapseRate: 0.0065, suth_mu0: 1.716e-5, suth_T0: 273.15, suth_S: 110.4
  });

  // Clear generating flag when new data arrives
  useEffect(() => {
    // eslint-disable-next-line
    if (data?.mode || data?.error) setGenerating(false);
    if (data?.error) alert("Server Error: " + data.error);
  }, [data]);

  // Debounced live geometry preview (Tier 1: fast NumPy)
  useEffect(() => {
    if (stage === 1 && status === 'CONNECTED') {
      const timer = setTimeout(() => {
        setGenerating(true);
        sendCommand('preview_geometry', params);
      }, 300);
      return () => clearTimeout(timer);
    }
  }, [
    params.nacaType, params.wingtip, params.customCoords, 
    params.rootChord, params.span, params.sweepOffset, params.tipScale, params.stl_base64,
    stage, status, sendCommand, params
  ]);

  // Debounced live domain preview
  useEffect(() => {
    if (stage === 2 && status === 'CONNECTED') {
      const timer = setTimeout(() => {
        setGenerating(true);
        sendCommand('preview_domain', params);
      }, 500);
      return () => clearTimeout(timer);
    }
  }, [
    params.domainShape, params.cylAxis, params.cylRadiusMult, params.cylUpMult, params.cylDownMult,
    params.blockXMinMult, params.blockXMaxMult, params.blockYMinMult, params.blockYMaxMult, params.blockZMinMult, params.blockZMaxMult,
    params.bcXMin, params.bcXMax, params.bcYMin, params.bcYMax, params.bcZMin, params.bcZMax,
    params.bcUpstream, params.bcDownstream, params.bcRadial,
    stage, status, sendCommand, params
  ]);



  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value, type } = e.target;
    if (type === 'checkbox') {
      const checked = (e.target as HTMLInputElement).checked;
      setParams(prev => ({ ...prev, [name]: checked }));
    } else {
      setParams(prev => ({ ...prev, [name]: isNaN(Number(value)) || value === '' ? value : Number(value) }));
    }
  };

  const handleBlur = () => {
    setParams(prev => {
      const next = { ...prev };
      
      // Sanity checks on blur to avoid interfering with active typing (e.g. typing "0.")
      if (typeof next.meshSizeMin === 'number' && next.meshSizeMin <= 0) next.meshSizeMin = 0.01;
      if (typeof next.meshSizeMax === 'number' && typeof next.meshSizeMin === 'number' && next.meshSizeMax < next.meshSizeMin) {
        next.meshSizeMax = next.meshSizeMin;
      }
      if (typeof next.blFirstLayer === 'number' && next.blFirstLayer <= 0) next.blFirstLayer = 0.001;
      if (typeof next.blNumLayers === 'number' && next.blNumLayers < 1) next.blNumLayers = 1;
      if (typeof next.wakeLength === 'number' && next.wakeLength < 0) next.wakeLength = 0;
      if (typeof next.wakeSize === 'number' && next.wakeSize <= 0) next.wakeSize = 0.01;
      if (typeof next.cfl === 'number' && next.cfl <= 0) next.cfl = 0.1;
      if (typeof next.maxIters === 'number' && next.maxIters < 1) next.maxIters = 1;
      
      return next;
    });
  };

  const handleMeshPresetChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const val = e.target.value;
    setParams(prev => {
      let next = { ...prev, mesh: val };
      if (val === 'coarse') {
        next = { ...next, meshSizeMax: 10.0, meshSizeMin: 1.0, blFirstLayer: 0.1, blNumLayers: 3, wakeSize: 1.0 };
      } else if (val === 'medium') {
        next = { ...next, meshSizeMax: 5.0, meshSizeMin: 0.5, blFirstLayer: 0.05, blNumLayers: 5, wakeSize: 0.5 };
      } else if (val === 'fine') {
        next = { ...next, meshSizeMax: 2.5, meshSizeMin: 0.1, blFirstLayer: 0.01, blNumLayers: 8, wakeSize: 0.2 };
      }
      return next;
    });
  };

  const handleSweepChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    const val = value === '' ? 0 : Number(value);
    
    setParams(prev => {
      const next = { ...prev, [name]: value === '' ? '' : val };
      if (name === 'sweepDeg' && value !== '') {
        next.sweepOffset = Number((prev.span * Math.tan(val * Math.PI / 180)).toFixed(4));
      } else if (name === 'sweepOffset' && value !== '') {
        next.sweepDeg = Number((Math.atan(val / prev.span) * 180 / Math.PI).toFixed(4));
      }
      return next;
    });
  };

  const handleSpanChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const val = e.target.value === '' ? 0 : Number(e.target.value);
    setParams(prev => {
      const next = { ...prev, span: val };
      if (e.target.value !== '') {
        next.sweepOffset = Number((val * Math.tan(prev.sweepDeg * Math.PI / 180)).toFixed(4));
      }
      return next;
    });
  };

  const handleFileUpload = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (file) {
      const reader = new FileReader();
      reader.onload = (evt) => {
        setParams(prev => ({ ...prev, customCoords: evt.target?.result as string, nacaType: 'Custom' }));
      };
      reader.readAsText(file);
    }
  };

  const handleStage1 = () => {
    setGenerating(true);
    sendCommand('generate_final_geometry', params);
    setStage(2);
  };

  const handleStage2 = () => {
    sendCommand('preview_domain', params);
    setStage(3);
  };

  const handleStage3 = () => {
    // The actual 3D volume mesh is too large to send to the browser.
    // We just show a success state and the domain surface mesh.
    sendCommand('preview_domain', params); 
    setStage(4);
  };

  const handleStage4 = () => {
    sendCommand('calculate', params);
    setStage(5);
  };

  const getFlowArrowTrace = (dataObj: CFDData) => {
    if (!dataObj?.x || !dataObj?.y) return null;
    const minX = Math.min(...dataObj.x) - (params.rootChord * 2.0);
    const minY = Math.min(...dataObj.y) - params.rootChord;
    const minZ = 0; // Root of the wing
    const arrowScale = params.rootChord * 0.5;
    const u_dir = Math.cos(params.aoa * Math.PI / 180.0);
    const v_dir = Math.sin(params.aoa * Math.PI / 180.0);
    return {
      type: 'cone',
      x: [minX], y: [minY], z: [minZ],
      u: [u_dir], v: [v_dir], w: [0],
      sizemode: 'absolute',
      sizeref: arrowScale,
      showscale: false,
      colorscale: [[0, '#00ffff'], [1, '#00ffff']],
      name: 'Freestream V_inf',
      hoverinfo: 'name'
    };
  };

  const renderGeometryPreview = () => {
    if (!data?.x || data.mode !== 'geometry') return <div className="p-4 text-secondary">No geometry generated yet.</div>;
    const plotData: Record<string, unknown>[] = [{ 
      x: data.x, y: data.y, z: data.z, 
      i: data.i, j: data.j, k: data.k,
      type: 'mesh3d', color: '#2ea043', flatshading: true, showedges: showMeshLines
    }];
    const arrow = getFlowArrowTrace(data);
    if (arrow) plotData.push(arrow);

    return (
      <PlotCard 
        title="3D Geometry Profile" 
        layout={{ uirevision: data.x.length + data.y[0] }}
        scene={{ aspectmode: 'data' }}
        data={plotData} 
      />
    );
  };

  const renderDomainPreview = () => {
    if (!data?.mode || data.mode !== 'domain') return <div className="p-4 text-secondary">No domain generated yet.</div>;
    
    const plotData: Record<string, unknown>[] = [];
    
    if (showGeometry && data.aero_x) {
      plotData.push({ 
        x: data.aero_x, y: data.aero_y, z: data.aero_z, 
        i: data.aero_i, j: data.aero_j, k: data.aero_k,
        type: 'mesh3d', color: '#555555', flatshading: true, name: 'Geometry', showedges: showMeshLines
      });
    }
    
    if (showDomain && data.x && data.c) {
      const cmap = ['#aaaaaa', '#ff4444', '#4444ff', '#cccc22', '#44ff44'];
      const facecolors = data.c.map((val: number) => cmap[val] || '#ffffff');
      plotData.push({ 
        x: data.x, y: data.y, z: data.z, 
        i: data.i, j: data.j, k: data.k,
        facecolor: facecolors,
        type: 'mesh3d', 
        opacity: 0.5, 
        flatshading: true,
        name: 'Domain',
        showedges: showMeshLines
      });
    }

    const arrow = getFlowArrowTrace(data);
    if (arrow) plotData.push(arrow);

    return (
      <div className="relative h-full w-full">
        <PlotCard 
          title="Bounding Domain Verification" 
          layout={{ uirevision: data.x?.length }}
          scene={{ aspectmode: 'data' }}
          data={plotData} 
        />
        
        {/* Legend Overlay */}
        <div className="absolute top-12 right-4 bg-black/40/90 border border-border p-2 rounded text-xs pointer-events-none shadow-md z-10">
          <div className="font-bold mb-1 border-b border-border pb-1">Boundary Tags</div>
          <div className="flex items-center gap-2"><div className="w-3 h-3 bg-[#aaaaaa] border border-gray-600 rounded-sm"></div> Wall</div>
          <div className="flex items-center gap-2 mt-1"><div className="w-3 h-3 bg-[#ff4444] border border-gray-600 rounded-sm"></div> Velocity Inlet</div>
          <div className="flex items-center gap-2 mt-1"><div className="w-3 h-3 bg-[#4444ff] border border-gray-600 rounded-sm"></div> Pressure Outlet</div>
          <div className="flex items-center gap-2 mt-1"><div className="w-3 h-3 bg-[#cccc22] border border-gray-600 rounded-sm"></div> Symmetry</div>
          <div className="flex items-center gap-2 mt-1"><div className="w-3 h-3 bg-[#44ff44] border border-gray-600 rounded-sm"></div> Freestream</div>
        </div>
      </div>
    );
  };

  const renderPostProcessing = () => {
    if (!data?.post_x || data.mode !== 'post_process') return <div className="p-4 text-secondary">Awaiting Post Processing Data...</div>;
    
    let intensityData;
    const colorscale = 'Jet';
    let titleStr: string;
    
    if (postMap === 'mach') {
      intensityData = data.post_mach;
      titleStr = 'Mach Number';
    } else if (postMap === 'velocity') {
      intensityData = data.post_velocity;
      titleStr = 'Velocity (m/s)';
    } else {
      intensityData = data.post_pressure;
      titleStr = 'Pressure (Pa)';
    }

    return (
      <div className="h-full w-full">
        <PlotCard 
          title={`3D Surface Heat Map: ${titleStr}`}
          data={[{ 
            x: data.post_x, y: data.post_y, z: data.post_z, 
            i: data.post_i, j: data.post_j, k: data.post_k,
            intensity: intensityData, type: 'mesh3d', 
            colorscale: colorscale, intensitymode: 'cell', flatshading: true, showedges: showMeshLines
          }]}  
        />
      </div>
    );
  };

  const renderSolverResults = () => {
    if (!data?.cfd_mach) return <div className="p-4 text-secondary">Awaiting solver initialization...</div>;
    
    const plotData: Record<string, unknown>[] = [{ 
      x: data.cfd_x, y: data.cfd_y, z: data.cfd_z, 
      i: data.cfd_i, j: data.cfd_j, k: data.cfd_k,
      intensity: data.cfd_mach, type: 'mesh3d', 
      colorscale: 'Jet', intensitymode: 'cell', flatshading: true, showedges: showMeshLines
    }];

    return (
      <div className="grid grid-cols-2 gap-4 h-full p-2">
        <PlotCard 
          title="Mach Contour (Mid-Span Slice)" 
          data={plotData}  
        />
        <div className="flex flex-col gap-2 h-full">
          <div className="flex gap-2 h-1/3">
            <div className="flex-1">
              <PlotCard 
                title="Lift Coefficient (CL)" 
                data={data?.cl ? [
                  { y: data.cl, name: 'CL', line: {color: '#2ea043'} }
                ] : []}
              />
            </div>
            <div className="flex-1">
              <PlotCard 
                title="Drag Coefficient (CD)" 
                data={data?.cd ? [
                  { y: data.cd, name: 'CD', line: {color: '#f85149'} }
                ] : []}
              />
            </div>
          </div>
          <div className="flex-1">
            <PlotCard 
              title="L2 Residuals (Log Scale)" 
              layout={{ yaxis: { type: 'log', exponentformat: 'e' } }}
              data={data?.residual ? [
                { y: data.residual, name: 'Total', line: {color: '#ffffff', width: 1, dash: 'dot'} },
                { y: data.res_rho, name: 'Rho', line: {color: '#58a6ff'} },
                { y: data.res_rhoU, name: 'RhoU', line: {color: '#ff7b72'} },
                { y: data.res_rhoE, name: 'RhoE', line: {color: '#d2a8ff'} }
              ] : []}
            />
          </div>
        </div>
      </div>
    );
  };

  return (
    <div className="grid grid-cols-[380px_1fr] h-full w-full gap-6 p-6">
      {/* Sidebar Wizard */}
      <aside className="w-80 glass-panel overflow-y-auto flex flex-col p-6 gap-5">
        
        {/* Stage 0: Environment */}
        <div className={`p-3 rounded-md border border-accent-cyan glass-panel`}>
          <div 
            className="font-bold text-sm mb-2 cursor-pointer hover:text-accent-cyan transition-colors flex justify-between items-center"
            onClick={() => { setStage(0); }}
            title="Click to adjust Fluid Properties"
          >
            <span>Stage 0: Flow Conditions & Fluid Properties</span>
            {stage > 0 && <span className="text-xs text-accent-green">✓</span>}
          </div>
          {stage === 0 && (
            <div className="flex flex-col gap-2 text-xs">
              <div className="grid grid-cols-3 gap-2 border-b border-border pb-2 mb-2">
                <div>
                  <label>Mach</label>
                  <input type="number" name="mach" value={params.mach} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                </div>
                <div>
                  <label>AoA [deg]</label>
                  <input type="number" name="aoa" value={params.aoa} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                </div>
                <div>
                  <label>Alt (m)</label>
                  <input type="number" name="alt" value={params.alt} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                </div>
              </div>
              <label>Gas Constant [J/(kg·K)]</label>
              <input type="number" name="gas_constant" value={params.gas_constant} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
              <label>Heat Capacity Ratio (γ)</label>
              <input type="number" name="gamma" value={params.gamma} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" step="0.01" />
              <label>Prandtl Number (Pr)</label>
              <input type="number" name="prandtl" value={params.prandtl} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" step="0.01" />
              <div className="grid grid-cols-2 gap-2 mt-2">
                <div>
                  <label>SL Temp [K]</label>
                  <input type="number" name="T_sl" value={params.T_sl} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                </div>
                <div>
                  <label>SL Press [Pa]</label>
                  <input type="number" name="P_sl" value={params.P_sl} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                </div>
              </div>
              
              <div className="mt-2 border border-border rounded">
                <div 
                  className="bg-[#161b22] px-2 py-1 text-xs font-bold cursor-pointer hover:text-accent-cyan transition-colors flex justify-between"
                  onClick={() => setShowAdvancedEnv(!showAdvancedEnv)}
                >
                  <span>Advanced Atmospheric Models</span>
                  <span>{showAdvancedEnv ? '▼' : '▶'}</span>
                </div>
                {showAdvancedEnv && (
                  <div className="p-2 flex flex-col gap-2">
                    <label>Lapse Rate [K/m]</label>
                    <input type="number" name="lapseRate" value={params.lapseRate} onChange={handleChange} onBlur={handleBlur} step="0.0001" className="w-full bg-black/40 border border-border p-1 rounded" />
                    
                    <div className="text-accent-cyan font-bold mt-1">Sutherland's Law</div>
                    <label>Reference Viscosity (μ0)</label>
                    <input type="number" name="suth_mu0" value={params.suth_mu0} onChange={handleChange} onBlur={handleBlur} step="1e-6" className="w-full bg-black/40 border border-border p-1 rounded" />
                    <div className="grid grid-cols-2 gap-2">
                      <div>
                        <label>Ref Temp (T0) [K]</label>
                        <input type="number" name="suth_T0" value={params.suth_T0} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                      </div>
                      <div>
                        <label>Constant (S) [K]</label>
                        <input type="number" name="suth_S" value={params.suth_S} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                      </div>
                    </div>
                  </div>
                )}
              </div>

              <button onClick={() => setStage(1)} className="btn-primary mt-2">Confirm & Continue</button>
            </div>
          )}
        </div>

        {/* Stage 1: Geometry */}
        <div className={`p-3 rounded-md border ${stage >= 1 ? 'border-accent-cyan glass-panel' : 'border-border opacity-50'}`}>
          <div 
            className="font-bold text-sm mb-2 cursor-pointer hover:text-accent-cyan transition-colors flex justify-between items-center"
            onClick={() => { if (stage >= 1) { setStage(1); } }}
            title="Click to return to Geometry setup"
          >
            <span>Stage 1: Profile & Platform</span>
            {stage > 1 && <span className="text-xs text-accent-green">✓</span>}
          </div>
          {stage === 1 && (
            <div className="flex flex-col gap-2 text-xs">
              <label>Airfoil Profile</label>
              <select name="nacaType" value={params.nacaType} onChange={handleChange} className="bg-black/40 border border-border p-1 rounded">
                <option value="0012">NACA 0012</option>
                <option value="2412">NACA 2412</option>
                <option value="4412">NACA 4412</option>
                <option value="2415">NACA 2415</option>
                <option value="Custom">Custom (.dat)</option>
              </select>
              {params.nacaType === 'Custom' && (
                <input type="file" accept=".dat,.txt" onChange={handleFileUpload} className="bg-black/40 border border-border p-1 rounded text-xs" />
              )}
              <label>Wingtip Style</label>
              <select name="wingtip" value={params.wingtip} onChange={handleChange} className="bg-black/40 border border-border p-1 rounded">
                <option value="flat">Flat</option>
                <option value="semi-elliptical">Semi-Elliptical</option>
                <option value="hoerner">Hoerner</option>
              </select>
              <label>Root Chord (m)</label>
              <input type="number" name="rootChord" value={params.rootChord} onChange={handleChange} className="bg-black/40 border border-border p-1 rounded" />
              <label>Span (m)</label>
              <input type="number" name="span" value={params.span} onChange={handleSpanChange} className="bg-black/40 border border-border p-1 rounded" />
              <label>Tip Scale (Taper Ratio)</label>
              <input type="number" name="tipScale" value={params.tipScale} onChange={handleChange} step="0.1" max="1.0" min="0.01" className="bg-black/40 border border-border p-1 rounded" />
              <div className="flex gap-2">
                <div className="flex-1">
                  <label>Sweep (deg)</label>
                  <input type="number" name="sweepDeg" value={params.sweepDeg} onChange={handleSweepChange} className="w-full bg-black/40 border border-border p-1 rounded" />
                </div>
                <div className="flex-1">
                  <label>Sweep Offset (m)</label>
                  <input type="number" name="sweepOffset" value={params.sweepOffset} onChange={handleSweepChange} className="w-full bg-black/40 border border-border p-1 rounded" />
                </div>
              </div>
              <label>Custom STL Geometry (Fallback)</label>
              <input type="file" accept=".stl" onChange={(e) => {
                const file = e.target.files?.[0];
                if (file) {
                  const reader = new FileReader();
                  reader.onload = (evt) => {
                    setParams(prev => ({ ...prev, stl_base64: evt.target?.result as string, stl_path: file.name }));
                  };
                  reader.readAsDataURL(file);
                }
              }} className="bg-black/40 border border-border p-1 rounded text-xs" />
              {params.stl_path && <div className="text-accent-green text-xs">Loaded: {params.stl_path}</div>}
              <button onClick={handleStage1} className="btn-primary mt-2">Confirm & Continue</button>
            </div>
          )}
        </div>

        {/* Stage 2: Domain */}
        <div className={`p-3 rounded-md border ${stage >= 2 ? 'border-accent-cyan glass-panel' : 'border-border opacity-50'}`}>
          <div 
            className="font-bold text-sm mb-2 cursor-pointer hover:text-accent-cyan transition-colors flex justify-between items-center"
            onClick={() => { if (stage >= 2) { setStage(2); } }}
            title="Click to return to Bounding Domain setup"
          >
            <span>Stage 2: Bounding Domain</span>
            {stage > 2 && <span className="text-xs text-accent-green">✓</span>}
          </div>
          {stage === 2 && (
            <div className="flex flex-col gap-2 text-xs">
              <label>Domain Shape</label>
              <select name="domainShape" value={params.domainShape} onChange={handleChange} className="bg-black/40 border border-border p-1 rounded">
                <option value="block">Block (Box)</option>
                <option value="cylinder">Cylinder</option>
              </select>

              {params.domainShape === 'cylinder' ? (
                <>
                  <label>Cylinder Axis</label>
                  <select name="cylAxis" value={params.cylAxis} onChange={handleChange} className="bg-black/40 border border-border p-1 rounded">
                    <option value="X">X-Axis (Streamwise)</option>
                    <option value="Y">Y-Axis (Vertical)</option>
                    <option value="Z">Z-Axis (Spanwise)</option>
                  </select>
                  <label>Radius (Multiple of Root Chord)</label>
                  <input type="number" name="cylRadiusMult" value={params.cylRadiusMult} onChange={handleChange} className="bg-black/40 border border-border p-1 rounded" />
                  <label>Upstream Length (Multiple of Root Chord)</label>
                  <input type="number" name="cylUpMult" value={params.cylUpMult} onChange={handleChange} className="bg-black/40 border border-border p-1 rounded" />
                  <label>Downstream Length (Multiple of Root Chord)</label>
                  <input type="number" name="cylDownMult" value={params.cylDownMult} onChange={handleChange} className="bg-black/40 border border-border p-1 rounded" />
                </>
              ) : (
                <>
                  <div className="flex gap-2">
                    <div className="flex-1">
                      <label>X-Min (Upstream, Mult of Chord)</label>
                      <input type="number" name="blockXMinMult" value={params.blockXMinMult} onChange={handleChange} className="w-full bg-black/40 border border-border p-1 rounded" />
                    </div>
                    <div className="flex-1">
                      <label>X-Max (Downstream, Mult of Chord)</label>
                      <input type="number" name="blockXMaxMult" value={params.blockXMaxMult} onChange={handleChange} className="w-full bg-black/40 border border-border p-1 rounded" />
                    </div>
                  </div>
                  <div className="flex gap-2">
                    <div className="flex-1">
                      <label>Y-Min (Lower, Mult of Chord)</label>
                      <input type="number" name="blockYMinMult" value={params.blockYMinMult} onChange={handleChange} className="w-full bg-black/40 border border-border p-1 rounded" />
                    </div>
                    <div className="flex-1">
                      <label>Y-Max (Upper, Mult of Chord)</label>
                      <input type="number" name="blockYMaxMult" value={params.blockYMaxMult} onChange={handleChange} className="w-full bg-black/40 border border-border p-1 rounded" />
                    </div>
                  </div>
                  <div className="flex gap-2">
                    <div className="flex-1">
                      <label>Z-Min (Root/Symmetry, Mult of Span)</label>
                      <input type="number" name="blockZMinMult" value={params.blockZMinMult} onChange={handleChange} className="w-full bg-black/40 border border-border p-1 rounded" />
                    </div>
                    <div className="flex-1">
                      <label>Z-Max (Tip, Mult of Span)</label>
                      <input type="number" name="blockZMaxMult" value={params.blockZMaxMult} onChange={handleChange} className="w-full bg-black/40 border border-border p-1 rounded" />
                    </div>
                  </div>
                </>
              )}
              <button onClick={handleStage2} className="btn-primary mt-2">Generate Domain</button>
            </div>
          )}
        </div>

        {/* Stage 3: Mesher */}
        <div className={`p-3 rounded-md border ${stage >= 3 ? 'border-accent-cyan glass-panel' : 'border-border opacity-50'}`}>
          <div 
            className="font-bold text-sm mb-2 cursor-pointer hover:text-accent-cyan transition-colors flex justify-between items-center"
            onClick={() => { if (stage >= 3) { setStage(3); } }}
            title="Click to return to Mesher setup"
          >
            <span>Stage 3: Mesher</span>
            {stage > 3 && <span className="text-xs text-accent-green">✓</span>}
          </div>
          {stage === 3 && (
            <div className="flex flex-col gap-2 text-xs">
              <label className="font-bold text-accent-cyan">Meshing Preset</label>
              <select name="mesh" value={params.mesh} onChange={handleMeshPresetChange} className="bg-black/40 border border-border p-1 rounded">
                <option value="coarse">Coarse (Fast Preview)</option>
                <option value="medium">Medium (Standard)</option>
                <option value="fine">Fine (High Fidelity)</option>
              </select>

              <div className="mt-2 font-bold text-accent-cyan">Global Sizing</div>
              <div className="grid grid-cols-2 gap-2">
                <div>
                  <label>Max Size (m)</label>
                  <input type="number" name="meshSizeMax" value={params.meshSizeMax} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                </div>
                <div>
                  <label>Min Size (m)</label>
                  <input type="number" name="meshSizeMin" value={params.meshSizeMin} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                </div>
                <div>
                  <label>Growth Rate</label>
                  <input type="number" name="growthRate" value={params.growthRate} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                </div>
              </div>

              <div className="mt-2 flex items-center justify-between">
                <label className="font-bold text-accent-cyan">Boundary Layer</label>
                <input type="checkbox" name="blEnabled" checked={params.blEnabled} onChange={handleChange} />
              </div>
              {params.blEnabled && (
                <div className="grid grid-cols-2 gap-2">
                  <div>
                    <label>First Layer (m)</label>
                    <input type="number" name="blFirstLayer" value={params.blFirstLayer} onChange={handleChange} onBlur={handleBlur} className={`w-full bg-black/40 border p-1 rounded ${params.blFirstLayer < 0.01 ? 'border-accent-red' : 'border-border'}`} />
                  </div>
                  <div>
                    <label>Layers</label>
                    <input type="number" name="blNumLayers" value={params.blNumLayers} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                  </div>
                  {params.blFirstLayer < 0.01 && (
                    <div className="col-span-2 text-accent-red text-[10px] leading-tight">
                      Warning: Very fine first layer (y+&lt;1) may drastically increase meshing time.
                    </div>
                  )}
                </div>
              )}

              <div className="mt-2 flex items-center justify-between" title="Refines the mesh behind the trailing edge to capture vortex shedding and wake physics.">
                <label className="font-bold text-accent-cyan">Wake Refinement</label>
                <input type="checkbox" name="wakeEnabled" checked={params.wakeEnabled} onChange={handleChange} />
              </div>
              {params.wakeEnabled && (
                <div className="grid grid-cols-2 gap-2">
                  <div>
                    <label>Wake Length (m)</label>
                    <input type="number" name="wakeLength" value={params.wakeLength} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                  </div>
                  <div>
                    <label>Wake Size (m)</label>
                    <input type="number" name="wakeSize" value={params.wakeSize} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                  </div>
                </div>
              )}

              <button onClick={handleStage3} className="btn-primary mt-2">Configure Mesher</button>
            </div>
          )}
        </div>

        {/* Stage 4: Solver */}
        <div className={`p-3 rounded-md border ${stage >= 4 ? 'border-accent-cyan glass-panel' : 'border-border opacity-50'}`}>
          <div 
            className="font-bold text-sm mb-2 cursor-pointer hover:text-accent-cyan transition-colors flex justify-between items-center"
            onClick={() => { if (stage >= 4) { setStage(4); } }}
          >
            <span>Stage 4: Solver Settings</span>
            {stage > 4 && <span className="text-xs text-accent-green">✓</span>}
          </div>
          {stage === 4 && (
            <div className="flex flex-col gap-2 text-xs">
              <div className="grid grid-cols-3 gap-2">
                <div>
                  <label>CFL</label>
                  <input type="number" name="cfl" value={params.cfl} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                </div>
                <div>
                  <label>Max Iters</label>
                  <input type="number" name="maxIters" value={params.maxIters} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                </div>
                <div>
                  <label>Epsilon</label>
                  <input type="number" name="epsilon" value={params.epsilon} onChange={handleChange} onBlur={handleBlur} className="w-full bg-black/40 border border-border p-1 rounded" />
                </div>
              </div>
              <label>Discretization Scheme</label>
              <select name="scheme" value={params.scheme} onChange={handleChange} className="bg-black/40 border border-border p-1 rounded">
                <option value="AUSM+">AUSM+</option>
                <option value="ROE">Roe FDS</option>
              </select>
              <label>Turbulence Model</label>
              <select name="turb" value={params.turb} onChange={handleChange} className="bg-black/40 border border-border p-1 rounded">
                <option value="DDES">SA-DDES</option>
                <option value="RANS">SA-RANS</option>
              </select>
              <button onClick={handleStage4} className="btn-primary mt-2">▶ RUN SOLVER</button>
            </div>
          )}
        </div>

        <div className="text-center text-xs mt-auto text-secondary flex justify-center items-center gap-2">
          <div className={`w-2 h-2 rounded-full ${status === 'CONNECTED' ? 'bg-accent-green' : 'bg-accent-red'}`}></div>
          WS: {status}
        </div>
      </aside>

      {/* Main Viewport */}
      <div className="flex-1 p-4 bg-black/40 flex flex-col overflow-hidden">
        {/* Progress Bar */}
        {generating && (
          <div className="w-full h-1 bg-border rounded-full overflow-hidden mb-1 shrink-0">
            <div className="h-full bg-primary rounded-full animate-pulse" style={{ width: '100%', animation: 'progress-slide 1.2s ease-in-out infinite' }} />
          </div>
        )}
        <div className="flex justify-between items-center mb-4 border-b border-border">
          <div className="flex gap-2">
            <button 
              className={`px-6 py-2 text-sm font-bold rounded-t-md ${viewMode === 'stage' ? 'bg-primary text-black' : 'glass-panel text-secondary hover:text-white'}`}
              onClick={() => setViewMode('stage')}
            >
              Current Stage View
            </button>
            <button 
              className={`px-6 py-2 text-sm font-bold rounded-t-md ${viewMode === 'post' ? 'bg-primary text-black' : 'glass-panel text-secondary hover:text-white'}`}
              onClick={() => setViewMode('post')}
              disabled={data?.mode !== 'post_process'}
            >
              Post Processing
            </button>
          </div>
          {viewMode === 'stage' ? (
            <div className="flex gap-4 pr-4">
              <label className="flex items-center gap-2 text-sm text-secondary cursor-pointer hover:text-white">
                <input type="checkbox" checked={showGeometry} onChange={(e) => setShowGeometry(e.target.checked)} className="cursor-pointer" />
                Show Geometry
              </label>
              <label className="flex items-center gap-2 text-sm text-secondary cursor-pointer hover:text-white">
                <input type="checkbox" checked={showDomain} onChange={(e) => setShowDomain(e.target.checked)} className="cursor-pointer" />
                Show Bounding Domain
              </label>
              <label className="flex items-center gap-2 text-sm text-secondary cursor-pointer hover:text-white">
                <input type="checkbox" checked={showMeshLines} onChange={(e) => setShowMeshLines(e.target.checked)} className="cursor-pointer" />
                Show Mesh Edges
              </label>
            </div>
          ) : (
            <div className="flex gap-4 pr-4">
              <select value={postMap} onChange={(e) => setPostMap(e.target.value as 'pressure'|'mach'|'velocity')} className="bg-black/40 border border-border p-1 rounded text-sm text-white">
                <option value="pressure">Pressure (Pa)</option>
                <option value="mach">Mach Number</option>
                <option value="velocity">Velocity (m/s)</option>
              </select>
            </div>
          )}
        </div>
        
        <div className="flex-1 min-h-0 relative flex flex-col">
          <div className="flex-1 w-full overflow-hidden">
            {viewMode === 'stage' ? (
              <>
                {(stage === 1 || stage === 2) && (data?.mode === 'geometry' || data?.mode === 'domain' ? (stage === 1 ? renderGeometryPreview() : renderDomainPreview()) : <div className="p-4 text-secondary flex h-full items-center justify-center border border-dashed border-border rounded-lg">Configure Geometry parameters and click Generate.</div>)}
                {(stage === 3 || stage === 4) && renderDomainPreview()}
                {stage === 5 && renderSolverResults()} 
              </>
            ) : (
              data?.mode === 'post_process' ? renderPostProcessing() : null
            )}
          </div>

          {/* Live Server Console */}
          <div className="w-full bg-[#0d1117] border-t border-border flex flex-col shrink-0 shadow-[0_-4px_10px_rgba(0,0,0,0.5)] z-20">
            <div 
              className="px-4 py-2 flex items-center justify-between cursor-pointer hover:bg-[#161b22] transition-colors font-mono text-[11px] text-gray-400"
              onClick={() => setTerminalExpanded(!terminalExpanded)}
            >
              <div className="flex items-center gap-3 truncate">
                <span className="text-accent-cyan font-bold">▶_</span>
                <span className="truncate text-gray-300">{logs.length > 0 ? logs[logs.length - 1].message : "Awaiting server logs..."}</span>
              </div>
              <div className="shrink-0 flex items-center gap-2">
                <span>{terminalExpanded ? '▼ Close' : '▲ Expand'}</span>
              </div>
            </div>
            
            {terminalExpanded && (
              <div className="h-64 overflow-y-auto px-4 py-2 font-mono text-[11px] border-t border-[#30363d] flex flex-col gap-1 bg-[#0a0c10]">
                {logs.map((log, i) => (
                  <div key={i} className="flex gap-4 hover:bg-white/5 p-0.5 rounded leading-relaxed">
                    <span className="text-gray-500 shrink-0">[{log.timestamp}]</span>
                    <span className={log.message.includes('[SYSTEM]') ? 'text-accent-cyan' : log.message.includes('[AERO_CORE]') ? 'text-accent-green' : log.message.includes('Error') ? 'text-accent-red' : 'text-gray-300'}>
                      {log.message}
                    </span>
                  </div>
                ))}
                <div ref={terminalEndRef} />
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
