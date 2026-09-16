import React, { useState, useEffect, useRef } from 'react';
import { useWebSocket } from '../hooks/useWebSocket';
import { PlotCard } from './PlotCard';

const MATERIAL_TMELT: Record<number, number> = {
  0: 1570, // Inconel 718
  1: 1350, // GRCop-42
  2: 2620  // C103 Niobium
};

export function PropulsionPanel() {
  const { status, data, logs, sendCommand, clearData } = useWebSocket('prop');
  
  const [params, setParams] = useState({
    thrust: 50000,
    pa: 101325,
    prop: 0,
    mat: 0,
    angles: 20,
    maxIter: 50,
    pcMinBar: 10,   // bar
    pcMaxBar: 200,  // bar
    twMinMm: 1.0,   // mm
    twMaxMm: 20.0,  // mm
    minMoS: 0.10,
    maxTempRatio: 0.90,
    maxLength: 2.5,
    maxExitRadius: 1.0,
    maxPumpPressure: 250, // bar
    sepMargin: 0.35
  });

  const [activeTab, setActiveTab] = useState<'analytics' | 'cooling' | 'aerodynamics'>('analytics');
  const [isSolving, setIsSolving] = useState(false);
  const [terminalExpanded, setTerminalExpanded] = useState(false);
  const terminalEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (terminalExpanded && terminalEndRef.current) {
      terminalEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [logs, terminalExpanded]);

  useEffect(() => {
    if (data?.status === 'completed' || data?.geometry_x || data?.error) {
      setIsSolving(false);
    }
    if (data?.status === 'optimizing') {
      setIsSolving(true);
    }
  }, [data]);

  const handleRun = () => {
    // 1. Immediately wipe previous plots to visually signal a fresh solve
    clearData();
    setIsSolving(true);

    // 2. Dispatch optimization command with all configurable parameters
    sendCommand('optimize', {
      thrust: params.thrust,
      pa: params.pa,
      prop: params.prop,
      mat: params.mat,
      angles: params.angles,
      maxIter: params.maxIter,
      pcMin: params.pcMinBar * 1e5,
      pcMax: params.pcMaxBar * 1e5,
      twMin: params.twMinMm * 1e-3,
      twMax: params.twMaxMm * 1e-3,
      minMoS: params.minMoS,
      maxTempRatio: params.maxTempRatio,
      maxLength: params.maxLength,
      maxExitRadius: params.maxExitRadius,
      maxPumpPressure: params.maxPumpPressure * 1e5, // bar to Pa
      sepMargin: params.sepMargin
    });
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setParams(prev => ({ ...prev, [name]: isNaN(Number(value)) ? value : Number(value) }));
  };

  const selectedTmelt = MATERIAL_TMELT[params.mat] || 1570;

  // Actual dimensions from current converged solution (if any)
  const geomX = (data?.geometry_x as number[]) || [];
  const geomY = (data?.geometry_y as number[]) || [];
  const xExit = geomX.length > 0 ? geomX[geomX.length - 1] : 0;
  const rExit = geomY.length > 0 ? geomY[geomY.length - 1] : 0;

  // ── Shock Diamond & Aerodynamics Generator ──
  const generatePlumeAndDiamonds = () => {
    if (geomX.length < 2) return [];

    const x = geomX;
    const y = geomY;
    const tw = (data?.t_w as number) || (params.twMinMm * 1e-3);
    const deltaStar = (data?.delta_star as number[]) || [];

    const machExit = data?.mach_dist ? (data.mach_dist as number[])[(data.mach_dist as number[]).length - 1] : 3.2;
    const pe = (data?.pe as number) || (data?.pressure ? (data.pressure as number[])[(data.pressure as number[]).length - 1] : params.pa);
    const pr = pe / Math.max(params.pa, 1);

    // Shock diamond cell spacing: Powell/Tam supersonic jet cell length
    const dExit = 2.0 * rExit;
    const beta = Math.sqrt(Math.max(machExit * machExit - 1.0, 0.25));
    const lCell = Math.max(1.15 * dExit * beta, 0.15);

    const traces: any[] = [];

    // 0. Payload Bay Envelope Clearance Bounds (User Constraints L_max, R_max)
    const envL = params.maxLength;
    const envR = params.maxExitRadius;
    traces.push({
      x: [0, envL, envL, 0],
      y: [envR, envR, -envR, -envR],
      mode: 'lines',
      name: `Payload Bay Envelope (${envL.toFixed(2)}m × ±${envR.toFixed(2)}m)`,
      line: { color: 'rgba(0, 240, 255, 0.65)', width: 2, dash: 'dashdot' },
      hoverinfo: 'name'
    });

    // 1. Structural Outer Casing Shell
    traces.push(
      {
        x: x,
        y: y.map(v => v + tw),
        name: 'Outer Structural Shell',
        mode: 'lines',
        line: { color: '#636366', width: 4 },
        hoverinfo: 'skip'
      },
      {
        x: x,
        y: y.map(v => -(v + tw)),
        mode: 'lines',
        showlegend: false,
        line: { color: '#636366', width: 4 },
        hoverinfo: 'skip'
      },
      // 2. Hot-Gas Wall Contour
      {
        x: x,
        y: y,
        name: 'Hot-Gas Wall Surface',
        mode: 'lines',
        line: { color: '#FF453A', width: 3 },
        hoverinfo: 'text',
        text: x.map((xv, i) => `x: ${xv.toFixed(3)}m, r: ${y[i].toFixed(3)}m`)
      },
      {
        x: x,
        y: y.map(v => -v),
        mode: 'lines',
        showlegend: false,
        line: { color: '#FF453A', width: 3 },
        hoverinfo: 'skip'
      }
    );

    // 3. Boundary Layer Displacement Thickness delta*(x)
    if (deltaStar.length === x.length) {
      traces.push({
        x: x,
        y: y.map((v, i) => Math.max(v - (deltaStar[i] || 0), 0.005)),
        name: 'Boundary Layer Edge (r - δ*)',
        mode: 'lines',
        line: { color: '#FFD60A', width: 2, dash: 'dash' },
        fill: 'tonexty',
        fillcolor: 'rgba(255, 214, 10, 0.12)',
        hoverinfo: 'text',
        text: x.map((xv, i) => `δ*: ${((deltaStar[i] || 0) * 1000).toFixed(2)} mm`)
      });
      traces.push({
        x: x,
        y: y.map((v, i) => -Math.max(v - (deltaStar[i] || 0), 0.005)),
        mode: 'lines',
        showlegend: false,
        line: { color: '#FFD60A', width: 2, dash: 'dash' },
        fill: 'tonexty',
        fillcolor: 'rgba(255, 214, 10, 0.12)',
        hoverinfo: 'skip'
      });
    }

    // 4. Centerline Sonic Core Axis
    traces.push({
      x: [x[0], xExit + 4 * lCell],
      y: [0, 0],
      name: 'Centerline Core Axis',
      mode: 'lines',
      line: { color: '#0A84FF', width: 1.5, dash: 'dash' },
      hoverinfo: 'skip'
    });

    // 5. Plume Shock Diamond Formations
    const numDiamonds = 4;
    for (let k = 0; k < numDiamonds; k++) {
      const decay = Math.pow(0.88, k);
      const x0 = xExit + k * lCell;
      const xMid = x0 + 0.5 * lCell;
      const xEnd = x0 + lCell;

      const rLip = rExit * (1.0 + 0.12 * Math.tanh(pr - 1.0)) * decay;
      const rWaist = rExit * 0.32 * decay;

      // Oblique shock reflections upper & lower
      traces.push({
        x: [x0, xMid, xEnd],
        y: [rLip, rWaist, rLip],
        mode: 'lines',
        name: k === 0 ? 'Oblique Shock Boundaries' : undefined,
        showlegend: k === 0,
        line: { color: '#00F0FF', width: 2.2 },
        hoverinfo: 'skip'
      });

      traces.push({
        x: [x0, xMid, xEnd],
        y: [-rLip, -rWaist, -rLip],
        mode: 'lines',
        showlegend: false,
        line: { color: '#00F0FF', width: 2.2 },
        hoverinfo: 'skip'
      });

      // Internal Expansion / Compression Cross Waves
      traces.push({
        x: [x0, xMid, xEnd],
        y: [rLip, -rWaist, rLip],
        mode: 'lines',
        name: k === 0 ? 'Expansion Fan Reflections' : undefined,
        showlegend: k === 0,
        line: { color: '#BF5AF2', width: 1.5, dash: 'dot' },
        hoverinfo: 'skip'
      });

      traces.push({
        x: [x0, xMid, xEnd],
        y: [-rLip, rWaist, -rLip],
        mode: 'lines',
        showlegend: false,
        line: { color: '#BF5AF2', width: 1.5, dash: 'dot' },
        hoverinfo: 'skip'
      });

      // Mach Disk (Normal Shock Core)
      traces.push({
        x: [xMid, xMid],
        y: [-rWaist, rWaist],
        mode: 'lines',
        name: k === 0 ? 'Mach Disk Normal Shock' : undefined,
        showlegend: k === 0,
        line: { color: '#FFD60A', width: 4 },
        hoverinfo: 'text',
        text: `Mach Disk #${k + 1} at x = ${xMid.toFixed(2)} m (Waist Dia = ${(2 * rWaist * 1000).toFixed(0)} mm)`
      });
    }

    return traces;
  };

  // ── Rao TOP Geometry Traces with Explicit Constraint Envelope Guides ──
  const generateRaoGeometryTraces = () => {
    if (geomX.length < 2) return [];

    const traces: any[] = [
      {
        x: geomX,
        y: geomY,
        name: 'Upper Nozzle Wall',
        mode: 'lines',
        line: { color: '#FF453A', width: 3 }
      },
      {
        x: geomX,
        y: geomY.map(y => -y),
        name: 'Lower Nozzle Wall',
        showlegend: false,
        mode: 'lines',
        line: { color: '#FF453A', width: 3 }
      },
      // Explicit Length Limit Boundary (X = params.maxLength)
      {
        x: [params.maxLength, params.maxLength],
        y: [-params.maxExitRadius * 1.05, params.maxExitRadius * 1.05],
        mode: 'lines',
        name: `Max Length Limit (X ≤ ${params.maxLength.toFixed(2)} m)`,
        line: { color: '#00F0FF', width: 2.2, dash: 'dash' },
        hoverinfo: 'name'
      },
      // Explicit Upper Radius Limit (Y = params.maxExitRadius)
      {
        x: [0, params.maxLength],
        y: [params.maxExitRadius, params.maxExitRadius],
        mode: 'lines',
        name: `Max Radius Limit (R ≤ ${params.maxExitRadius.toFixed(2)} m)`,
        line: { color: '#FF2D55', width: 2, dash: 'dot' },
        hoverinfo: 'name'
      },
      // Explicit Lower Radius Limit (Y = -params.maxExitRadius)
      {
        x: [0, params.maxLength],
        y: [-params.maxExitRadius, -params.maxExitRadius],
        mode: 'lines',
        showlegend: false,
        line: { color: '#FF2D55', width: 2, dash: 'dot' },
        hoverinfo: 'skip'
      }
    ];

    return traces;
  };

  return (
    <div className="flex flex-col h-full w-full overflow-hidden">
      
      {/* Main Work Area: Sidebar Controls + Viewport */}
      <div className="flex-1 grid grid-cols-[400px_1fr] gap-6 p-6 pb-2 min-h-0 overflow-hidden">
        
        {/* Sidebar Controls (Glass Panel) */}
        <aside className="glass-panel overflow-y-auto flex flex-col p-5 gap-5 h-full min-h-0">
          
          {/* Connection Status Header */}
          <div className="flex items-center justify-between pb-3 border-b border-border/50 shrink-0">
            <div className="flex items-center gap-2.5">
              <div className={`status-indicator ${
                status === 'CONNECTED' ? 'status-connected' : 
                status === 'CONNECTING' ? 'status-connecting' : 'status-disconnected'
              }`} />
              <span className="text-xs font-bold text-text-secondary uppercase tracking-wider">
                {status === 'CONNECTED' ? 'Propulsion Core Online' : 'Offline'}
              </span>
            </div>
            <span className="text-[11px] font-mono text-gray-500">
              SLSQP KKT Solver
            </span>
          </div>

          {/* Group 1: Propellant & Combustion Chamber */}
          <div>
            <div className="panel-title">1. Propellant & Chamber State</div>
            <div className="control-group">
              <div className="grid grid-cols-2 gap-3">
                
                <div className="control-item">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    Thrust [N]
                  </label>
                  <input type="number" name="thrust" value={params.thrust} onChange={handleChange} step="1000" />
                </div>
                
                <div className="control-item">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    Ambient Pa [Pa]
                  </label>
                  <input type="number" name="pa" value={params.pa} onChange={handleChange} step="1000" />
                </div>

                <div className="control-item col-span-2">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    Propellant Combination
                  </label>
                  <select name="prop" value={params.prop} onChange={handleChange}>
                    <option value={0}>LOX / RP-1 (Hydrocarbon)</option>
                    <option value={1}>LOX / LH2 (Hydrolox)</option>
                    <option value={2}>LOX / LCH4 (Methalox)</option>
                  </select>
                </div>

                <div className="control-item">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    Pc Min [bar]
                  </label>
                  <input type="number" name="pcMinBar" value={params.pcMinBar} onChange={handleChange} step="5" min="1" />
                </div>

                <div className="control-item">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    Pc Max [bar]
                  </label>
                  <input type="number" name="pcMaxBar" value={params.pcMaxBar} onChange={handleChange} step="10" min="10" />
                </div>

              </div>
            </div>
          </div>

          {/* Group 2: Wall Material & Numerical Mesh */}
          <div>
            <div className="panel-title">2. Material & Discretization</div>
            <div className="control-group">
              <div className="grid grid-cols-2 gap-3">
                
                <div className="control-item col-span-2">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    Chamber Wall Alloy
                  </label>
                  <select name="mat" value={params.mat} onChange={handleChange}>
                    <option value={0}>Inconel 718 (Tmelt: 1570 K)</option>
                    <option value={1}>GRCop-42 (Tmelt: 1350 K)</option>
                    <option value={2}>C103 Niobium (Tmelt: 2620 K)</option>
                  </select>
                </div>

                <div className="control-item">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    t_w Min [mm]
                  </label>
                  <input type="number" name="twMinMm" value={params.twMinMm} onChange={handleChange} step="0.5" min="0.5" max="20" />
                </div>

                <div className="control-item">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    t_w Max [mm]
                  </label>
                  <input type="number" name="twMaxMm" value={params.twMaxMm} onChange={handleChange} step="0.5" min="1" max="50" />
                </div>

                <div className="control-item">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    MoC Rays (Mesh)
                  </label>
                  <input type="number" name="angles" value={params.angles} onChange={handleChange} min="10" max="40" step="1" />
                </div>

                <div className="control-item">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    Max Iterations
                  </label>
                  <input type="number" name="maxIter" value={params.maxIter} onChange={handleChange} min="5" max="200" step="5" />
                </div>

              </div>
            </div>
          </div>

          {/* Group 3: Configurable Mission & Physical Constraints */}
          <div>
            <div className="panel-title">3. Configurable Mission Constraints</div>
            <div className="control-group">
              <div className="grid grid-cols-2 gap-3">
                
                <div className="control-item">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    Max Length [m]
                  </label>
                  <input type="number" name="maxLength" value={params.maxLength} onChange={handleChange} step="0.1" min="0.1" />
                </div>

                <div className="control-item">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    Max Exit Radius [m]
                  </label>
                  <input type="number" name="maxExitRadius" value={params.maxExitRadius} onChange={handleChange} step="0.05" min="0.05" />
                </div>

                <div className="control-item">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    Min Margin Safety
                  </label>
                  <input type="number" name="minMoS" value={params.minMoS} onChange={handleChange} step="0.05" min="-0.5" max="2.0" />
                </div>

                <div className="control-item">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    Max Temp (% Tmelt)
                  </label>
                  <input type="number" name="maxTempRatio" value={params.maxTempRatio} onChange={handleChange} step="0.05" min="0.5" max="0.98" />
                </div>

                <div className="control-item">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    Pump Head [bar]
                  </label>
                  <input type="number" name="maxPumpPressure" value={params.maxPumpPressure} onChange={handleChange} step="10" min="50" />
                </div>

                <div className="control-item">
                  <label className="h-5 flex items-center text-[11px] font-medium text-text-secondary uppercase truncate">
                    Sep Factor (Pe/Pa)
                  </label>
                  <input type="number" name="sepMargin" value={params.sepMargin} onChange={handleChange} step="0.05" min="0.15" max="1.0" />
                </div>

              </div>
            </div>
          </div>

          {/* Action Trigger Button */}
          <div className="mt-auto pt-2 shrink-0">
            <button 
              onClick={handleRun} 
              disabled={status !== 'CONNECTED' || isSolving} 
              className="w-full py-3 px-4 bg-accent-red text-white border-none rounded-lg font-semibold text-sm cursor-pointer transition-all duration-200 hover:bg-[#FF3B30] active:scale-[0.98] shadow-[0_0_15px_rgba(255,69,58,0.3)] disabled:opacity-50 disabled:cursor-not-allowed flex items-center justify-center gap-2"
            >
              {isSolving ? (
                <>
                  <div className="w-4 h-4 border-2 border-white/30 border-t-white rounded-full animate-spin" />
                  <span>SOLVING SLSQP (MAX {params.maxIter})...</span>
                </>
              ) : (
                'RUN VECTOR SOLVER'
              )}
            </button>
          </div>

        </aside>

        {/* Main Viewport */}
        <div className="flex flex-col h-full min-h-0 overflow-hidden bg-black/40 rounded-xl border border-border">
          
          {/* Progress Bar */}
          {isSolving && (
            <div className="w-full h-1 bg-border rounded-full overflow-hidden shrink-0">
              <div className="h-full bg-accent-red rounded-full animate-pulse" style={{ width: '100%', animation: 'progress-slide 1.2s ease-in-out infinite' }} />
            </div>
          )}

          <div className="p-4 flex-1 flex flex-col gap-4 overflow-hidden min-h-0">
            
            {/* Header & Tabs */}
            <div className="flex justify-between items-center border-b border-border pb-3 shrink-0">
              <div className="flex gap-3">
                <button 
                  className={`px-5 py-2 rounded-xl text-xs font-semibold transition-all duration-300 border ${activeTab === 'analytics' ? 'bg-surface-hover border-border-strong text-text-primary shadow-lg' : 'bg-surface border-border/50 text-text-secondary hover:text-text-primary hover:bg-white/[0.05]'}`}
                  onClick={() => setActiveTab('analytics')}
                >
                  Optimizer Analytics
                </button>
                <button 
                  className={`px-5 py-2 rounded-xl text-xs font-semibold transition-all duration-300 border ${activeTab === 'cooling' ? 'bg-surface-hover border-border-strong text-text-primary shadow-lg' : 'bg-surface border-border/50 text-text-secondary hover:text-text-primary hover:bg-white/[0.05]'}`}
                  onClick={() => setActiveTab('cooling')}
                >
                  Thermal & Regenerative Flow
                </button>
                <button 
                  className={`px-5 py-2 rounded-xl text-xs font-semibold transition-all duration-300 border ${activeTab === 'aerodynamics' ? 'bg-surface-hover border-border-strong text-text-primary shadow-lg' : 'bg-surface border-border/50 text-text-secondary hover:text-text-primary hover:bg-white/[0.05]'}`}
                  onClick={() => setActiveTab('aerodynamics')}
                >
                  Aerodynamics & Plume Slice
                </button>
              </div>

              {/* Live Telemetry KPI Badges */}
              <div className="flex items-center gap-2.5 text-xs font-mono">
                <div className="bg-surface px-3 py-1.5 rounded-lg border border-border/50 flex items-center gap-1.5">
                  <span className="text-text-secondary">Status:</span>
                  <span className={isSolving ? 'text-accent-red animate-pulse font-bold' : data?.geometry_x ? 'text-accent-green font-bold' : 'text-text-secondary'}>
                    {isSolving ? `ITER ${data?.iter || 0}/${params.maxIter}` : data?.geometry_x ? 'CONVERGED' : 'STANDBY'}
                  </span>
                </div>
                <div className="bg-surface px-3 py-1.5 rounded-lg border border-border/50 flex items-center gap-1.5">
                  <span className="text-text-secondary">Isp:</span>
                  <span className="text-accent-green font-bold">{data?.isp ? `${Number(data.isp).toFixed(1)} s` : '--'}</span>
                </div>
                <div className="bg-surface px-3 py-1.5 rounded-lg border border-border/50 flex items-center gap-1.5">
                  <span className="text-text-secondary">MoS:</span>
                  <span className={data?.mos !== undefined && Number(data.mos) >= params.minMoS ? 'text-accent-green font-bold' : 'text-accent-red font-bold'}>
                    {data?.mos !== undefined ? Number(data.mos).toFixed(3) : '--'}
                  </span>
                </div>
                <div className="bg-surface px-3 py-1.5 rounded-lg border border-border/50 flex items-center gap-1.5">
                  <span className="text-text-secondary">L/Lmax:</span>
                  <span className={xExit <= params.maxLength ? 'text-accent-cyan font-bold' : 'text-accent-red font-bold'}>
                    {xExit > 0 ? `${xExit.toFixed(2)}/${params.maxLength.toFixed(2)}m` : '--'}
                  </span>
                </div>
                <div className="bg-surface px-3 py-1.5 rounded-lg border border-border/50 flex items-center gap-1.5">
                  <span className="text-text-secondary">R/Rmax:</span>
                  <span className={rExit <= params.maxExitRadius ? 'text-accent-cyan font-bold' : 'text-accent-red font-bold'}>
                    {rExit > 0 ? `${rExit.toFixed(2)}/${params.maxExitRadius.toFixed(2)}m` : '--'}
                  </span>
                </div>
              </div>
            </div>
            
            {/* Visualizations Container */}
            <div className="flex-1 min-h-0 overflow-hidden relative">
              
              {/* Active Solving Indicator Overlay when Visualizations are Cleared */}
              {isSolving && !data?.geometry_x && (
                <div className="absolute inset-0 z-20 flex flex-col items-center justify-center bg-black/60 backdrop-blur-md rounded-xl border border-border/50 gap-4">
                  <div className="w-10 h-10 border-3 border-accent-red/20 border-t-accent-red rounded-full animate-spin" />
                  <div className="flex flex-col items-center gap-1 text-center font-mono">
                    <span className="text-accent-red font-bold text-sm tracking-wider">
                      RE-COMPUTING INVERSE NOZZLE UNDER ACTIVE CONSTRAINTS
                    </span>
                    <span className="text-gray-400 text-xs">
                      Envelopes: L &le; {params.maxLength.toFixed(2)} m | R &le; {params.maxExitRadius.toFixed(2)} m | Min MoS &ge; {params.minMoS.toFixed(2)}
                    </span>
                    <span className="text-gray-500 text-[11px] mt-1">
                      Max SLSQP Iterations: {params.maxIter} | Numerical Rays: {params.angles}
                    </span>
                  </div>
                </div>
              )}

              {/* TAB 1: Optimizer Analytics */}
              {activeTab === 'analytics' && (
                <div className="grid grid-cols-2 grid-rows-2 gap-4 h-full">
                  <div className="card p-2 min-h-0">
                    <PlotCard 
                      title={`Rao TOP Geometry (Envelope: L ≤ ${params.maxLength}m, R ≤ ${params.maxExitRadius}m)`} 
                      data={generateRaoGeometryTraces()}
                      layout={{ 
                        xaxis: {
                          title: 'Axial Coordinate X (m)',
                          scaleratio: 1, 
                          scaleanchor: 'y', 
                          gridcolor: '#2c2c2e',
                          range: [-0.02, Math.max(xExit * 1.1, params.maxLength * 1.05, 0.5)]
                        }, 
                        yaxis: {
                          title: 'Radial Coordinate Y (m)',
                          gridcolor: '#2c2c2e',
                          range: [-Math.max(rExit * 1.15, params.maxExitRadius * 1.1, 0.3), Math.max(rExit * 1.15, params.maxExitRadius * 1.1, 0.3)]
                        },
                        legend: {
                          x: 0.02,
                          y: 0.98,
                          bgcolor: 'rgba(10, 12, 16, 0.85)',
                          bordercolor: '#30363d',
                          borderwidth: 1,
                          font: { size: 10 }
                        }
                      }}
                    />
                  </div>
                  <div className="card p-2 min-h-0">
                    <PlotCard 
                      title="Centerline Mach Number Distribution" 
                      data={data?.mach_dist ? [{ x: data.geometry_x, y: data.mach_dist, line: {color: '#0A84FF', width: 2.5} }] : []}
                      layout={{ xaxis: {gridcolor: '#2c2c2e', title: 'Axial X (m)'}, yaxis: {gridcolor: '#2c2c2e', title: 'Mach (M)'} }}
                    />
                  </div>
                  <div className="card p-2 min-h-0">
                    <PlotCard 
                      title="Thermal Environment & Hot Wall Temperature (K)" 
                      data={data?.temp ? [
                        { x: data.geometry_x, y: data.temp, name: 'Gas Recovery Temp T_aw', line: {color: '#FF9F0A', width: 2, dash: 'dash'} },
                        { x: data.geometry_x, y: data.t_hw, name: 'Hot Wall T_hw', line: {color: '#FF453A', width: 3} },
                        { 
                          x: [data.geometry_x[0], (data.geometry_x as number[])[data.geometry_x.length - 1]],
                          y: [selectedTmelt * params.maxTempRatio, selectedTmelt * params.maxTempRatio],
                          name: `Temp Limit (${(params.maxTempRatio * 100).toFixed(0)}% Tmelt)`,
                          line: {color: '#FF3B30', width: 2, dash: 'dot'}
                        }
                      ] : []}
                      layout={{ xaxis: {gridcolor: '#2c2c2e', title: 'Axial X (m)'}, yaxis: {gridcolor: '#2c2c2e', title: 'Temperature (K)'} }}
                    />
                  </div>
                  <div className="card p-2 min-h-0">
                    <PlotCard 
                      title="Structural Margin of Safety Distribution" 
                      data={data?.margin_of_safety ? [
                        {
                          x: data.geometry_x, 
                          y: data.margin_of_safety, 
                          fill: 'tozeroy', 
                          name: 'Local MoS(x)',
                          line: {color: '#32D74B', width: 2.5}, 
                          fillcolor: 'rgba(50, 215, 75, 0.2)'
                        },
                        {
                          x: [data.geometry_x[0], (data.geometry_x as number[])[data.geometry_x.length - 1]],
                          y: [params.minMoS, params.minMoS],
                          name: `Min MoS Limit (${params.minMoS.toFixed(2)})`,
                          mode: 'lines',
                          line: {color: '#FF453A', width: 2, dash: 'dot'}
                        }
                      ] : []}
                      layout={{ xaxis: {gridcolor: '#2c2c2e', title: 'Axial X (m)'}, yaxis: {gridcolor: '#2c2c2e', title: 'Margin of Safety'} }}
                    />
                  </div>
                </div>
              )}

              {/* TAB 2: Thermal & Regenerative Flow */}
              {activeTab === 'cooling' && (
                <div className="grid grid-cols-[1fr_310px] gap-4 h-full">
                  <div className="card p-2 min-h-0">
                    <PlotCard 
                      title="Regenerative Cooling, Heat Flux & Thermal Stress Profile" 
                      data={data?.geometry_x ? [
                        {
                          x: data.geometry_x,
                          y: data.t_hw,
                          name: 'Hot-Gas Wall T_hw (K)',
                          line: { color: '#FF453A', width: 3.5 },
                          yaxis: 'y1'
                        },
                        {
                          x: data.geometry_x,
                          y: data.temp,
                          name: 'Gas Recovery T_aw (K)',
                          line: { color: '#FF9F0A', width: 2, dash: 'dash' },
                          yaxis: 'y1'
                        },
                        {
                          x: [data.geometry_x[0], (data.geometry_x as number[])[data.geometry_x.length - 1]],
                          y: [selectedTmelt * params.maxTempRatio, selectedTmelt * params.maxTempRatio],
                          name: `Constraint Limit (${(params.maxTempRatio * 100).toFixed(0)}% Tmelt)`,
                          line: { color: '#FF3B30', width: 2, dash: 'dot' },
                          yaxis: 'y1'
                        },
                        {
                          x: [data.geometry_x[0], (data.geometry_x as number[])[data.geometry_x.length - 1]],
                          y: [selectedTmelt, selectedTmelt],
                          name: `Melting Limit Tmelt (${selectedTmelt} K)`,
                          line: { color: '#8E8E93', width: 1.5, dash: 'dashdot' },
                          yaxis: 'y1'
                        },
                        {
                          x: data.geometry_x,
                          y: ((data.q_flux as number[]) || []).map(q => q / 1e6),
                          name: 'Convective Heat Flux q" (MW/m²)',
                          line: { color: '#30D158', width: 3 },
                          yaxis: 'y2'
                        },
                        {
                          x: data.geometry_x,
                          y: ((data.delta_p_cool_dist as number[]) || []).map(p => p / 1e5),
                          name: 'Jacket Pressure Drop ΔP (bar)',
                          line: { color: '#0A84FF', width: 2.5 },
                          yaxis: 'y2'
                        },
                        {
                          x: [data.geometry_x[0], (data.geometry_x as number[])[data.geometry_x.length - 1]],
                          y: [params.maxPumpPressure, params.maxPumpPressure],
                          name: `Pump Head Limit (${params.maxPumpPressure} bar)`,
                          line: { color: '#BF5AF2', width: 2, dash: 'dash' },
                          yaxis: 'y2'
                        }
                      ] : []}
                      layout={{
                        xaxis: { title: 'Axial Coordinate X (m)', gridcolor: '#2c2c2e' },
                        yaxis: {
                          title: 'Temperature (K)',
                          titlefont: { color: '#FF453A' },
                          tickfont: { color: '#FF453A' },
                          gridcolor: '#2c2c2e'
                        },
                        yaxis2: {
                          title: 'Heat Flux (MW/m²) & Pressure (bar)',
                          titlefont: { color: '#30D158' },
                          tickfont: { color: '#30D158' },
                          overlaying: 'y',
                          side: 'right',
                          gridcolor: 'rgba(255,255,255,0.05)'
                        },
                        legend: {
                          x: 0.02,
                          y: 0.98,
                          bgcolor: 'rgba(10, 12, 16, 0.85)',
                          bordercolor: '#30363d',
                          borderwidth: 1,
                          font: { size: 10 }
                        }
                      }}
                    />
                  </div>
                  
                  {/* Thermal Diagnostic Breakdown Panel */}
                  <div className="card p-4 flex flex-col gap-3.5 font-mono text-xs overflow-y-auto">
                    <div className="text-accent-red font-bold uppercase tracking-wider pb-2 border-b border-border">
                      Thermal & Flow Diagnostics
                    </div>
                    
                    <div className="flex flex-col gap-1 bg-surface p-2.5 rounded-lg border border-border/50">
                      <span className="text-text-secondary text-[11px]">Peak Throat Heat Flux</span>
                      <span className="text-accent-green font-bold text-sm">
                        {data?.q_max ? `${(Number(data.q_max) / 1e6).toFixed(2)} MW/m²` : '--'}
                      </span>
                    </div>

                    <div className="flex flex-col gap-1 bg-surface p-2.5 rounded-lg border border-border/50">
                      <span className="text-text-secondary text-[11px]">Max Hot Wall Temp</span>
                      <span className={data?.t_hw_max && Number(data.t_hw_max) <= selectedTmelt * params.maxTempRatio ? 'text-accent-green font-bold text-sm' : 'text-accent-red font-bold text-sm'}>
                        {data?.t_hw_max ? `${Number(data.t_hw_max).toFixed(1)} K` : '--'}
                      </span>
                      <span className="text-gray-500 text-[10px]">
                        Limit: {(selectedTmelt * params.maxTempRatio).toFixed(0)} K | Tmelt: {selectedTmelt} K
                      </span>
                    </div>

                    <div className="flex flex-col gap-1 bg-surface p-2.5 rounded-lg border border-border/50">
                      <span className="text-text-secondary text-[11px]">Coolant Jacket ΔP</span>
                      <span className="text-accent-cyan font-bold text-sm">
                        {data?.delta_p_cool ? `${(Number(data.delta_p_cool) / 1e5).toFixed(1)} bar` : '--'}
                      </span>
                    </div>

                    <div className="flex flex-col gap-1 bg-surface p-2.5 rounded-lg border border-border/50">
                      <span className="text-text-secondary text-[11px]">Turbopump Head Discharge</span>
                      <span className="text-text-primary font-bold text-sm">
                        {data?.pc && data?.delta_p_cool ? `${((1.25 * Number(data.pc) + Number(data.delta_p_cool)) / 1e5).toFixed(1)} bar` : '--'}
                      </span>
                      <span className="text-gray-500 text-[10px]">Head Limit: {params.maxPumpPressure} bar</span>
                    </div>

                    <div className="flex flex-col gap-1 bg-surface p-2.5 rounded-lg border border-border/50">
                      <span className="text-text-secondary text-[11px]">Min Margin of Safety</span>
                      <span className={data?.mos !== undefined && Number(data.mos) >= params.minMoS ? 'text-accent-green font-bold text-sm' : 'text-accent-red font-bold text-sm'}>
                        {data?.mos !== undefined ? Number(data.mos).toFixed(3) : '--'}
                      </span>
                    </div>

                    <div className="text-[10px] text-gray-500 leading-relaxed pt-2 border-t border-border">
                      {'>'} Model: Conjugate series Bartz boundary resistance + homologous temperature degradation.
                    </div>
                  </div>
                </div>
              )}

              {/* TAB 3: Aerodynamics & Plume Slice */}
              {activeTab === 'aerodynamics' && (
                <div className="grid grid-cols-[1fr_310px] gap-4 h-full">
                  <div className="card p-2 min-h-0">
                    <PlotCard 
                      title={`2D Axisymmetric Slice: Boundary Layer Growth, Shock Diamonds & Bay Envelope (L≤${params.maxLength}m, R≤${params.maxExitRadius}m)`} 
                      data={generatePlumeAndDiamonds()}
                      layout={{
                        xaxis: { 
                          title: 'Axial Coordinate X (m)', 
                          gridcolor: '#2c2c2e', 
                          scaleratio: 1, 
                          scaleanchor: 'y',
                          range: [-0.02, Math.max(xExit * 1.5, params.maxLength * 1.1, 0.8)]
                        },
                        yaxis: { 
                          title: 'Radial Coordinate Y (m)', 
                          gridcolor: '#2c2c2e',
                          range: [-Math.max(rExit * 1.25, params.maxExitRadius * 1.1, 0.4), Math.max(rExit * 1.25, params.maxExitRadius * 1.1, 0.4)]
                        },
                        legend: {
                          x: 0.02,
                          y: 0.98,
                          bgcolor: 'rgba(10, 12, 16, 0.85)',
                          bordercolor: '#30363d',
                          borderwidth: 1,
                          font: { size: 10 }
                        }
                      }}
                    />
                  </div>

                  {/* Aerodynamics Diagnostic Panel */}
                  <div className="card p-4 flex flex-col gap-3 font-mono text-xs overflow-y-auto">
                    <div className="text-accent-cyan font-bold uppercase tracking-wider pb-2 border-b border-border">
                      Aero & Envelope Compliance
                    </div>

                    {/* Dedicated Bay Envelope Compliance Box */}
                    <div className="flex flex-col gap-1.5 bg-surface p-2.5 rounded-lg border border-border/50">
                      <span className="text-text-secondary text-[11px] font-bold">Stage Bay Geometric Envelopes</span>
                      <div className="flex justify-between items-center text-[11px]">
                        <span className="text-gray-400">Length Limit:</span>
                        <span className={xExit <= params.maxLength ? 'text-accent-green font-bold' : 'text-accent-red font-bold'}>
                          {xExit > 0 ? `${xExit.toFixed(3)} m / ${params.maxLength.toFixed(2)} m` : '--'}
                        </span>
                      </div>
                      <div className="flex justify-between items-center text-[11px]">
                        <span className="text-gray-400">Radius Limit:</span>
                        <span className={rExit <= params.maxExitRadius ? 'text-accent-green font-bold' : 'text-accent-red font-bold'}>
                          {rExit > 0 ? `${rExit.toFixed(3)} m / ${params.maxExitRadius.toFixed(2)} m` : '--'}
                        </span>
                      </div>
                      <div className="text-[10px] text-gray-500 pt-1 border-t border-border/30">
                        {xExit > 0 && xExit <= params.maxLength && rExit <= params.maxExitRadius 
                          ? '✓ NOZZLE CONFORMS TO BAY ENVELOPE'
                          : xExit > 0 ? '⚠ ENVELOPE BOUNDARY VIOLATION' : 'Awaiting geometry...'}
                      </div>
                    </div>

                    <div className="flex flex-col gap-1 bg-surface p-2.5 rounded-lg border border-border/50">
                      <span className="text-text-secondary text-[11px]">Geometric Expansion Ratio (ε_geom)</span>
                      <span className="text-text-primary font-bold text-sm">
                        {data?.epsilon ? Number(data.epsilon).toFixed(2) : '--'}
                      </span>
                    </div>

                    <div className="flex flex-col gap-1 bg-surface p-2.5 rounded-lg border border-border/50">
                      <span className="text-text-secondary text-[11px]">Effective Aerodynamic Ratio (ε_eff)</span>
                      <span className="text-accent-cyan font-bold text-sm">
                        {data?.epsilon_eff ? Number(data.epsilon_eff).toFixed(2) : '--'}
                      </span>
                    </div>

                    <div className="flex flex-col gap-1 bg-surface p-2.5 rounded-lg border border-border/50">
                      <span className="text-text-secondary text-[11px]">Boundary Layer δ* at Exit</span>
                      <span className="text-[#FFD60A] font-bold text-sm">
                        {data?.delta_exit ? `${(Number(data.delta_exit) * 1000).toFixed(2)} mm` : '--'}
                      </span>
                    </div>

                    <div className="flex flex-col gap-1 bg-surface p-2.5 rounded-lg border border-border/50">
                      <span className="text-text-secondary text-[11px]">Divergence Efficiency (λ_div)</span>
                      <span className="text-accent-green font-bold text-sm">
                        {data?.lambda_div ? `${(Number(data.lambda_div) * 100).toFixed(2)} %` : '--'}
                      </span>
                    </div>

                    <div className="flex flex-col gap-1 bg-surface p-2.5 rounded-lg border border-border/50">
                      <span className="text-text-secondary text-[11px]">Delivered Specific Impulse</span>
                      <span className="text-accent-green font-bold text-sm">
                        {data?.isp ? `${Number(data.isp).toFixed(1)} s` : '--'}
                      </span>
                    </div>

                    <div className="flex flex-col gap-1 bg-surface p-2.5 rounded-lg border border-border/50">
                      <span className="text-text-secondary text-[11px]">Delivered Thrust</span>
                      <span className="text-text-primary font-bold text-sm">
                        {data?.thrust_delivered ? `${(Number(data.thrust_delivered)).toLocaleString('en-US', {maximumFractionDigits: 0})} N` : '--'}
                      </span>
                    </div>

                    <div className="text-[10px] text-gray-500 leading-relaxed pt-1 border-t border-border">
                      {'>'} Visualization maps 2D axisymmetric profile, turbulent compressible boundary layer δ*(x), and supersonic shock diamond cell reflections (λ = 1.3·D·√(M²-1)).
                    </div>
                  </div>
                </div>
              )}

            </div>
          </div>
        </div>
      </div>

      {/* Docked Telemetry Terminal (Bottom of Workbench) */}
      <div className="w-full bg-[#0d1117] border-t border-border flex flex-col shrink-0 shadow-[0_-4px_12px_rgba(0,0,0,0.6)] z-30">
        <div 
          className="px-6 py-2.5 flex items-center justify-between cursor-pointer hover:bg-[#161b22] transition-colors font-mono text-xs text-gray-300"
          onClick={() => setTerminalExpanded(!terminalExpanded)}
        >
          <div className="flex items-center gap-3 truncate">
            <span className="text-accent-red font-bold flex items-center gap-2">
              <span className="inline-block w-2 h-2 rounded-full bg-accent-red animate-pulse"></span>
              NOZZLE TELEMETRY & LOGS
            </span>
            <span className="text-gray-600">|</span>
            <span className="truncate text-gray-400 font-mono text-[11px]">
              {logs.length > 0 ? logs[logs.length - 1].message : "Awaiting propulsion telemetry..."}
            </span>
          </div>
          <div className="shrink-0 flex items-center gap-3 text-gray-400 text-xs">
            <span className="text-[11px] bg-white/5 px-2 py-0.5 rounded border border-white/10 font-mono">{logs.length} logs</span>
            <span className="text-accent-red font-semibold hover:text-white transition-colors">{terminalExpanded ? '▼ Collapse Terminal' : '▲ Expand Terminal'}</span>
          </div>
        </div>
        
        {terminalExpanded && (
          <div className="h-56 overflow-y-auto px-6 py-3 font-mono text-[11px] border-t border-[#30363d] flex flex-col gap-1.5 bg-[#0a0c10]">
            {logs.map((log, i) => (
              <div key={i} className="flex gap-4 hover:bg-white/5 p-1 rounded leading-relaxed">
                <span className="text-gray-500 shrink-0">[{log.timestamp}]</span>
                <span className={
                  log.message.includes('[SYSTEM]') ? 'text-accent-cyan font-medium' : 
                  log.message.includes('[PROP_CORE]') ? 'text-accent-green font-medium' : 
                  log.message.includes('[PROP_PARAMS]') ? 'text-[#e3b341] font-medium' : 
                  log.message.includes('[CLIENT_TX]') ? 'text-[#a371f7] font-medium' : 
                  log.message.includes('[CLIENT_RX]') ? 'text-[#3fb950] font-medium' : 
                  log.message.includes('Error') || log.message.includes('warning') ? 'text-accent-red font-bold' : 
                  'text-gray-300'
                }>
                  {log.message}
                </span>
              </div>
            ))}
            <div ref={terminalEndRef} />
          </div>
        )}
      </div>

    </div>
  );
}
