import { useState, useEffect, useRef } from 'react';
import { useWebSocket } from '../hooks/useWebSocket';
import { PlotCard } from './PlotCard';
import type { SADCData } from '../types';
import { Orbit, Compass, Play, RotateCcw, ChevronDown, ChevronUp, Radio, Disc, MapPin } from 'lucide-react';

// Standard Earth Radius in km
const R_EARTH = 6371.0;
const DEFAULT_START_TIME = 51744.4; // s (intended Arizona ground station pass start time)

const GROUND_TARGET_PRESETS = [
  { id: 0, name: 'Prescott (ERAU Campus, 34.62°N, 112.45°W)', lat: 34.6178, lon: -112.45 },
  { id: 1, name: 'Sedona / Flagstaff (34.94°N, 111.75°W)', lat: 34.9439, lon: -111.75 },
  { id: 2, name: 'Verde Valley (34.37°N, 111.45°W)', lat: 34.3659, lon: -111.45 }
];

export function SADCPanel() {
  const { status, data: rawData, logs, sendCommand, clearData } = useWebSocket('sadc');
  const data = rawData as SADCData | null;

  const [activeTab, setActiveTab] = useState<'overpass' | '3d' | 'attitude' | 'torques'>('overpass');
  const [isSolving, setIsSolving] = useState(false);
  const [terminalExpanded, setTerminalExpanded] = useState(false);
  const terminalEndRef = useRef<HTMLDivElement>(null);

  // Simulation Horizon & Targeting Controls
  const [startTime, setStartTime] = useState<number>(DEFAULT_START_TIME);
  const [duration, setDuration] = useState<number>(300.0);
  const [dt, setDt] = useState<number>(0.01);
  const [recordInterval, setRecordInterval] = useState<number>(10);
  const [targetMode, setTargetMode] = useState<'ground' | 'nadir'>('ground');
  const [targetIdx, setTargetIdx] = useState<number>(0);

  // Initial Attitude Perturbation (deg)
  const [initRollDeg, setInitRollDeg] = useState<number>(0.0);
  const [initPitchDeg, setInitPitchDeg] = useState<number>(0.0);
  const [initYawDeg, setInitYawDeg] = useState<number>(0.0);

  useEffect(() => {
    if (terminalExpanded && terminalEndRef.current) {
      terminalEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [logs, terminalExpanded]);

  useEffect(() => {
    if (data?.status === 'completed' || data?.error) {
      setIsSolving(false);
    }
  }, [data]);

  const handleSimulate = () => {
    clearData();
    setIsSolving(true);

    const payload: Record<string, unknown> = {
      start_time: startTime,
      duration,
      dt,
      record_interval: recordInterval,
      target_mode: targetMode,
      target_idx: targetIdx
    };

    if (Math.abs(initRollDeg) > 0.01 || Math.abs(initPitchDeg) > 0.01 || Math.abs(initYawDeg) > 0.01) {
      const r = (initRollDeg * Math.PI) / 360.0;
      const p = (initPitchDeg * Math.PI) / 360.0;
      const y = (initYawDeg * Math.PI) / 360.0;
      const q1 = Math.sin(r) * Math.cos(p) * Math.cos(y) - Math.cos(r) * Math.sin(p) * Math.sin(y);
      const q2 = Math.cos(r) * Math.sin(p) * Math.cos(y) + Math.sin(r) * Math.cos(p) * Math.sin(y);
      const q3 = Math.cos(r) * Math.cos(p) * Math.sin(y) - Math.sin(r) * Math.sin(p) * Math.cos(y);
      const q4 = Math.cos(r) * Math.cos(p) * Math.cos(y) + Math.sin(r) * Math.sin(p) * Math.sin(y);
      payload.q0_offset = [q1, q2, q3, q4];
    }

    sendCommand('simulate', payload);
  };

  const handleReset = () => {
    clearData();
    setStartTime(DEFAULT_START_TIME);
    sendCommand('reset', { start_time: DEFAULT_START_TIME });
  };

  // Telemetry computations
  const time = data?.time || [];
  const alt = data?.altitude || [];
  const currentAlt = alt.length > 0 ? alt[alt.length - 1] : 207.0;
  const lastIdx = (time.length || 1) - 1;

  const currentR = data?.euler_actual_r?.[lastIdx] ?? 0.0;
  const currentP = data?.euler_actual_p?.[lastIdx] ?? 0.0;
  const currentY = data?.euler_actual_y?.[lastIdx] ?? 0.0;

  const curErrR = data?.euler_error_r?.[lastIdx] ?? 0.0;
  const curErrP = data?.euler_error_p?.[lastIdx] ?? 0.0;
  const curErrY = data?.euler_error_y?.[lastIdx] ?? 0.0;
  const totalAttError = Math.sqrt(curErrR * curErrR + curErrP * curErrP + curErrY * curErrY);

  const curVelX = data?.vel_x?.[lastIdx] ?? 0.0;
  const curVelY = data?.vel_y?.[lastIdx] ?? 0.0;
  const curVelZ = data?.vel_z?.[lastIdx] ?? 0.0;
  const currentVelMag = Math.sqrt(curVelX * curVelX + curVelY * curVelY + curVelZ * curVelZ);

  // Ground pass overpass stats
  const elevations = data?.gs_elevation || [];
  const peakElevation = elevations.length > 0 ? Math.max(...elevations) : 77.4;
  const ranges = data?.gs_range || [];
  const minSlantRange = ranges.length > 0 ? Math.min(...ranges) : 207.0;
  const currentElevation = elevations.length > 0 ? elevations[lastIdx] : 53.3;

  // ---------------------------------------------------------------------------
  // 3D Visualizations
  // ---------------------------------------------------------------------------
  const generateEarthSphere = () => {
    const latSteps = 16;
    const lonSteps = 32;
    const x: number[] = [];
    const y: number[] = [];
    const z: number[] = [];

    for (let i = 0; i <= latSteps; i++) {
      const lat = (i / latSteps) * Math.PI - Math.PI / 2;
      for (let j = 0; j <= lonSteps; j++) {
        const lon = (j / lonSteps) * 2 * Math.PI;
        x.push(R_EARTH * Math.cos(lat) * Math.cos(lon));
        y.push(R_EARTH * Math.cos(lat) * Math.sin(lon));
        z.push(R_EARTH * Math.sin(lat));
      }
    }
    return { x, y, z };
  };

  const earthMesh = generateEarthSphere();

  // Active ground station ECI position
  const activeGsPos = data?.gs_eci?.[targetIdx] || [1420.0, 5080.0, 3600.0];
  const lastScPos = data?.sc_pos || [-1142.0, 5306.0, 3715.0];

  // Satellite Body Triad
  const dcm = data?.sc_dcm || [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
  ];
  const axisScale = 2.0;
  const triadTraces = [
    {
      x: [0, dcm[0][0] * axisScale],
      y: [0, dcm[1][0] * axisScale],
      z: [0, dcm[2][0] * axisScale],
      mode: 'lines+text' as const,
      line: { color: '#FF453A', width: 5 },
      name: 'X_b (Cross-Track)',
      text: ['', 'X_b'],
      textposition: 'top right' as const
    },
    {
      x: [0, dcm[0][1] * axisScale],
      y: [0, dcm[1][1] * axisScale],
      z: [0, dcm[2][1] * axisScale],
      mode: 'lines+text' as const,
      line: { color: '#32D74B', width: 5 },
      name: 'Y_b (Roll Axis)',
      text: ['', 'Y_b'],
      textposition: 'top right' as const
    },
    {
      x: [0, dcm[0][2] * axisScale],
      y: [0, dcm[1][2] * axisScale],
      z: [0, dcm[2][2] * axisScale],
      mode: 'lines+text' as const,
      line: { color: '#0A84FF', width: 5 },
      name: 'Z_b (Target Boresight)',
      text: ['', 'Z_b (Boresight)'],
      textposition: 'top right' as const
    }
  ];

  return (
    <div className="flex h-full w-full bg-background overflow-hidden font-sans">
      {/* Sidebar Controls */}
      <aside className="w-88 border-r border-border bg-surface backdrop-blur-xl flex flex-col shrink-0 overflow-y-auto">
        <div className="p-4 border-b border-border flex items-center justify-between">
          <div className="flex items-center gap-2">
            <Orbit className="text-accent-blue" size={18} />
            <h2 className="font-semibold text-sm tracking-wide uppercase text-text-primary">SADC Overpass Engine</h2>
          </div>
          <span className={`text-[10px] font-mono px-2 py-0.5 rounded-full border ${status === 'CONNECTED' ? 'bg-green-500/10 text-green-400 border-green-500/30' : 'bg-red-500/10 text-red-400 border-red-500/30'}`}>
            {status}
          </span>
        </div>

        <div className="p-4 flex-1 space-y-4">
          {/* Ground Target Location Selector */}
          <div className="control-group">
            <div className="panel-title text-[11px] mb-2 flex items-center gap-1.5 text-accent-cyan">
              <MapPin size={13} /> Desired Ground Target Location
            </div>
            <div className="space-y-2">
              <div className="control-item">
                <label>Target Station</label>
                <select
                  value={targetIdx}
                  onChange={(e) => setTargetIdx(Number(e.target.value))}
                  className="bg-black/50 border border-border text-xs rounded-lg px-2.5 py-1.5 font-mono text-text-primary"
                >
                  {GROUND_TARGET_PRESETS.map((tgt) => (
                    <option key={tgt.id} value={tgt.id}>
                      {tgt.name}
                    </option>
                  ))}
                </select>
              </div>

              <div className="control-item">
                <label>Attitude Pointing Mode</label>
                <div className="grid grid-cols-2 gap-2 mt-1">
                  <button
                    onClick={() => setTargetMode('ground')}
                    className={`py-1.5 px-2 rounded-lg text-xs font-medium border transition-all ${targetMode === 'ground' ? 'bg-accent-blue/20 text-accent-cyan border-accent-blue' : 'bg-black/30 text-text-secondary border-border hover:text-text-primary'}`}
                  >
                    Target Tracking
                  </button>
                  <button
                    onClick={() => setTargetMode('nadir')}
                    className={`py-1.5 px-2 rounded-lg text-xs font-medium border transition-all ${targetMode === 'nadir' ? 'bg-accent-blue/20 text-accent-cyan border-accent-blue' : 'bg-black/30 text-text-secondary border-border hover:text-text-primary'}`}
                  >
                    Nadir Pointing
                  </button>
                </div>
              </div>
            </div>
          </div>

          {/* Start Time & Horizon Controls */}
          <div className="control-group">
            <div className="panel-title text-[11px] mb-2 flex items-center gap-1.5 text-accent-blue">
              <Disc size={13} /> Simulation Epoch & Horizon
            </div>
            <div className="space-y-2">
              <div className="control-item">
                <div className="flex justify-between items-center mb-1">
                  <label className="mb-0">Simulation Start Time (s)</label>
                  <span className="text-[10px] text-accent-cyan font-mono">Overpass Epoch</span>
                </div>
                <input
                  type="number"
                  value={startTime}
                  step={10.0}
                  min={0.0}
                  onChange={(e) => setStartTime(Number(e.target.value))}
                  className="font-mono text-accent-cyan font-bold"
                />
                <span className="text-[10px] text-text-secondary mt-1">
                  {startTime === DEFAULT_START_TIME ? '★ 51,744.4s: Exact Arizona Overpass Window' : `T+${startTime.toFixed(1)} s`}
                </span>
              </div>

              <div className="grid grid-cols-2 gap-2 pt-1">
                <div className="control-item">
                  <label>Duration (s)</label>
                  <input
                    type="number"
                    value={duration}
                    step={50}
                    min={10}
                    max={3600}
                    onChange={(e) => setDuration(Math.max(10, Number(e.target.value)))}
                  />
                </div>
                <div className="control-item">
                  <label>Timestep dt (s)</label>
                  <input
                    type="number"
                    value={dt}
                    step={0.005}
                    min={0.001}
                    max={0.05}
                    onChange={(e) => setDt(Math.max(0.001, Number(e.target.value)))}
                  />
                </div>
              </div>

              <div className="control-item pt-1">
                <div className="flex justify-between text-[11px] text-text-secondary">
                  <span>Downsample Decimation</span>
                  <span className="font-mono">{recordInterval}x</span>
                </div>
                <input
                  type="range"
                  min={1}
                  max={25}
                  value={recordInterval}
                  onChange={(e) => setRecordInterval(Number(e.target.value))}
                />
              </div>
            </div>
          </div>

          {/* Initial Attitude Offsets (Optional Perturbations) */}
          <div className="control-group">
            <div className="panel-title text-[11px] mb-2 flex items-center gap-1.5 text-accent-green">
              <Compass size={13} /> Initial Attitude Offsets (Deg)
            </div>
            <div className="grid grid-cols-3 gap-1.5">
              <div className="control-item">
                <label>Roll (φ)</label>
                <input
                  type="number"
                  value={initRollDeg}
                  step={1}
                  onChange={(e) => setInitRollDeg(Number(e.target.value))}
                />
              </div>
              <div className="control-item">
                <label>Pitch (θ)</label>
                <input
                  type="number"
                  value={initPitchDeg}
                  step={1}
                  onChange={(e) => setInitPitchDeg(Number(e.target.value))}
                />
              </div>
              <div className="control-item">
                <label>Yaw (ψ)</label>
                <input
                  type="number"
                  value={initYawDeg}
                  step={1}
                  onChange={(e) => setInitYawDeg(Number(e.target.value))}
                />
              </div>
            </div>
          </div>

          {/* Actions */}
          <div className="pt-1 flex flex-col gap-2">
            <button
              onClick={handleSimulate}
              disabled={isSolving || status !== 'CONNECTED'}
              className="w-full flex items-center justify-center gap-2 py-2 px-4 rounded-lg bg-accent-blue text-white font-medium text-xs shadow-lg shadow-accent-blue/20 hover:bg-accent-blue-hover disabled:opacity-50 transition-all duration-200"
            >
              {isSolving ? (
                <>
                  <div className="w-3.5 h-3.5 border-2 border-white/20 border-t-white rounded-full animate-spin" />
                  <span>Integrating Ground Track...</span>
                </>
              ) : (
                <>
                  <Play size={14} /> Execute Overpass Simulation
                </>
              )}
            </button>

            <button
              onClick={handleReset}
              className="w-full flex items-center justify-center gap-2 py-1.5 px-4 rounded-lg bg-surface-solid border border-border text-text-secondary hover:text-text-primary text-xs transition-all duration-200"
            >
              <RotateCcw size={13} /> Reset to startTime=51744.4s
            </button>
          </div>
        </div>
      </aside>

      {/* Main Content Area */}
      <main className="flex-1 flex flex-col overflow-hidden relative">
        {/* Top KPI Telemetry Banner */}
        <header className="p-3 border-b border-border bg-surface/50 backdrop-blur-md shrink-0 flex items-center justify-between">
          <div className="grid grid-cols-2 md:grid-cols-5 gap-3 w-full max-w-5xl">
            <div className="card p-2 flex flex-col">
              <span className="text-[10px] uppercase font-mono text-text-secondary">Overpass Epoch</span>
              <span className="text-sm font-bold font-mono text-accent-cyan">T+{startTime.toFixed(1)} s</span>
              <span className="text-[10px] text-text-secondary font-mono">Alt: {currentAlt.toFixed(1)} km</span>
            </div>
            <div className="card p-2 flex flex-col">
              <span className="text-[10px] uppercase font-mono text-text-secondary">Peak Pass Elevation</span>
              <span className="text-sm font-bold font-mono text-accent-green">{peakElevation.toFixed(1)}°</span>
              <span className="text-[10px] text-text-secondary font-mono">Current: {currentElevation.toFixed(1)}°</span>
            </div>
            <div className="card p-2 flex flex-col">
              <span className="text-[10px] uppercase font-mono text-text-secondary">Closest Range</span>
              <span className="text-sm font-bold font-mono text-yellow-400">{minSlantRange.toFixed(1)} km</span>
              <span className="text-[10px] text-text-secondary font-mono">Speed: {currentVelMag.toFixed(2)} km/s</span>
            </div>
            <div className="card p-2 flex flex-col">
              <span className="text-[10px] uppercase font-mono text-text-secondary">Pointing Error</span>
              <span className="text-sm font-bold font-mono text-accent-blue">{totalAttError.toFixed(3)}°</span>
              <span className="text-[10px] text-text-secondary font-mono">[{currentR.toFixed(1)}°, {currentP.toFixed(1)}°, {currentY.toFixed(1)}°]</span>
            </div>
            <div className="card p-2 flex flex-col">
              <span className="text-[10px] uppercase font-mono text-text-secondary">Pass Contact Status</span>
              <span className={`text-sm font-bold font-mono ${currentElevation > 10.0 ? 'text-accent-green' : 'text-text-secondary'}`}>
                {currentElevation > 10.0 ? 'Active LOS (>10°)' : 'Below Horizon'}
              </span>
              <span className="text-[10px] text-text-secondary font-mono">{GROUND_TARGET_PRESETS[targetIdx].name.split(' ')[0]}</span>
            </div>
          </div>

          {/* Tab Selector */}
          <div className="hidden lg:flex items-center gap-1 bg-black/40 p-1 rounded-lg border border-border">
            <button
              onClick={() => setActiveTab('overpass')}
              className={`px-3 py-1 rounded-md text-xs font-medium transition-all ${activeTab === 'overpass' ? 'bg-surface-solid text-text-primary' : 'text-text-secondary hover:text-text-primary'}`}
            >
              Overpass Telemetry
            </button>
            <button
              onClick={() => setActiveTab('3d')}
              className={`px-3 py-1 rounded-md text-xs font-medium transition-all ${activeTab === '3d' ? 'bg-surface-solid text-text-primary' : 'text-text-secondary hover:text-text-primary'}`}
            >
              3D Orbit & Targets
            </button>
            <button
              onClick={() => setActiveTab('attitude')}
              className={`px-3 py-1 rounded-md text-xs font-medium transition-all ${activeTab === 'attitude' ? 'bg-surface-solid text-text-primary' : 'text-text-secondary hover:text-text-primary'}`}
            >
              Attitude & Rates
            </button>
            <button
              onClick={() => setActiveTab('torques')}
              className={`px-3 py-1 rounded-md text-xs font-medium transition-all ${activeTab === 'torques' ? 'bg-surface-solid text-text-primary' : 'text-text-secondary hover:text-text-primary'}`}
            >
              LQR Torques
            </button>
          </div>
        </header>

        {/* Dynamic Visualizations Area */}
        <section className="flex-1 p-4 overflow-y-auto min-h-0">
          {activeTab === 'overpass' && (
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-4 h-full min-h-[550px]">
              {/* Ground Station Elevation Profile */}
              <div className="card p-2 flex flex-col min-h-[260px]">
                <PlotCard
                  title={`Ground Station Elevation Profile [deg] (${GROUND_TARGET_PRESETS[targetIdx].name})`}
                  data={[
                    {
                      x: time,
                      y: data?.gs_elevation || [],
                      mode: 'lines',
                      name: 'Elevation (°)',
                      line: { color: '#32D74B', width: 2.5 }
                    },
                    // 10° Acquisition of Signal (AOS) Horizon Mask
                    {
                      x: time,
                      y: time.map(() => 10.0),
                      mode: 'lines',
                      name: '10° Mask Threshold',
                      line: { color: '#FF453A', width: 1.5, dash: 'dash' }
                    }
                  ]}
                  layout={{
                    xaxis: { title: 'Simulation Time (s)', gridcolor: 'rgba(255,255,255,0.08)' },
                    yaxis: { title: 'Elevation (deg)', range: [0, 90], gridcolor: 'rgba(255,255,255,0.08)' }
                  }}
                />
              </div>

              {/* Slant Range Profile */}
              <div className="card p-2 flex flex-col min-h-[260px]">
                <PlotCard
                  title="Slant Range to Ground Target (km)"
                  data={[
                    {
                      x: time,
                      y: data?.gs_range || [],
                      mode: 'lines',
                      name: 'Slant Range',
                      line: { color: '#64D2FF', width: 2.5 }
                    }
                  ]}
                  layout={{
                    xaxis: { title: 'Simulation Time (s)', gridcolor: 'rgba(255,255,255,0.08)' },
                    yaxis: { title: 'Range (km)', gridcolor: 'rgba(255,255,255,0.08)' }
                  }}
                />
              </div>

              {/* Dynamic Target Pointing Error */}
              <div className="card p-2 flex flex-col min-h-[260px]">
                <PlotCard
                  title="Target Pointing Error (Euler Residuals) [deg]"
                  data={[
                    { x: time, y: data?.euler_error_r || [], mode: 'lines', name: 'ΔRoll', line: { color: '#FF453A', width: 2 } },
                    { x: time, y: data?.euler_error_p || [], mode: 'lines', name: 'ΔPitch', line: { color: '#32D74B', width: 2 } },
                    { x: time, y: data?.euler_error_y || [], mode: 'lines', name: 'ΔYaw', line: { color: '#0A84FF', width: 2 } }
                  ]}
                  layout={{
                    xaxis: { title: 'Simulation Time (s)', gridcolor: 'rgba(255,255,255,0.08)' },
                    yaxis: { title: 'Error (deg)', gridcolor: 'rgba(255,255,255,0.08)' }
                  }}
                />
              </div>

              {/* Ground Station Azimuth Tracking */}
              <div className="card p-2 flex flex-col min-h-[260px]">
                <PlotCard
                  title="Ground Station Azimuth Tracking Angle [deg]"
                  data={[
                    {
                      x: time,
                      y: data?.gs_azimuth || [],
                      mode: 'lines',
                      name: 'Azimuth (Clockwise from North)',
                      line: { color: '#FFD60A', width: 2 }
                    }
                  ]}
                  layout={{
                    xaxis: { title: 'Simulation Time (s)', gridcolor: 'rgba(255,255,255,0.08)' },
                    yaxis: { title: 'Azimuth (deg)', range: [0, 360], gridcolor: 'rgba(255,255,255,0.08)' }
                  }}
                />
              </div>
            </div>
          )}

          {activeTab === '3d' && (
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-4 h-full min-h-[500px]">
              {/* 3D Earth Orbit Track & Ground Targets */}
              <div className="card p-2 flex flex-col h-full min-h-[400px]">
                <PlotCard
                  title="3D Keplerian Orbit & Ground Target Locations"
                  data={[
                    // Earth wireframe
                    {
                      type: 'scatter3d',
                      mode: 'markers',
                      x: earthMesh.x,
                      y: earthMesh.y,
                      z: earthMesh.z,
                      marker: { size: 1.5, color: '#1E3A8A', opacity: 0.35 },
                      name: 'Earth Globe',
                      hoverinfo: 'skip'
                    },
                    // Spacecraft orbit track
                    {
                      type: 'scatter3d',
                      mode: 'lines',
                      x: data?.orbit_x || [],
                      y: data?.orbit_y || [],
                      z: data?.orbit_z || [],
                      line: { color: '#0A84FF', width: 4 },
                      name: 'Spacecraft Orbit'
                    },
                    // Spacecraft position
                    {
                      type: 'scatter3d',
                      mode: 'markers+text',
                      x: [lastScPos[0]],
                      y: [lastScPos[1]],
                      z: [lastScPos[2]],
                      marker: { size: 8, color: '#FFD60A', symbol: 'diamond' },
                      text: ['Spacecraft'],
                      textposition: 'top right',
                      name: 'Spacecraft'
                    },
                    // Active ground station
                    {
                      type: 'scatter3d',
                      mode: 'markers+text',
                      x: [activeGsPos[0]],
                      y: [activeGsPos[1]],
                      z: [activeGsPos[2]],
                      marker: { size: 9, color: '#32D74B', symbol: 'circle' },
                      text: [GROUND_TARGET_PRESETS[targetIdx].name.split(' ')[0]],
                      textposition: 'bottom center',
                      name: 'Target Ground Site'
                    },
                    // Line of Sight Beam
                    {
                      type: 'scatter3d',
                      mode: 'lines',
                      x: [lastScPos[0], activeGsPos[0]],
                      y: [lastScPos[1], activeGsPos[1]],
                      z: [lastScPos[2], activeGsPos[2]],
                      line: { color: '#32D74B', width: 3, dash: 'dash' },
                      name: 'Line of Sight'
                    }
                  ]}
                  layout={{
                    scene: {
                      xaxis: { title: 'X (km)', gridcolor: 'rgba(255,255,255,0.08)' },
                      yaxis: { title: 'Y (km)', gridcolor: 'rgba(255,255,255,0.08)' },
                      zaxis: { title: 'Z (km)', gridcolor: 'rgba(255,255,255,0.08)' },
                      aspectmode: 'data'
                    }
                  }}
                />
              </div>

              {/* 3D Attitude Body Frame */}
              <div className="card p-2 flex flex-col h-full min-h-[400px]">
                <PlotCard
                  title="Spacecraft Attitude Frame (Body DCM Triad)"
                  data={triadTraces}
                  layout={{
                    scene: {
                      xaxis: { title: 'Inertial X', range: [-2.5, 2.5], gridcolor: 'rgba(255,255,255,0.08)' },
                      yaxis: { title: 'Inertial Y', range: [-2.5, 2.5], gridcolor: 'rgba(255,255,255,0.08)' },
                      zaxis: { title: 'Inertial Z', range: [-2.5, 2.5], gridcolor: 'rgba(255,255,255,0.08)' },
                      aspectmode: 'cube'
                    }
                  }}
                />
              </div>
            </div>
          )}

          {activeTab === 'attitude' && (
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-4 h-full min-h-[550px]">
              {/* Euler Angles */}
              <div className="card p-2 flex flex-col min-h-[260px]">
                <PlotCard
                  title="Euler 3-2-1 Angles (Roll, Pitch, Yaw) [deg]"
                  data={[
                    { x: time, y: data?.euler_actual_r || [], mode: 'lines', name: 'Roll Actual (φ)', line: { color: '#FF453A', width: 2 } },
                    { x: time, y: data?.euler_cmd_r || [], mode: 'lines', name: 'Roll Target', line: { color: '#FF453A', width: 1.5, dash: 'dash' } },
                    { x: time, y: data?.euler_actual_p || [], mode: 'lines', name: 'Pitch Actual (θ)', line: { color: '#32D74B', width: 2 } },
                    { x: time, y: data?.euler_cmd_p || [], mode: 'lines', name: 'Pitch Target', line: { color: '#32D74B', width: 1.5, dash: 'dash' } },
                    { x: time, y: data?.euler_actual_y || [], mode: 'lines', name: 'Yaw Actual (ψ)', line: { color: '#0A84FF', width: 2 } },
                    { x: time, y: data?.euler_cmd_y || [], mode: 'lines', name: 'Yaw Target', line: { color: '#0A84FF', width: 1.5, dash: 'dash' } }
                  ]}
                  layout={{
                    xaxis: { title: 'Time (s)', gridcolor: 'rgba(255,255,255,0.08)' },
                    yaxis: { title: 'Angle (deg)', gridcolor: 'rgba(255,255,255,0.08)' }
                  }}
                />
              </div>

              {/* Body Rates */}
              <div className="card p-2 flex flex-col min-h-[260px]">
                <PlotCard
                  title="Body Angular Rates (ω_x, ω_y, ω_z) [deg/s]"
                  data={[
                    { x: time, y: data?.omega_x || [], mode: 'lines', name: 'ω_x', line: { color: '#FF453A', width: 2 } },
                    { x: time, y: data?.omega_y || [], mode: 'lines', name: 'ω_y', line: { color: '#32D74B', width: 2 } },
                    { x: time, y: data?.omega_z || [], mode: 'lines', name: 'ω_z', line: { color: '#0A84FF', width: 2 } }
                  ]}
                  layout={{
                    xaxis: { title: 'Time (s)', gridcolor: 'rgba(255,255,255,0.08)' },
                    yaxis: { title: 'Angular Rate (deg/s)', gridcolor: 'rgba(255,255,255,0.08)' }
                  }}
                />
              </div>

              {/* Error Quaternion Components */}
              <div className="card p-2 flex flex-col min-h-[260px] lg:col-span-2">
                <PlotCard
                  title="Error Quaternion Tracking Components (qe_1, qe_2, qe_3, qe_4)"
                  data={[
                    { x: time, y: data?.qe1 || [], mode: 'lines', name: 'qe_1', line: { color: '#FF453A', width: 1.5 } },
                    { x: time, y: data?.qe2 || [], mode: 'lines', name: 'qe_2', line: { color: '#32D74B', width: 1.5 } },
                    { x: time, y: data?.qe3 || [], mode: 'lines', name: 'qe_3', line: { color: '#0A84FF', width: 1.5 } },
                    { x: time, y: data?.qe4 || [], mode: 'lines', name: 'qe_4 (Scalar)', line: { color: '#FFD60A', width: 2 } }
                  ]}
                  layout={{
                    xaxis: { title: 'Time (s)', gridcolor: 'rgba(255,255,255,0.08)' },
                    yaxis: { title: 'Quaternion Component', gridcolor: 'rgba(255,255,255,0.08)' }
                  }}
                />
              </div>
            </div>
          )}

          {activeTab === 'torques' && (
            <div className="grid grid-cols-1 lg:grid-cols-2 gap-4 h-full min-h-[550px]">
              {/* Control Torques */}
              <div className="card p-2 flex flex-col min-h-[260px]">
                <PlotCard
                  title="LQR Control Torques T_ctrl [N·m]"
                  data={[
                    { x: time, y: data?.torque_ctrl_x || [], mode: 'lines', name: 'T_ctrl_x', line: { color: '#FF453A', width: 2 } },
                    { x: time, y: data?.torque_ctrl_y || [], mode: 'lines', name: 'T_ctrl_y', line: { color: '#32D74B', width: 2 } },
                    { x: time, y: data?.torque_ctrl_z || [], mode: 'lines', name: 'T_ctrl_z', line: { color: '#0A84FF', width: 2 } }
                  ]}
                  layout={{
                    xaxis: { title: 'Time (s)', gridcolor: 'rgba(255,255,255,0.08)' },
                    yaxis: { title: 'Torque (N·m)', gridcolor: 'rgba(255,255,255,0.08)' }
                  }}
                />
              </div>

              {/* Gravity Gradient Torques */}
              <div className="card p-2 flex flex-col min-h-[260px]">
                <PlotCard
                  title="Gravity-Gradient Torques T_gg [N·m]"
                  data={[
                    { x: time, y: data?.torque_gg_x || [], mode: 'lines', name: 'T_gg_x', line: { color: '#FF9F0A', width: 2 } },
                    { x: time, y: data?.torque_gg_y || [], mode: 'lines', name: 'T_gg_y', line: { color: '#BF5AF2', width: 2 } },
                    { x: time, y: data?.torque_gg_z || [], mode: 'lines', name: 'T_gg_z', line: { color: '#64D2FF', width: 2 } }
                  ]}
                  layout={{
                    xaxis: { title: 'Time (s)', gridcolor: 'rgba(255,255,255,0.08)' },
                    yaxis: { title: 'Torque (N·m)', gridcolor: 'rgba(255,255,255,0.08)' }
                  }}
                />
              </div>
            </div>
          )}
        </section>

        {/* Collapsible Telemetry Logs Terminal */}
        <div className="border-t border-border bg-surface shrink-0">
          <button
            onClick={() => setTerminalExpanded(!terminalExpanded)}
            className="w-full px-4 py-2 flex items-center justify-between text-xs text-text-secondary hover:text-text-primary transition-colors"
          >
            <div className="flex items-center gap-2">
              <Radio size={14} className={status === 'CONNECTED' ? 'text-accent-green' : 'text-accent-red'} />
              <span className="font-mono uppercase font-semibold">Gateway Telemetry Stream (SADC Overpass)</span>
              <span className="text-[10px] bg-white/5 px-2 py-0.5 rounded font-mono">{logs.length} events</span>
            </div>
            {terminalExpanded ? <ChevronDown size={14} /> : <ChevronUp size={14} />}
          </button>

          {terminalExpanded && (
            <div className="h-44 p-3 font-mono text-[11px] overflow-y-auto bg-black/60 space-y-1">
              {logs.map((log, index) => (
                <div key={index} className="flex gap-2">
                  <span className="text-text-secondary">{log.timestamp}</span>
                  <span className={log.message.includes('Error') ? 'text-accent-red' : log.message.includes('[SADC]') ? 'text-accent-cyan' : 'text-text-primary'}>
                    {log.message}
                  </span>
                </div>
              ))}
              <div ref={terminalEndRef} />
            </div>
          )}
        </div>
      </main>
    </div>
  );
}
