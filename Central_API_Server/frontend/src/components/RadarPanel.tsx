import React, { useState, useEffect, useRef } from 'react';
import { useWebSocket } from '../hooks/useWebSocket';
import { PlotCard } from './PlotCard';

export function RadarPanel() {
  const { status, data, logs, sendCommand } = useWebSocket('radar');
  
  const [params, setParams] = useState({
    mode: 1, freq: 10.0, pwr: 50.0, tau: 10.0, prf: 5000, bw: 50,
    ny: 16, nz: 16, taper: 1, az: 45, el: 15, steerType: 1
  });

  const [tgtParams, setTgtParams] = useState({
    x: 1000, y: 500, z: 5000, vx: 250
  });

  const [activeTab, setActiveTab] = useState('fundamentals');
  const [isSimulating, setIsSimulating] = useState(false);
  const [terminalExpanded, setTerminalExpanded] = useState(false);
  const terminalEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (terminalExpanded && terminalEndRef.current) {
      terminalEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [logs, terminalExpanded]);
  
  // Accumulate track history & kinematics
  const [trackHistory, setTrackHistory] = useState<{x: number[], y: number[], z: number[]}>({x: [], y: [], z: []});
  const [rangeErrors, setRangeErrors] = useState<number[]>([]);
  const [velErrors, setVelErrors] = useState<number[]>([]);
  const [rd3dHistory, setRd3dHistory] = useState<{ r: number[], v: number[], t: number[] }>({ r: [], v: [], t: [] });
  const [thermalHistory, setThermalHistory] = useState<number[]>([]);
  const simTimeRef = useRef(0);

  useEffect(() => {
    if (data?.track_history_x && data?.track_history_y && data?.track_history_z) {
      // eslint-disable-next-line
      setTrackHistory(prev => {
        const nx = [...prev.x, data.track_history_x[0]].slice(-500); // keep last 500
        const ny = [...prev.y, data.track_history_y[0]].slice(-500);
        const nz = [...prev.z, data.track_history_z[0]].slice(-500);
        return { x: nx, y: ny, z: nz };
      });

      // Accumulate 3D Range-Doppler Trace History (AxRD3D)
      simTimeRef.current += 0.25;
      const t = simTimeRef.current;
      const x = data.track_history_x[0];
      const y = data.track_history_y[0];
      const z = data.track_history_z[0];
      const r = Math.sqrt(x*x + y*y + z*z) / 1000; // km
      const v = 250.0; // m/s nominal Doppler velocity
      setRd3dHistory(prev => ({
        r: [...prev.r, r].slice(-100),
        v: [...prev.v, v].slice(-100),
        t: [...prev.t, t].slice(-100)
      }));
    }

    // Accumulate EKF Filter Error Convergence
    if (data?.filter_range_error && data.filter_range_error.length > 0) {
      setRangeErrors(prev => [...prev, data.filter_range_error![0]].slice(-60));
    }
    if (data?.filter_velocity_error && data.filter_velocity_error.length > 0) {
      setVelErrors(prev => [...prev, data.filter_velocity_error![0]].slice(-60));
    }

    // Thermal simulation: Junction temp Tj approaches equilibrium (Bader 2004)
    const dutyFrac = (params.tau * 1e-6) * params.prf;
    const pAvgW = (params.pwr * 1e3) * dutyFrac;
    const targetTj = 25.0 + (pAvgW * 0.008); // deg C
    setThermalHistory(prev => {
      const last = prev.length ? prev[prev.length - 1] : 25.0;
      const next = last + 0.15 * (targetTj - last);
      return [...prev, next].slice(-60);
    });
  }, [data, params]);

  // Responsive polling mechanism to simulate real-time engine
  useEffect(() => {
    if (isSimulating && status === 'CONNECTED') {
      const timer = setTimeout(() => {
        sendCommand('scan', {
          mode: params.mode,
          freq: params.freq * 1e9,
          pwr: params.pwr * 1e3,
          tau: params.tau * 1e-6,
          prf: params.prf,
          bw: params.bw * 1e6,
          Ny: params.ny,
          Nz: params.nz,
          taper: params.taper,
          steerAz: params.az,
          steerEl: params.el,
          steerType: params.steerType
        });
      }, 250); // ~4 fps tick rate (avoids Plotly/WebGL memory crash)
      return () => clearTimeout(timer);
    }
  }, [isSimulating, data, status, params, sendCommand]);



  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setParams(prev => ({ ...prev, [name]: parseFloat(value) }));
  };

  const tabs = [
    { id: 'fundamentals', label: 'Fundamentals' },
    { id: 'array', label: 'Array Synthesis' },
    { id: 'ew', label: 'Sensor & EW' },
    { id: 'signal', label: 'Signal Proc & STAP' },
    { id: 'c2', label: 'C2 Operator' },
    { id: 'pointer', label: 'Pointer Dynamics' }
  ];

  return (
    <div className="flex flex-col h-full w-full overflow-hidden">
      
      {/* Main Work Area: Sidebar Controls + Viewport */}
      <div className="flex-1 grid grid-cols-[360px_1fr] gap-6 p-6 pb-2 min-h-0 overflow-hidden">
        
        {/* Sidebar Controls (Glass Panel) */}
        <aside className="glass-panel overflow-y-auto flex flex-col p-6 gap-6 h-full min-h-0">
          {/* Connection Status Header */}
          <div className="flex items-center justify-between pb-5 border-b border-border/50 shrink-0">
            <div className="flex items-center gap-3">
              <div className={`status-indicator ${
                status === 'CONNECTED' ? 'status-connected' : 
                status === 'CONNECTING' ? 'status-connecting' : 'status-disconnected'
              }`} />
              <span className="text-xs font-bold text-text-secondary uppercase tracking-widest">
                {status === 'CONNECTED' ? 'Data Link Active' : 'Offline'}
              </span>
            </div>
          </div>

        {/* Form Controls */}
        <div>
          <div className="panel-title">Radar Parameters</div>
          <div className="control-group">
            <div className="grid grid-cols-2 gap-3">
              <div className="control-item">
                <label>Mode</label>
                <select name="mode" value={params.mode} onChange={handleChange}>
                  <option value={1}>Search</option>
                  <option value={2}>Track</option>
                  <option value={3}>SAR</option>
                </select>
              </div>
              <div className="control-item">
                <label>Freq (GHz)</label>
                <input type="number" name="freq" value={params.freq} onChange={handleChange} step="0.1" />
              </div>
              <div className="control-item">
                <label>Pwr (kW)</label>
                <input type="number" name="pwr" value={params.pwr} onChange={handleChange} />
              </div>
              <div className="control-item">
                <label>Tau (μs)</label>
                <input type="number" name="tau" value={params.tau} onChange={handleChange} />
              </div>
              <div className="control-item">
                <label>PRF (Hz)</label>
                <input type="number" name="prf" value={params.prf} onChange={handleChange} />
              </div>
              <div className="control-item">
                <label>BW (MHz)</label>
                <input type="number" name="bw" value={params.bw} onChange={handleChange} />
              </div>
            </div>
          </div>
        </div>

        <div>
          <div className="panel-title">Array Synthesis</div>
          <div className="control-group">
            <div className="grid grid-cols-2 gap-3">
              <div className="control-item">
                <label>Ny (Cols)</label>
                <input type="number" name="ny" value={params.ny} onChange={handleChange} />
              </div>
              <div className="control-item">
                <label>Nz (Rows)</label>
                <input type="number" name="nz" value={params.nz} onChange={handleChange} />
              </div>
              <div className="control-item col-span-2">
                <label>Taper Window</label>
                <select name="taper" value={params.taper} onChange={handleChange}>
                  <option value={0}>Uniform</option>
                  <option value={1}>Hamming</option>
                  <option value={2}>Hann</option>
                </select>
              </div>
            </div>
          </div>
        </div>

        <div>
          <div className="panel-title">Beam Steering</div>
          <div className="control-group">
            <div className="grid grid-cols-2 gap-3">
              <div className="control-item col-span-2">
                <label>Steering Type</label>
                <select name="steerType" value={params.steerType} onChange={handleChange}>
                  <option value={0}>Broadside</option>
                  <option value={1}>Electronic (Phase)</option>
                  <option value={2}>Mechanical</option>
                </select>
              </div>
              <div className="control-item">
                <label>Azimuth (°)</label>
                <input type="number" name="az" value={params.az} onChange={handleChange} />
              </div>
              <div className="control-item">
                <label>Elevation (°)</label>
                <input type="number" name="el" value={params.el} onChange={handleChange} />
              </div>
            </div>
            
            <button 
              className="mt-4 w-full px-4 py-2 bg-accent-cyan/20 text-accent-cyan border border-accent-cyan/50 rounded hover:bg-accent-cyan/30 font-bold uppercase tracking-wider transition-colors"
              onClick={() => {
                const Rmax = 20000;
                const c = 299792458;
                const idealPrf = Math.floor(c / (2 * Rmax));
                const idealBw = 50; 
                setParams(prev => ({
                  ...prev,
                  prf: Math.min(idealPrf, 10000),
                  bw: idealBw,
                  pwr: 100
                }));
              }}
            >
              Auto-Optimize Parameters
            </button>
          </div>
        </div>

        <div>
          <div className="panel-title">Target Injector</div>
          <div className="control-group">
            <div className="grid grid-cols-2 gap-3">
              <div className="control-item">
                <label>X (m)</label>
                <input type="number" value={tgtParams.x} onChange={(e) => setTgtParams(prev => ({ ...prev, x: parseFloat(e.target.value) }))} />
              </div>
              <div className="control-item">
                <label>Y (m)</label>
                <input type="number" value={tgtParams.y} onChange={(e) => setTgtParams(prev => ({ ...prev, y: parseFloat(e.target.value) }))} />
              </div>
              <div className="control-item">
                <label>Z (m)</label>
                <input type="number" value={tgtParams.z} onChange={(e) => setTgtParams(prev => ({ ...prev, z: parseFloat(e.target.value) }))} />
              </div>
              <div className="control-item">
                <label>Vx (m/s)</label>
                <input type="number" value={tgtParams.vx} onChange={(e) => setTgtParams(prev => ({ ...prev, vx: parseFloat(e.target.value) }))} />
              </div>
            </div>
            <button 
              onClick={() => {
                sendCommand('add_target', { 
                  id: Math.floor(Math.random() * 1000000), 
                  x: tgtParams.x, y: tgtParams.y, z: tgtParams.z, 
                  vx: tgtParams.vx, vy: 0, vz: 0 
                });
              }}
              className="w-full mt-4 py-2 bg-white/5 border border-white/10 rounded-lg text-xs font-semibold hover:bg-white/10 transition-colors"
            >
              + ADD TARGET
            </button>
          </div>
        </div>

        <div className="mt-auto pt-4 shrink-0">
          <button 
            onClick={() => setIsSimulating(!isSimulating)} 
            disabled={status !== 'CONNECTED'} 
            className={`w-full py-3 rounded-lg font-bold transition-all ${
              isSimulating 
                ? 'bg-red-500/80 hover:bg-red-500 text-white border border-red-500 shadow-[0_0_15px_rgba(239,68,68,0.3)]'
                : 'btn-primary'
            }`}
          >
            {isSimulating ? '■ STOP SIMULATION' : '▶ RUN SIMULATION'}
          </button>
        </div>
      </aside>

      {/* Main Viewport */}
      <div className="flex flex-col h-full min-h-0 overflow-hidden bg-black/40 rounded-xl border border-border">
        
        {/* Progress Bar */}
        {isSimulating && (
          <div className="w-full h-1 bg-border rounded-full overflow-hidden shrink-0">
            <div className="h-full bg-accent-cyan rounded-full animate-pulse" style={{ width: '100%', animation: 'progress-slide 1.2s ease-in-out infinite' }} />
          </div>
        )}

        <div className="p-4 flex-1 flex flex-col gap-4 overflow-hidden min-h-0">
          {/* Header with Tabs and Live Telemetry Badges */}
          <div className="flex justify-between items-center border-b border-border pb-3 shrink-0">
            <div className="flex gap-2 overflow-x-auto pb-1">
              {tabs.map(t => (
                <button 
                  key={t.id}
                  className={`px-4 py-1.5 rounded-xl text-xs font-semibold transition-all duration-300 border whitespace-nowrap ${activeTab === t.id ? 'bg-surface-hover border-border-strong text-text-primary shadow-lg' : 'bg-surface border-border/50 text-text-secondary hover:text-text-primary hover:bg-white/[0.05]'}`}
                  onClick={() => setActiveTab(t.id)}
                >
                  {t.label}
                </button>
              ))}
            </div>

            {/* Live Telemetry KPI Badges */}
            <div className="flex items-center gap-3 text-xs font-mono shrink-0">
              <div className="bg-surface px-3 py-1 rounded-lg border border-border/50 flex items-center gap-2">
                <span className="text-text-secondary">DSP:</span>
                <span className={isSimulating ? 'text-accent-green font-bold animate-pulse' : 'text-text-secondary'}>
                  {isSimulating ? 'TRANSMITTING' : 'STANDBY'}
                </span>
              </div>
              <div className="bg-surface px-3 py-1 rounded-lg border border-border/50 flex items-center gap-2">
                <span className="text-text-secondary">Freq:</span>
                <span className="text-accent-cyan font-bold">{params.freq} GHz</span>
              </div>
              <div className="bg-surface px-3 py-1 rounded-lg border border-border/50 flex items-center gap-2">
                <span className="text-text-secondary">PRF:</span>
                <span className="text-text-primary font-bold">{params.prf} Hz</span>
              </div>
              <div className="bg-surface px-3 py-1 rounded-lg border border-border/50 flex items-center gap-2">
                <span className="text-text-secondary">Detections:</span>
                <span className="text-accent-green font-bold">
                  {data?.cfar_detections ? (data.cfar_detections as number[][]).flat().filter(Boolean).length : 0}
                </span>
              </div>
            </div>
          </div>
          
          <div className="flex-1 min-h-0 overflow-hidden">
            {activeTab === 'fundamentals' && (
              <div className="grid grid-cols-2 grid-rows-2 gap-6 flex-1 min-h-0 h-full">
                <div className="card p-2">
                  <PlotCard 
                    title="SNR vs Range (Radar Equation)" 
                    data={[
                      ...(data?.snr_range ? [{
                        x: Array.from({length: 100}, (_, i) => 1 + i * (19 / 99)),
                        y: data.snr_range,
                        mode: 'lines', name: 'SNR (dB)', line: { color: '#FFD60A', width: 2.5 }, fill: 'tozeroy', fillcolor: 'rgba(255, 214, 10, 0.1)'
                      }] : []),
                      {
                        x: [1, 20],
                        y: [13, 13],
                        mode: 'lines', name: 'Detection Threshold (13 dB)', line: { color: '#FF453A', width: 1.5, dash: 'dash' }
                      }
                    ]}
                    layout={{ xaxis: { title: 'Range (km)', gridcolor: '#2c2c2e' }, yaxis: { title: 'SNR (dB)', gridcolor: '#2c2c2e' } }}
                  />
                </div>
                <div className="card p-2">
                  <PlotCard 
                    title="Active Pulse Train Timing" 
                    data={(() => {
                      const priUs = Math.max(1, 1e6 / params.prf);
                      const tMaxUs = priUs * 3.5;
                      const numPts = 350;
                      const tVec = Array.from({length: numPts}, (_, i) => (i / (numPts - 1)) * tMaxUs);
                      const ampVec = tVec.map(t => (t % priUs <= params.tau ? 1.0 : 0.0));
                      return [{
                        x: tVec,
                        y: ampVec,
                        mode: 'lines', name: 'RF Pulse Envelope', line: { color: '#32D74B', width: 2 }, fill: 'tozeroy', fillcolor: 'rgba(50, 215, 75, 0.15)'
                      }];
                    })()}
                    layout={{ xaxis: { title: 'Time (μs)', gridcolor: '#2c2c2e' }, yaxis: { title: 'Normalized Amplitude', gridcolor: '#2c2c2e', range: [-0.1, 1.2] } }}
                  />
                </div>
                <div className="card p-2">
                  <PlotCard 
                    title="Transmitter Thermal Status" 
                    data={[
                      {
                        x: Array.from({length: Math.max(1, thermalHistory.length)}, (_, i) => i * 0.25),
                        y: thermalHistory.length ? thermalHistory : [25.0],
                        mode: 'lines', name: 'Junction Temp Tj (°C)', line: { color: '#FF9F0A', width: 2.5 }
                      },
                      {
                        x: [0, Math.max(15, thermalHistory.length * 0.25)],
                        y: [85, 85],
                        mode: 'lines', name: 'Safety Ceiling (85°C)', line: { color: '#FF453A', width: 1.5, dash: 'dot' }
                      }
                    ]}
                    layout={{ xaxis: { title: 'Elapsed Time (s)', gridcolor: '#2c2c2e' }, yaxis: { title: 'Temperature (°C)', gridcolor: '#2c2c2e', range: [20, 100] } }}
                  />
                </div>
                <div className="card p-2 flex flex-col justify-center gap-4 p-4 font-mono">
                  <div className="text-xs uppercase font-bold tracking-widest text-text-secondary border-b border-border/50 pb-2">
                    Duty Cycle & Power Budget Monitor
                  </div>
                  {(() => {
                    const dutyFrac = (params.tau * 1e-6) * params.prf;
                    const dutyPct = dutyFrac * 100;
                    const pAvgKw = params.pwr * dutyFrac;
                    const pulseEnergy = (params.pwr * 1e3) * (params.tau * 1e-6);
                    const isOverDuty = dutyPct > 10.0;
                    return (
                      <div className="grid grid-cols-2 gap-3 text-xs">
                        <div className="bg-surface-hover p-3 rounded-lg border border-border/50">
                          <span className="text-text-secondary block mb-1">Duty Cycle:</span>
                          <span className={`text-xl font-bold ${isOverDuty ? 'text-accent-red animate-pulse' : 'text-accent-cyan'}`}>
                            {dutyPct.toFixed(2)}%
                          </span>
                          <span className="text-[10px] text-gray-400 block mt-1">Safety Limit: ≤ 10.0%</span>
                        </div>
                        <div className="bg-surface-hover p-3 rounded-lg border border-border/50">
                          <span className="text-text-secondary block mb-1">Average Power:</span>
                          <span className="text-xl font-bold text-accent-green">
                            {(pAvgKw * 1000).toFixed(1)} W
                          </span>
                          <span className="text-[10px] text-gray-400 block mt-1">Peak: {params.pwr.toFixed(0)} kW</span>
                        </div>
                        <div className="bg-surface-hover p-3 rounded-lg border border-border/50 col-span-2">
                          <div className="flex justify-between items-center mb-1.5">
                            <span className="text-text-secondary">Pulse Energy:</span>
                            <span className="text-text-primary font-bold">{pulseEnergy.toFixed(2)} J</span>
                          </div>
                          <div className="w-full h-2.5 bg-black/50 rounded-full overflow-hidden border border-border/50">
                            <div 
                              className={`h-full transition-all duration-300 ${isOverDuty ? 'bg-accent-red' : 'bg-accent-cyan'}`} 
                              style={{ width: `${Math.min(100, (dutyPct / 10.0) * 100)}%` }}
                            />
                          </div>
                        </div>
                      </div>
                    );
                  })()}
                </div>
              </div>
            )}

        {activeTab === 'array' && (
          <div className="grid grid-cols-3 grid-rows-2 gap-4 flex-1 min-h-0 h-full">
            <div className="card p-2 col-span-1 row-span-2">
              <PlotCard 
                title="3D Spatial Pattern (Wideband)" 
                data={data?.spatial_pattern ? [{ z: data.spatial_pattern, type: 'surface', colorscale: 'Turbo' }] : []}
                layout={{ scene: { xaxis: {title: 'Azimuth (Bins)', gridcolor: '#2c2c2e'}, yaxis: {title: 'Elevation (Bins)', gridcolor: '#2c2c2e'}, zaxis: {title: 'Gain (dB)', gridcolor: '#2c2c2e'} } }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="Azimuth Pattern Cut (dB)" 
                data={[
                  ...(data?.azimuth_cut ? [{
                    y: data.azimuth_cut,
                    mode: 'lines', name: 'Az Cut', line: { color: '#0A84FF', width: 2 }, fill: 'tozeroy', fillcolor: 'rgba(10, 132, 255, 0.15)'
                  }] : []),
                  { x: [0, 90], y: [-3, -3], mode: 'lines', name: '-3 dB Beamwidth', line: { color: '#FF453A', dash: 'dash', width: 1.5 } }
                ]}
                layout={{ xaxis: { title: 'Azimuth (Bins)', gridcolor: '#2c2c2e' }, yaxis: { title: 'Gain (dB)', gridcolor: '#2c2c2e', range: [-45, 5] } }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="Physical Array Geometry (Tx/Rx)" 
                data={(() => {
                  const c = 299792458;
                  const lambda = c / (params.freq * 1e9);
                  const dy = lambda / 2;
                  const dz = lambda / 2;
                  const txX: number[] = [], txY: number[] = [], txZ: number[] = [];
                  for (let col = 0; col < params.ny; col++) {
                    for (let row = 0; row < params.nz; row++) {
                      txX.push((col - (params.ny - 1) / 2) * dy);
                      txY.push((row - (params.nz - 1) / 2) * dz);
                      txZ.push(0);
                    }
                  }
                  const rxX: number[] = [], rxY: number[] = [], rxZ: number[] = [];
                  for (let col = 0; col < params.ny; col++) {
                    for (let row = 0; row < params.nz; row++) {
                      rxX.push((col - (params.ny - 1) / 2) * dy);
                      rxY.push((row - (params.nz - 1) / 2) * dz);
                      rxZ.push(0.04);
                    }
                  }
                  return [
                    { x: txX, y: txY, z: txZ, mode: 'markers', type: 'scatter3d', name: 'Tx Elements', marker: { size: 3.5, color: '#0A84FF' } },
                    { x: rxX, y: rxY, z: rxZ, mode: 'markers', type: 'scatter3d', name: 'Rx Elements', marker: { size: 2.5, color: '#FF453A' } }
                  ];
                })()}
                layout={{ scene: { xaxis: { title: 'Y (m)' }, yaxis: { title: 'Z (m)' }, zaxis: { title: 'X (m)', range: [-0.1, 0.2] } } }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="Elevation Pattern Cut (dB)" 
                data={[
                  ...(data?.elevation_cut ? [{
                    y: data.elevation_cut,
                    mode: 'lines', name: 'El Cut', line: { color: '#FF453A', width: 2 }, fill: 'tozeroy', fillcolor: 'rgba(255, 69, 58, 0.15)'
                  }] : []),
                  { x: [0, 90], y: [-3, -3], mode: 'lines', name: '-3 dB Beamwidth', line: { color: '#0A84FF', dash: 'dash', width: 1.5 } }
                ]}
                layout={{ xaxis: { title: 'Elevation (Bins)', gridcolor: '#2c2c2e' }, yaxis: { title: 'Gain (dB)', gridcolor: '#2c2c2e', range: [-45, 5] } }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="Global Earth Projection Map" 
                data={(() => {
                  const lat0 = 39.0 * (Math.PI / 180);
                  const lon0 = -98.0 * (Math.PI / 180);
                  const Re = 6371.0;
                  const xRadar = Re * Math.cos(lat0) * Math.cos(lon0);
                  const yRadar = Re * Math.cos(lat0) * Math.sin(lon0);
                  const zRadar = Re * Math.sin(lat0);

                  const azRad = params.az * (Math.PI / 180);
                  const elRad = params.el * (Math.PI / 180);
                  const e_enu = Math.cos(elRad) * Math.sin(azRad);
                  const n_enu = Math.cos(elRad) * Math.cos(azRad);
                  const u_enu = Math.sin(elRad);
                  const beamLen = 1200;
                  const xBeam = xRadar + beamLen * (-Math.sin(lon0)*e_enu - Math.sin(lat0)*Math.cos(lon0)*n_enu + Math.cos(lat0)*Math.cos(lon0)*u_enu);
                  const yBeam = yRadar + beamLen * (Math.cos(lon0)*e_enu - Math.sin(lat0)*Math.sin(lon0)*n_enu + Math.cos(lat0)*Math.sin(lon0)*u_enu);
                  const zBeam = zRadar + beamLen * (Math.cos(lat0)*n_enu + Math.sin(lat0)*u_enu);

                  const uT = Array.from({length: 16}, (_, i) => (i * 2 * Math.PI) / 15);
                  const vP = Array.from({length: 9}, (_, i) => (i * Math.PI) / 8);
                  const xG: number[] = [], yG: number[] = [], zG: number[] = [];
                  for (const phi of vP) {
                    for (const theta of uT) {
                      xG.push(Re * Math.sin(phi) * Math.cos(theta));
                      yG.push(Re * Math.sin(phi) * Math.sin(theta));
                      zG.push(Re * Math.cos(phi));
                    }
                  }
                  return [
                    { x: xG, y: yG, z: zG, mode: 'markers', type: 'scatter3d', name: 'Earth Surface', marker: { size: 1.5, color: '#1a365d', opacity: 0.6 } },
                    { x: [xRadar], y: [yRadar], z: [zRadar], mode: 'markers+text', type: 'scatter3d', name: 'Radar Site (Kansas)', text: ['Radar Origin'], marker: { size: 6, color: '#FF3B30' } },
                    { x: [xRadar, xBeam], y: [yRadar, yBeam], z: [zRadar, zBeam], mode: 'lines', type: 'scatter3d', name: 'Steered Beam Mainlobe', line: { color: '#30D158', width: 4 } }
                  ];
                })()}
                layout={{ scene: { xaxis: { title: 'ECEF X (km)' }, yaxis: { title: 'ECEF Y (km)' }, zaxis: { title: 'ECEF Z (km)' } } }}
              />
            </div>
          </div>
        )}

        {activeTab === 'ew' && (
          <div className="grid grid-cols-2 grid-rows-2 gap-4 flex-1 min-h-0 h-full">
            <div className="card p-2">
              <PlotCard 
                title="IMM Tracker Airspace (3D Trace)" 
                data={trackHistory.x.length > 0 ? [
                  {
                    x: trackHistory.x, y: trackHistory.y, z: trackHistory.z,
                    mode: 'lines+markers', type: 'scatter3d', name: 'Target Trajectory',
                    line: { color: '#FF453A', width: 3 }, marker: { size: 4, color: '#FFD60A' }
                  },
                  ...(data?.ellipsoid_x ? [{
                    x: data?.ellipsoid_x, y: data?.ellipsoid_y, z: data?.ellipsoid_z,
                    type: 'mesh3d', alphahull: 0, opacity: 0.15, color: '#0A84FF', name: 'Covariance'
                  }] : [])
                ] : []}
                layout={{
                  scene: {
                    xaxis: { title: 'X (m)', range: [-10000, 10000], gridcolor: '#2c2c2e' },
                    yaxis: { title: 'Y (m)', range: [-10000, 10000], gridcolor: '#2c2c2e' },
                    zaxis: { title: 'Altitude (m)', range: [0, 10000], gridcolor: '#2c2c2e' },
                    camera: { eye: { x: 1.5, y: -1.5, z: 1.2 } }
                  }
                }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="PPI (Plan Position Indicator) Scope" 
                data={(() => {
                  const ringAngles = Array.from({length: 73}, (_, i) => i * 5);
                  const ringTraces = [5000, 10000, 15000, 20000].map(r => ({
                    r: ringAngles.map(() => r),
                    theta: ringAngles,
                    mode: 'lines',
                    line: { color: 'rgba(50, 215, 75, 0.25)', width: 1, dash: 'dot' },
                    showlegend: false,
                    hoverinfo: 'none'
                  }));
                  const sweepTrace = {
                    r: [0, 20000],
                    theta: [params.az, params.az],
                    mode: 'lines',
                    line: { color: '#32D74B', width: 2.5 },
                    name: 'Antenna Boresight Sweep'
                  };
                  const targetBlipTraces = data?.ppi_blips?.map(b => ({
                    r: [b.r],
                    theta: [b.theta],
                    mode: 'markers+text',
                    name: `Trk-${b.id}`,
                    text: [`Trk-${b.id} (${(b.r/1000).toFixed(1)} km)`],
                    textposition: 'top right',
                    textfont: { family: 'monospace', color: '#FFD60A', size: 10 },
                    marker: { size: 10, color: '#FF453A', symbol: 'diamond' }
                  })) || [];
                  return [...ringTraces, sweepTrace, ...targetBlipTraces];
                })()}
                layout={{ 
                  polar: { 
                    bgcolor: '#001500',
                    angularaxis: { rotation: 90, direction: 'clockwise', gridcolor: '#004400', linecolor: '#004400' }, 
                    radialaxis: { gridcolor: '#004400', linecolor: '#00ff00', range: [0, 20000] } 
                  } 
                }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="EW Jammer Spectral Environment" 
                data={data?.ew_freq_mhz ? [{
                  x: data.ew_freq_mhz,
                  y: data.ew_power_dbm,
                  mode: 'lines',
                  name: 'Rx Spectral Power',
                  line: { color: '#FF453A', width: 2 },
                  fill: 'tozeroy',
                  fillcolor: 'rgba(255, 69, 58, 0.2)'
                }] : []}
                layout={{
                  xaxis: { title: 'Baseband Frequency (MHz)', gridcolor: '#2c2c2e' },
                  yaxis: { title: 'Rx Power (dBm)', gridcolor: '#2c2c2e', range: [-120, -20] }
                }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="3D Range-Doppler Trace History" 
                data={rd3dHistory.r.length > 0 ? [{
                  x: rd3dHistory.r,
                  y: rd3dHistory.v,
                  z: rd3dHistory.t,
                  mode: 'lines+markers',
                  type: 'scatter3d',
                  name: 'Target RD Path',
                  line: { color: '#0A84FF', width: 3 },
                  marker: { size: 4, color: '#32D74B' }
                }] : []}
                layout={{
                  scene: {
                    xaxis: { title: 'Range (km)', gridcolor: '#2c2c2e' },
                    yaxis: { title: 'Velocity (m/s)', range: [-300, 300], gridcolor: '#2c2c2e' },
                    zaxis: { title: 'Time (s)', gridcolor: '#2c2c2e' }
                  }
                }}
              />
            </div>
          </div>
        )}

        {activeTab === 'signal' && (
          <div className="grid grid-cols-3 grid-rows-2 gap-4 flex-1 min-h-0 h-full">
            <div className="card p-2">
              <PlotCard 
                title="Raw Receiver Baseband (Fast Time)" 
                data={data?.fast_time_rx ? [{
                  x: data.fast_time_t_us,
                  y: data.fast_time_rx,
                  mode: 'lines',
                  name: 'ADC In-Phase I(t)',
                  line: { color: '#30D158', width: 1.5 }
                }] : []}
                layout={{ xaxis: { title: 'Fast Time (μs)', gridcolor: '#2c2c2e' }, yaxis: { title: 'Raw ADC Voltage (V)', gridcolor: '#2c2c2e' } }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="Matched Filter Range Profile" 
                data={data?.mf_profile ? [{
                  y: data.mf_profile,
                  mode: 'lines', line: { color: '#0A84FF', width: 2 }, fill: 'tozeroy', fillcolor: 'rgba(10, 132, 255, 0.2)'
                }] : []}
                layout={{ xaxis: { title: 'Range Bin', gridcolor: '#2c2c2e' }, yaxis: { title: 'Power (dB)', gridcolor: '#2c2c2e' } }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="STAP Covariance Angle-Doppler Ridge" 
                data={data?.stap_covariance ? [{ z: data.stap_covariance, type: 'heatmap', colorscale: 'Plasma' }] : []}
                layout={{ xaxis: { title: 'Spatial DOF', gridcolor: '#2c2c2e' }, yaxis: { title: 'Temporal DOF', gridcolor: '#2c2c2e' } }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="CPI FFT Range-Doppler Matrix" 
                data={data?.rd_matrix ? [{ z: data.rd_matrix, type: 'heatmap', colorscale: 'Jet' }] : []} 
              />
            </div>
            <div className="card p-2 col-span-2">
              <PlotCard 
                title="1D CA-CFAR Detections" 
                data={data?.cfar_detections ? [{ z: data.cfar_detections, type: 'heatmap', colorscale: 'Greys', showscale: false }] : []}
                layout={{ xaxis: { title: 'Range Bin', gridcolor: '#2c2c2e' }, yaxis: { title: 'Doppler Bin', gridcolor: '#2c2c2e' } }}
              />
            </div>
          </div>
        )}

        {activeTab === 'c2' && (
          <div className="flex-1 flex flex-col gap-4 min-h-0 overflow-y-auto">
            {/* Operator Table */}
            <div className="card p-4 shrink-0">
              <div className="panel-title mb-4">C2 Operator Tracks (IMM-EKF 7-State)</div>
              <div className="overflow-x-auto">
                <table className="w-full text-left text-sm">
                  <thead className="text-text-secondary border-b border-border/50 text-xs">
                    <tr>
                      <th className="pb-2 font-semibold">Track ID</th>
                      <th className="pb-2 font-semibold">Range (km)</th>
                      <th className="pb-2 font-semibold">Alt (m)</th>
                      <th className="pb-2 font-semibold">Speed (m/s)</th>
                      <th className="pb-2 font-semibold">SNR (dB)</th>
                      <th className="pb-2 font-semibold">Aspect Shape</th>
                      <th className="pb-2 font-semibold">Classification</th>
                      <th className="pb-2 font-semibold">HH (m²)</th>
                      <th className="pb-2 font-semibold">VV (m²)</th>
                    </tr>
                  </thead>
                  <tbody className="divide-y divide-border/30 font-mono text-xs">
                    {data?.c2_measurements?.map((m: {id: number, snr_db: number, pol: number[]}) => {
                      const r = m.pol[0] / Math.max(1e-6, m.pol[1]);
                      let shape = 'Symmetric';
                      if (r > 2.0) shape = 'Horizontal';
                      else if (r < 0.5) shape = 'Vertical';
                      
                      let cls = 'Missile';
                      if (r > 2.0) cls = 'Airliner';
                      else if (r > 0.5 && r < 2.0) cls = 'Stealth/UAV';

                      const estX = data?.track_history_x?.[0] || 1000;
                      const estY = data?.track_history_y?.[0] || 500;
                      const estZ = data?.track_history_z?.[0] || 5000;
                      const rngKm = Math.sqrt(estX*estX + estY*estY + estZ*estZ) / 1000;

                      return (
                        <tr key={m.id} className="hover:bg-white/5 transition-colors">
                          <td className="py-2 text-accent-cyan font-bold">Trk-{m.id}</td>
                          <td className="py-2 text-text-primary">{rngKm.toFixed(2)}</td>
                          <td className="py-2 text-text-primary">{estZ.toFixed(0)}</td>
                          <td className="py-2 text-accent-green">250.0</td>
                          <td className="py-2">{m.snr_db.toFixed(1)}</td>
                          <td className="py-2">{shape}</td>
                          <td className="py-2 font-bold text-accent-cyan">{cls}</td>
                          <td className="py-2 text-blue-400">{m.pol[0].toFixed(2)}</td>
                          <td className="py-2 text-green-400">{m.pol[1].toFixed(2)}</td>
                        </tr>
                      );
                    }) || (
                      <tr>
                        <td colSpan={9} className="py-4 text-center text-text-secondary">No tracks detected or simulating...</td>
                      </tr>
                    )}
                  </tbody>
                </table>
              </div>
            </div>

            {/* Graphs: 4 plots in a 2x2 grid */}
            <div className="grid grid-cols-2 grid-rows-2 gap-4 flex-1 min-h-[480px]">
              <div className="card p-2 flex flex-col">
                <PlotCard 
                  title="Dual-Pol Signature Classification" 
                  data={data?.c2_measurements?.length ? [{
                    x: ['HH', 'VV', 'HV'], 
                    y: [data.c2_measurements[0].pol[0], data.c2_measurements[0].pol[1], data.c2_measurements[0].pol[2]], 
                    type: 'bar', 
                    marker: { color: ['#0A84FF', '#32D74B', '#FF453A'] }
                  }] : []}
                  layout={{ xaxis: { gridcolor: '#2c2c2e' }, yaxis: { gridcolor: '#2c2c2e', title: 'RCS (m²)' } }}
                />
              </div>

              <div className="card p-2 flex flex-col">
                <PlotCard 
                  title="RCS Silhouette Aspect Estimate" 
                  data={(() => {
                    const pol = data?.c2_measurements?.[0]?.pol || [4.0, 2.0, 0.5];
                    const a = Math.max(0.5, Math.sqrt(pol[0]) * 2.5);
                    const b = Math.max(0.5, Math.sqrt(pol[1]) * 2.5);
                    const thetaVec = Array.from({length: 60}, (_, i) => (i * 2 * Math.PI) / 59);
                    return [{
                      x: thetaVec.map(th => a * Math.cos(th)),
                      y: thetaVec.map(th => b * Math.sin(th)),
                      mode: 'lines',
                      line: { color: '#FFD60A', width: 2.5 },
                      fill: 'toself',
                      fillcolor: 'rgba(255, 214, 10, 0.2)',
                      name: 'Aspect Silhouette'
                    }];
                  })()}
                  layout={{
                    xaxis: { title: 'Cross Dimension (m)', range: [-10, 10], gridcolor: '#2c2c2e' },
                    yaxis: { title: 'Vertical Dimension (m)', range: [-10, 10], gridcolor: '#2c2c2e', scaleanchor: 'x' }
                  }}
                />
              </div>

              <div className="card p-2 flex flex-col">
                <PlotCard 
                  title="Filter Range Error (m)" 
                  data={rangeErrors.length > 0 ? [{
                    y: rangeErrors,
                    mode: 'lines+markers',
                    name: 'ΔR (Est - True)',
                    line: { color: '#FFD60A', width: 2 },
                    marker: { size: 3, color: '#FFD60A' }
                  }] : []}
                  layout={{
                    xaxis: { title: 'Telemetry Update Step', gridcolor: '#2c2c2e' },
                    yaxis: { title: 'Range Error (m)', gridcolor: '#2c2c2e' }
                  }}
                />
              </div>

              <div className="card p-2 flex flex-col">
                <PlotCard 
                  title="Filter Velocity Error (m/s)" 
                  data={velErrors.length > 0 ? [{
                    y: velErrors,
                    mode: 'lines+markers',
                    name: 'ΔV (Est - True)',
                    line: { color: '#30D158', width: 2 },
                    marker: { size: 3, color: '#30D158' }
                  }] : []}
                  layout={{
                    xaxis: { title: 'Telemetry Update Step', gridcolor: '#2c2c2e' },
                    yaxis: { title: 'Velocity Error (m/s)', gridcolor: '#2c2c2e' }
                  }}
                />
              </div>
            </div>
          </div>
        )}

        {activeTab === 'pointer' && (
          <div className="grid grid-cols-2 gap-6 flex-1 min-h-0">
            <div className="card p-2 relative flex flex-col">
              <PlotCard 
                title="Hexapod Dynamics (Newton-Euler)" 
                data={data?.hex_tp ? [
                  {
                    x: data?.hex_base?.[0] as number[], y: data?.hex_base?.[1] as number[], z: [0,0,0,0,0,0],
                    mode: 'markers', type: 'scatter3d', name: 'Base', marker: { size: 6, color: '#32D74B' }
                  },
                  {
                    x: data.hex_tp[0] as number[], y: data.hex_tp[1] as number[], z: data.hex_tp[2] as number[],
                    mode: 'lines+markers', type: 'scatter3d', name: 'Top Plate', marker: { size: 6, color: '#0A84FF' }, line: {color: '#0A84FF', width: 4}
                  },
                  ...(data.hex_laser ? [{
                    x: [0, data.hex_laser[0]], y: [0, data.hex_laser[1]], z: [0.6, data.hex_laser[2]],
                    mode: 'lines', type: 'scatter3d', name: 'Boresight', line: { color: '#FF453A', width: 3, dash: 'dash' }
                  }] : [])
                ] : []}
                layout={{
                  scene: {
                    xaxis: { title: 'X (m)', range: [-1, 1], gridcolor: '#2c2c2e' },
                    yaxis: { title: 'Y (m)', range: [-1, 1], gridcolor: '#2c2c2e' },
                    zaxis: { title: 'Z (m)', range: [0, 1.5], gridcolor: '#2c2c2e' },
                  }
                }}
              />
              {data?.hex_warn && (
                <div className={`absolute bottom-4 left-4 right-4 p-3 rounded-lg text-sm font-bold shadow-lg ${data.hex_warn === 'NOMINAL' ? 'bg-green-500/20 text-green-400 border border-green-500/50' : 'bg-red-500/20 text-red-400 border border-red-500/50 animate-pulse'}`}>
                  HEXAPOD STATUS: {data.hex_warn} (Cond: {data.hex_cond?.toFixed(2)})
                </div>
              )}
            </div>
            
            <div className="card p-2 relative flex flex-col">
              <div className="absolute inset-0 p-6 flex flex-col">
                <div className="panel-title mb-4">Az/El Gimbal Dynamics (Euler-Lagrange)</div>
                <div className="flex-1 flex flex-col justify-center gap-6">
                  {data?.azel_q ? (
                    <>
                      <div className="grid grid-cols-2 gap-4">
                        <div className="bg-surface-hover p-4 rounded-xl border border-border/50">
                          <div className="text-text-secondary text-xs font-semibold mb-1">Azimuth Angle</div>
                          <div className="text-2xl font-mono text-accent-cyan">{(data.azel_q[0] * 180 / Math.PI).toFixed(1)}°</div>
                        </div>
                        <div className="bg-surface-hover p-4 rounded-xl border border-border/50">
                          <div className="text-text-secondary text-xs font-semibold mb-1">Elevation Angle</div>
                          <div className="text-2xl font-mono text-accent-cyan">{(data.azel_q[1] * 180 / Math.PI).toFixed(1)}°</div>
                        </div>
                        <div className="bg-surface-hover p-4 rounded-xl border border-border/50">
                          <div className="text-text-secondary text-xs font-semibold mb-1">Azimuth Torque</div>
                          <div className="text-xl font-mono text-text-primary">{data.azel_tau_a?.toFixed(1)} Nm</div>
                        </div>
                        <div className="bg-surface-hover p-4 rounded-xl border border-border/50">
                          <div className="text-text-secondary text-xs font-semibold mb-1">Elevation Torque</div>
                          <div className="text-xl font-mono text-text-primary">{data.azel_tau_e?.toFixed(1)} Nm</div>
                        </div>
                      </div>
                      
                      {data.azel_warn && (
                        <div className={`p-4 rounded-lg text-sm font-bold text-center border ${data.azel_warn === 'NOMINAL' ? 'bg-green-500/10 text-green-400 border-green-500/30' : 'bg-red-500/10 text-red-400 border-red-500/30 animate-pulse'}`}>
                          GIMBAL STATUS: {data.azel_warn}
                        </div>
                      )}
                    </>
                  ) : (
                    <div className="text-center text-text-secondary font-mono">Waiting for dynamics telemetry...</div>
                  )}
                </div>
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
            <span className="text-accent-cyan font-bold flex items-center gap-2">
              <span className="inline-block w-2 h-2 rounded-full bg-accent-green animate-pulse"></span>
              RADAR TELEMETRY & DSP LOGS
            </span>
            <span className="text-gray-600">|</span>
            <span className="truncate text-gray-400 font-mono text-[11px]">
              {logs.length > 0 ? logs[logs.length - 1].message : "Awaiting radar server telemetry..."}
            </span>
          </div>
          <div className="shrink-0 flex items-center gap-3 text-gray-400 text-xs">
            <span className="text-[11px] bg-white/5 px-2 py-0.5 rounded border border-white/10 font-mono">{logs.length} logs</span>
            <span className="text-accent-cyan font-semibold hover:text-white transition-colors">{terminalExpanded ? '▼ Collapse Terminal' : '▲ Expand Terminal'}</span>
          </div>
        </div>
        
        {terminalExpanded && (
          <div className="h-56 overflow-y-auto px-6 py-3 font-mono text-[11px] border-t border-[#30363d] flex flex-col gap-1.5 bg-[#0a0c10]">
            {logs.map((log, i) => (
              <div key={i} className="flex gap-4 hover:bg-white/5 p-1 rounded leading-relaxed">
                <span className="text-gray-500 shrink-0">[{log.timestamp}]</span>
                <span className={
                  log.message.includes('[SYSTEM]') ? 'text-accent-cyan font-medium' : 
                  log.message.includes('[RADAR_DSP]') ? 'text-accent-green font-medium' : 
                  log.message.includes('[RADAR_TRACK]') ? 'text-[#58a6ff] font-medium' : 
                  log.message.includes('[RADAR_PARAMS]') ? 'text-[#e3b341] font-medium' : 
                  log.message.includes('[RADAR_GIMBAL]') ? 'text-[#d2a8ff] font-medium' : 
                  log.message.includes('[CLIENT_TX]') ? 'text-[#a371f7] font-medium' : 
                  log.message.includes('[CLIENT_RX]') ? 'text-[#3fb950] font-medium' : 
                  log.message.includes('Error') || log.message.includes('warn') ? 'text-accent-red font-bold' : 
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
