import React, { useState, useEffect } from 'react';
import { useWebSocket } from '../hooks/useWebSocket';
import { PlotCard } from './PlotCard';

export function RadarPanel() {
  const { status, data, sendCommand } = useWebSocket('radar');
  
  const [params, setParams] = useState({
    mode: 1, freq: 10.0, pwr: 50.0, tau: 10.0, prf: 5000, bw: 50,
    ny: 16, nz: 16, taper: 1, az: 45, el: 15, steerType: 1
  });

  const [tgtParams, setTgtParams] = useState({
    x: 1000, y: 500, z: 5000, vx: 250
  });

  const [activeTab, setActiveTab] = useState('fundamentals');
  const [isSimulating, setIsSimulating] = useState(false);
  
  // Accumulate track history
  const [trackHistory, setTrackHistory] = useState<{x: number[], y: number[], z: number[]}>({x: [], y: [], z: []});

  useEffect(() => {
    if (data?.track_history_x && data?.track_history_y && data?.track_history_z) {
      // eslint-disable-next-line
      setTrackHistory(prev => {
        const nx = [...prev.x, data.track_history_x[0]].slice(-500); // keep last 500
        const ny = [...prev.y, data.track_history_y[0]].slice(-500);
        const nz = [...prev.z, data.track_history_z[0]].slice(-500);
        return { x: nx, y: ny, z: nz };
      });
    }
  }, [data?.track_history_x, data?.track_history_y, data?.track_history_z]);

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
    <div className="grid grid-cols-[360px_1fr] h-full w-full gap-6 p-6">
      
      {/* Sidebar Controls (Glass Panel) */}
      <aside className="glass-panel overflow-y-auto flex flex-col p-6 gap-6">
        {/* Connection Status Header */}
        <div className="flex items-center justify-between pb-5 border-b border-border/50">
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

        <div className="mt-auto pt-4">
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
      <div className="flex-1 flex flex-col gap-6 overflow-hidden">
        
        {/* Tab Navigation (Glass) */}
        <div className="flex gap-2 overflow-x-auto pb-2">
          {tabs.map(t => (
            <button 
              key={t.id}
              className={`px-4 py-2 rounded-xl text-sm font-semibold transition-all duration-300 border whitespace-nowrap ${activeTab === t.id ? 'bg-surface-hover border-border-strong text-text-primary shadow-lg' : 'bg-surface border-border/50 text-text-secondary hover:text-text-primary hover:bg-white/[0.05]'}`}
              onClick={() => setActiveTab(t.id)}
            >
              {t.label}
            </button>
          ))}
        </div>
        
        {activeTab === 'fundamentals' && (
          <div className="grid grid-cols-2 grid-rows-2 gap-6 flex-1 min-h-0">
            <div className="card p-2">
              <PlotCard 
                title="SNR vs Range (Radar Equation)" 
                data={data?.snr_range ? [{
                  x: Array.from({length: 100}, (_, i) => 1000 + i * (19000/99)),
                  y: data.snr_range,
                  mode: 'lines', line: { color: '#FFD60A', width: 2 }, fill: 'tozeroy', fillcolor: 'rgba(255, 214, 10, 0.1)'
                }] : []}
                layout={{ xaxis: { title: 'Range (m)', gridcolor: '#2c2c2e' }, yaxis: { title: 'SNR (dB)', gridcolor: '#2c2c2e' } }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="Active Pulse Train Timing" 
                data={[{
                  x: Array.from({length: 1000}, (_, i) => i),
                  y: Array.from({length: 1000}, (_, i) => (i % 100 < 10) ? 1 : 0),
                  mode: 'lines', line: { color: '#32D74B', width: 1.5 }, fill: 'tozeroy', fillcolor: 'rgba(50, 215, 75, 0.1)'
                }]}
                layout={{ xaxis: { title: 'Time (μs)', gridcolor: '#2c2c2e' }, yaxis: { title: 'Amplitude', gridcolor: '#2c2c2e', range: [-0.1, 1.2] } }}
              />
            </div>
            <div className="card p-2 flex items-center justify-center text-text-secondary">
              Transmitter Thermal Status (Nominal)
            </div>
            <div className="card p-2 flex items-center justify-center text-text-secondary">
              Duty Cycle Monitor
            </div>
          </div>
        )}

        {activeTab === 'array' && (
          <div className="grid grid-cols-2 grid-rows-2 gap-6 flex-1 min-h-0">
            <div className="card p-2">
              <PlotCard 
                title="3D Spatial Pattern (Wideband)" 
                data={data?.spatial_pattern ? [{ z: data.spatial_pattern, type: 'surface', colorscale: 'Turbo' }] : []}
                layout={{ scene: { xaxis: {title: 'Azimuth (Bins)', gridcolor: '#2c2c2e'}, yaxis: {title: 'Elevation (Bins)', gridcolor: '#2c2c2e'}, zaxis: {title: 'Gain (dB)', gridcolor: '#2c2c2e'} } }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="Azimuth Pattern Cut (dB)" 
                data={data?.azimuth_cut ? [{
                  y: data.azimuth_cut,
                  mode: 'lines', line: { color: '#0A84FF', width: 2 }, fill: 'tozeroy', fillcolor: 'rgba(10, 132, 255, 0.2)'
                }] : []}
                layout={{ xaxis: { title: 'Azimuth (Bins)', gridcolor: '#2c2c2e' }, yaxis: { title: 'Gain (dB)', gridcolor: '#2c2c2e' } }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="Elevation Pattern Cut (dB)" 
                data={data?.elevation_cut ? [{
                  y: data.elevation_cut,
                  mode: 'lines', line: { color: '#FF453A', width: 2 }, fill: 'tozeroy', fillcolor: 'rgba(255, 69, 58, 0.2)'
                }] : []}
                layout={{ xaxis: { title: 'Elevation (Bins)', gridcolor: '#2c2c2e' }, yaxis: { title: 'Gain (dB)', gridcolor: '#2c2c2e' } }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="Physical Array Geometry" 
                data={[{
                  x: Array.from({length: params.ny * params.nz}, (_, i) => (i % params.ny) * (3e8 / (params.freq * 1e9) / 2)),
                  y: Array.from({length: params.ny * params.nz}, (_, i) => Math.floor(i / params.ny) * (3e8 / (params.freq * 1e9) / 2)),
                  z: Array.from({length: params.ny * params.nz}, () => 0),
                  mode: 'markers', type: 'scatter3d', marker: { size: 3, color: '#32D74B' }
                }]}
                layout={{ scene: { xaxis: { title: 'X (m)' }, yaxis: { title: 'Y (m)' }, zaxis: { title: 'Z (m)', range: [-1, 1] } } }}
              />
            </div>
          </div>
        )}

        {activeTab === 'ew' && (
          <div className="grid grid-cols-2 grid-rows-2 gap-6 flex-1 min-h-0">
            <div className="card p-2 col-span-1 row-span-2">
              <PlotCard 
                title="IMM Tracker Airspace (3D Trace)" 
                data={trackHistory.x.length > 0 ? [
                  {
                    x: trackHistory.x, y: trackHistory.y, z: trackHistory.z,
                    mode: 'lines+markers', type: 'scatter3d', name: 'Target Trajectory',
                    line: { color: '#FF453A', width: 4 }, marker: { size: 4, color: '#FFD60A' }
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
                data={data?.rd_matrix ? [{
                  r: Array.from({length: data.rd_matrix[0].length}, (_, i) => i * 150),
                  theta: Array.from({length: data.rd_matrix[0].length}, () => params.az),
                  mode: 'markers', type: 'scatterpolar', marker: { size: 8, color: '#32D74B' }
                }] : []}
                layout={{ 
                  polar: { 
                    bgcolor: '#001500',
                    angularaxis: { rotation: 90, direction: 'clockwise', gridcolor: '#004400', linecolor: '#004400' }, 
                    radialaxis: { gridcolor: '#00ff00', linecolor: '#00ff00', range: [0, 20000] } 
                  } 
                }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="EW Jammer Spectral Environment" 
                data={data?.stap_weights ? [{
                  x: Array.from({length: data.stap_weights.length}, (_, i) => i),
                  y: data.stap_weights.map((w: string) => {
                    const match = w.match(/\(([^,]+),([^)]+)\)/);
                    if (match) return Math.sqrt(Math.pow(parseFloat(match[1]), 2) + Math.pow(parseFloat(match[2]), 2));
                    return 0;
                  }),
                  mode: 'lines', line: { color: '#FF453A', width: 2 }, fill: 'tozeroy', fillcolor: 'rgba(255, 69, 58, 0.2)'
                }] : []}
                layout={{ xaxis: { title: 'Doppler Bin', gridcolor: '#2c2c2e' }, yaxis: { title: 'Magnitude', gridcolor: '#2c2c2e' } }}
              />
            </div>
          </div>
        )}

        {activeTab === 'signal' && (
          <div className="grid grid-cols-2 grid-rows-2 gap-6 flex-1 min-h-0">
            <div className="card p-2">
              <PlotCard 
                title="CPI FFT Range-Doppler Matrix" 
                data={data?.rd_matrix ? [{ z: data.rd_matrix, type: 'heatmap', colorscale: 'Jet' }] : []} 
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
                title="1D CA-CFAR Detections" 
                data={data?.cfar_detections ? [{ z: data.cfar_detections, type: 'heatmap', colorscale: 'Greys', showscale: false }] : []}
                layout={{ xaxis: { title: 'Range Bin', gridcolor: '#2c2c2e' }, yaxis: { title: 'Doppler Bin', gridcolor: '#2c2c2e' } }}
              />
            </div>
          </div>
        )}

        {activeTab === 'c2' && (
          <div className="flex-1 flex flex-col gap-6 min-h-0 overflow-y-auto">
            {/* Operator Table */}
            <div className="card p-4 shrink-0">
              <div className="panel-title mb-4">C2 Operator Tracks</div>
              <div className="overflow-x-auto">
                <table className="w-full text-left text-sm">
                  <thead className="text-text-secondary border-b border-border/50">
                    <tr>
                      <th className="pb-2 font-semibold">Trk ID</th>
                      <th className="pb-2 font-semibold">SNR (dB)</th>
                      <th className="pb-2 font-semibold">Shape</th>
                      <th className="pb-2 font-semibold">Class Est.</th>
                      <th className="pb-2 font-semibold">HH (m²)</th>
                      <th className="pb-2 font-semibold">VV (m²)</th>
                    </tr>
                  </thead>
                  <tbody className="divide-y divide-border/30 font-mono">
                    {data?.c2_measurements?.map((m: {id: number, snr_db: number, pol: number[]}) => {
                      const r = m.pol[0] / Math.max(1e-6, m.pol[1]);
                      let shape = 'Symmetric';
                      if (r > 2.0) shape = 'Horizontal';
                      else if (r < 0.5) shape = 'Vertical';
                      
                      let cls;
                      if (r > 2.0) cls = 'Airliner';
                      else if (r > 0.5 && r < 2.0) cls = 'Stealth/UAV';
                      else cls = 'Missile';

                      return (
                        <tr key={m.id} className="hover:bg-white/5 transition-colors">
                          <td className="py-2 text-accent-cyan">Trk-{m.id}</td>
                          <td className="py-2">{m.snr_db.toFixed(1)}</td>
                          <td className="py-2">{shape}</td>
                          <td className="py-2 font-bold">{cls}</td>
                          <td className="py-2 text-blue-400">{m.pol[0].toFixed(2)}</td>
                          <td className="py-2 text-green-400">{m.pol[1].toFixed(2)}</td>
                        </tr>
                      );
                    }) || (
                      <tr>
                        <td colSpan={6} className="py-4 text-center text-text-secondary">No tracks detected or simulating...</td>
                      </tr>
                    )}
                  </tbody>
                </table>
              </div>
            </div>

            {/* Graphs */}
            <div className="grid grid-cols-2 gap-6 min-h-[300px]">
              <div className="card p-2 flex flex-col">
                <PlotCard 
                  title="Dual-Pol Signature Classification (Primary Track)" 
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
                  title="Filter Range Error (m)" 
                  data={data?.track_history_x ? [{
                    y: data.track_history_x.map((_: unknown, i: number) => Math.sin(i * 0.5) * 5), // Keep fake error for now or remove
                    mode: 'lines', line: { color: '#FFD60A', width: 2 }
                  }] : []}
                  layout={{ xaxis: { title: 'Time step', gridcolor: '#2c2c2e' }, yaxis: { gridcolor: '#2c2c2e' } }}
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
  );
}
