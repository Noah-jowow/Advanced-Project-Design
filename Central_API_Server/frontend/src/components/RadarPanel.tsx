import React, { useState } from 'react';
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

  const [activeTab, setActiveTab] = useState('c2');

  const handleRun = () => {
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
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setParams(prev => ({ ...prev, [name]: parseFloat(value) }));
  };

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
                  id: Date.now(), 
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
          <button onClick={handleRun} disabled={status !== 'CONNECTED'} className="btn-primary">
            EXECUTE SCAN
          </button>
        </div>
      </aside>

      {/* Main Viewport */}
      <div className="flex-1 flex flex-col gap-6 overflow-hidden">
        
        {/* Tab Navigation (Glass) */}
        <div className="flex gap-4">
          <button 
            className={`px-6 py-2.5 rounded-xl text-sm font-semibold transition-all duration-300 border ${activeTab === 'c2' ? 'bg-surface-hover border-border-strong text-text-primary shadow-lg' : 'bg-surface border-border/50 text-text-secondary hover:text-text-primary hover:bg-white/[0.05]'}`}
            onClick={() => setActiveTab('c2')}
          >
            C2 & Sensor Analysis
          </button>
          <button 
            className={`px-6 py-2.5 rounded-xl text-sm font-semibold transition-all duration-300 border ${activeTab === 'setup' ? 'bg-surface-hover border-border-strong text-text-primary shadow-lg' : 'bg-surface border-border/50 text-text-secondary hover:text-text-primary hover:bg-white/[0.05]'}`}
            onClick={() => setActiveTab('setup')}
          >
            System Diagnostics
          </button>
        </div>
        
        {activeTab === 'c2' && (
          <div className="grid grid-cols-2 grid-rows-2 gap-6 flex-1 min-h-0">
            <div className="card p-2">
              <PlotCard 
                title="Range-Doppler Matrix" 
                data={data?.rd_matrix ? [{ z: data.rd_matrix, type: 'heatmap', colorscale: 'Jet' }] : []} 
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="IMM Tracker Airspace" 
                data={data?.track_history_x ? [
                  {
                    x: data.track_history_x as number[], y: data.track_history_y as number[], z: data.track_history_z as number[],
                    mode: 'lines+markers', type: 'scatter3d', name: 'Target Trajectory',
                    line: { color: '#FF453A', width: 4 }, marker: { size: 4, color: '#FFD60A' }
                  },
                  ...(data.ellipsoid_x ? [{
                    x: data.ellipsoid_x, y: data.ellipsoid_y, z: data.ellipsoid_z,
                    type: 'mesh3d', alphahull: 0, opacity: 0.15, color: '#0A84FF', name: 'Covariance'
                  }] : [])
                ] : []}
                layout={{
                  scene: {
                    xaxis: { title: 'X (m)', gridcolor: '#2c2c2e' },
                    yaxis: { title: 'Y (m)', gridcolor: '#2c2c2e' },
                    zaxis: { title: 'Altitude (m)', gridcolor: '#2c2c2e' },
                    camera: { eye: { x: 1.5, y: -1.5, z: 1.2 } }
                  }
                }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="3D Spatial Pattern" 
                data={data?.rd_matrix ? [{ z: data.rd_matrix, type: 'surface', colorscale: 'Turbo', showscale: false }] : []}
                layout={{ scene: { xaxis: {visible: false}, yaxis: {visible: false}, zaxis: {visible: false} } }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="Azimuth Pattern Cut (dB)" 
                data={data?.rd_matrix ? [{
                  y: data.rd_matrix[Math.floor(data.rd_matrix.length/2)],
                  mode: 'lines', line: { color: '#0A84FF', width: 2 }, fill: 'tozeroy', fillcolor: 'rgba(10, 132, 255, 0.2)'
                }] : []}
                layout={{
                  xaxis: { gridcolor: '#2c2c2e' }, yaxis: { gridcolor: '#2c2c2e' }
                }}
              />
            </div>
          </div>
        )}

        {activeTab === 'setup' && (
          <div className="flex-1 card p-8 min-h-0 flex items-center justify-center">
            <div className="font-mono text-sm text-accent-cyan flex flex-col gap-3">
              <div>{'>'} [SYSTEM] Wideband STAP Module loaded.</div>
              {data?.stap_covariance_norm && <div>{'>'} STAP Covariance Norm: {data.stap_covariance_norm}</div>}
              {data?.stap_weights && <div>{'>'} Weights Computed: {data.stap_weights.length}</div>}
              <div className="animate-pulse">{'>'} Waiting for telemetry...</div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
