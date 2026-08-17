import React, { useState } from 'react';
import { useWebSocket } from '../hooks/useWebSocket';
import { PlotCard } from './PlotCard';

export function RadarPanel() {
  const { status, data, sendCommand } = useWebSocket('radar');
  
  // State for inputs
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
    <div className="flex h-full w-full">
      {/* Sidebar */}
      <aside className="w-80 bg-sidebar border-r border-border p-4 overflow-y-auto flex flex-col gap-6">
        <div>
          <div className="panel-title">📡 Radar Parameters</div>
          <div className="control-group">
            <div className="grid grid-cols-2 gap-2">
              <div className="control-item mb-0">
                <label>Radar Mode</label>
                <select name="mode" value={params.mode} onChange={handleChange}>
                  <option value={1}>Search</option>
                  <option value={2}>Track</option>
                  <option value={3}>SAR</option>
                </select>
              </div>
              <div className="control-item mb-0">
                <label>Freq (GHz)</label>
                <input type="number" name="freq" value={params.freq} onChange={handleChange} step="0.1" />
              </div>
              <div className="control-item mb-0">
                <label>Pwr (kW)</label>
                <input type="number" name="pwr" value={params.pwr} onChange={handleChange} />
              </div>
              <div className="control-item mb-0">
                <label>Tau (μs)</label>
                <input type="number" name="tau" value={params.tau} onChange={handleChange} />
              </div>
              <div className="control-item mb-0">
                <label>PRF (Hz)</label>
                <input type="number" name="prf" value={params.prf} onChange={handleChange} />
              </div>
              <div className="control-item mb-0">
                <label>BW (MHz)</label>
                <input type="number" name="bw" value={params.bw} onChange={handleChange} />
              </div>
            </div>
          </div>
        </div>

        <div>
          <div className="panel-title">📐 Array Synthesis</div>
          <div className="control-group">
            <div className="grid grid-cols-2 gap-2">
              <div className="control-item mb-0">
                <label>Ny (Cols)</label>
                <input type="number" name="ny" value={params.ny} onChange={handleChange} />
              </div>
              <div className="control-item mb-0">
                <label>Nz (Rows)</label>
                <input type="number" name="nz" value={params.nz} onChange={handleChange} />
              </div>
              <div className="control-item mb-0 col-span-2">
                <label>Taper Window</label>
                <select name="taper" value={params.taper} onChange={handleChange}>
                  <option value={0}>Uniform</option>
                  <option value={1}>Hamming</option>
                  <option value={2}>Hann</option>
                </select>
              </div>
              <div className="control-item mb-0">
                <label>Azimuth (°)</label>
                <input type="number" name="az" value={params.az} onChange={handleChange} />
              </div>
              <div className="control-item mb-0">
                <label>Elevation (°)</label>
                <input type="number" name="el" value={params.el} onChange={handleChange} />
              </div>
              <div className="control-item mb-0 col-span-2">
                <label>Steering Type</label>
                <select name="steerType" value={params.steerType} onChange={handleChange}>
                  <option value={1}>Phase Shifters</option>
                  <option value={2}>Time Delay</option>
                </select>
              </div>
            </div>
          </div>
        </div>

        <div>
          <div className="panel-title">🎯 Target Management</div>
          <div className="control-group">
            <div className="grid grid-cols-2 gap-2">
              <div className="control-item mb-0">
                <label>X (m)</label>
                <input 
                  type="number" 
                  value={tgtParams.x} 
                  onChange={(e) => setTgtParams(prev => ({ ...prev, x: parseFloat(e.target.value) }))} 
                />
              </div>
              <div className="control-item mb-0">
                <label>Y (m)</label>
                <input 
                  type="number" 
                  value={tgtParams.y} 
                  onChange={(e) => setTgtParams(prev => ({ ...prev, y: parseFloat(e.target.value) }))} 
                />
              </div>
              <div className="control-item mb-0">
                <label>Z (m)</label>
                <input 
                  type="number" 
                  value={tgtParams.z} 
                  onChange={(e) => setTgtParams(prev => ({ ...prev, z: parseFloat(e.target.value) }))} 
                />
              </div>
              <div className="control-item mb-0">
                <label>Vx (m/s)</label>
                <input 
                  type="number" 
                  value={tgtParams.vx} 
                  onChange={(e) => setTgtParams(prev => ({ ...prev, vx: parseFloat(e.target.value) }))} 
                />
              </div>
            </div>
            <button 
              onClick={() => {
                sendCommand('add_target', { 
                  id: Date.now(), 
                  x: tgtParams.x, 
                  y: tgtParams.y, 
                  z: tgtParams.z, 
                  vx: tgtParams.vx, 
                  vy: 0, 
                  vz: 0 
                });
              }}
              className="btn-primary bg-accent-green text-white w-full mt-4 py-1 text-xs"
            >
              + ADD DYNAMIC TARGET
            </button>
          </div>
        </div>

        <button onClick={handleRun} className="btn-primary bg-accent-blue text-white mt-auto">
          ▶ RUN SIMULATION
        </button>
        <div className="text-center text-xs mt-2 text-secondary flex justify-center items-center gap-2">
          <div className={`w-2 h-2 rounded-full ${status === 'CONNECTED' ? 'bg-accent-green' : 'bg-accent-red'}`}></div>
          WS: {status}
        </div>
      </aside>

      {/* Main Viewport */}
      <div className="flex-1 p-4 bg-background flex flex-col overflow-hidden">
        <div className="flex gap-1 mb-4 border-b border-border">
          <div 
            onClick={() => setActiveTab('c2')}
            className={`px-6 py-2 text-sm font-bold rounded-t-md cursor-pointer transition-colors ${activeTab === 'c2' ? 'bg-accent-blue text-white' : 'bg-sidebar text-secondary hover:bg-panel'}`}
          >
            C2 & Sensor Analysis
          </div>
          <div 
            onClick={() => setActiveTab('setup')}
            className={`px-6 py-2 text-sm font-bold rounded-t-md cursor-pointer transition-colors ${activeTab === 'setup' ? 'bg-accent-blue text-white' : 'bg-sidebar text-secondary hover:bg-panel'}`}
          >
            System Setup
          </div>
        </div>
        
        {activeTab === 'c2' && (
          <div className="grid grid-cols-2 grid-rows-2 gap-4 flex-1 min-h-0">
            <PlotCard 
              title="Range-Doppler Matrix" 
              data={data?.rd_matrix ? [{ z: data.rd_matrix, type: 'heatmap', colorscale: 'Viridis' }] : []} 
            />
            <PlotCard 
              title="IMM Tracker Airspace" 
              data={data?.track_history_x ? [
                {
                  x: data.track_history_x, y: data.track_history_y, z: data.track_history_z,
                  mode: 'lines+markers', type: 'scatter3d', name: 'Target Trajectory',
                  line: { color: '#f85149', width: 4 }, marker: { size: 4, color: '#e3b341' }
                },
                ...(data.ellipsoid_x ? [{
                  x: data.ellipsoid_x, y: data.ellipsoid_y, z: data.ellipsoid_z,
                  type: 'mesh3d', alphahull: 0, opacity: 0.2, color: '#2f81f7', name: 'Covariance Ellipsoid'
                }] : [])
              ] : []}
            />
            <PlotCard 
              title="3D Spatial Pattern" 
              data={data?.rd_matrix ? [{ z: data.rd_matrix, type: 'surface', colorscale: 'Viridis', showscale: false }] : []}
              scene={{ xaxis: {visible: false}, yaxis: {visible: false} }}
            />
            <PlotCard 
              title="Azimuth Pattern Cut (dB)" 
              data={data?.rd_matrix ? [{
                y: data.rd_matrix[Math.floor(data.rd_matrix.length/2)],
                mode: 'lines', line: { color: '#2f81f7', width: 2 }, fill: 'tozeroy'
              }] : []}
            />
          </div>
        )}

        {activeTab === 'setup' && (
          <div className="grid grid-cols-1 gap-4 flex-1 min-h-0">
            <div className="card p-4 font-mono text-xs text-accent-blue flex flex-col items-center justify-center">
              <div>[SYSTEM] Wideband STAP Module loaded.</div>
              {data?.stap_covariance_norm && <div>STAP Covariance Norm: {data.stap_covariance_norm}</div>}
              {data?.stap_weights && <div>Weights Computed: {data.stap_weights.length}</div>}
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
