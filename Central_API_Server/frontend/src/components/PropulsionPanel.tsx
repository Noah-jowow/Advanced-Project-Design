import React, { useState } from 'react';
import { useWebSocket } from '../hooks/useWebSocket';
import { PlotCard } from './PlotCard';

export function PropulsionPanel() {
  const { status, data, sendCommand } = useWebSocket('prop');
  
  const [params, setParams] = useState({
    thrust: 50000, pa: 101325, prop: 0, mat: 0, gens: 10,
    pcMin: 1000000, pcMax: 21000000, twMin: 0.001, twMax: 0.011
  });

  const [activeTab, setActiveTab] = useState('analytics');

  const handleRun = () => {
    sendCommand('optimize', {
      thrust: params.thrust,
      pa: params.pa,
      prop: params.prop,
      mat: params.mat,
      gens: params.gens,
      pcMin: params.pcMin,
      pcMax: params.pcMax,
      twMin: params.twMin,
      twMax: params.twMax
    });
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setParams(prev => ({ ...prev, [name]: isNaN(Number(value)) ? value : Number(value) }));
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

        <div>
          <div className="panel-title">Propellant & Combustion</div>
          <div className="control-group">
            <div className="grid grid-cols-2 gap-3">
              <div className="control-item">
                <label>Thrust [N]</label>
                <input type="number" name="thrust" value={params.thrust} onChange={handleChange} step="1000" />
              </div>
              <div className="control-item">
                <label>Pressure [Pa]</label>
                <input type="number" name="pa" value={params.pa} onChange={handleChange} step="100" />
              </div>
              <div className="control-item col-span-2">
                <label>Propellant System</label>
                <select name="prop" value={params.prop} onChange={handleChange}>
                  <option value={0}>LOX / RP-1</option>
                  <option value={1}>LOX / LH2</option>
                  <option value={2}>LOX / LCH4</option>
                </select>
              </div>
            </div>
          </div>
        </div>

        <div>
          <div className="panel-title">Material & Structure</div>
          <div className="control-group">
            <div className="grid grid-cols-2 gap-3">
              <div className="control-item col-span-2">
                <label>Wall Material</label>
                <select name="mat" value={params.mat} onChange={handleChange}>
                  <option value={0}>Inconel 718</option>
                  <option value={1}>GRCop-42</option>
                  <option value={2}>C103 Niobium</option>
                </select>
              </div>
              <div className="control-item col-span-2">
                <label>Max GA Generations</label>
                <input type="number" name="gens" value={params.gens} onChange={handleChange} />
              </div>
            </div>
          </div>
        </div>

        <div>
          <div className="panel-title">Advanced Constraints</div>
          <div className="control-group">
            <div className="grid grid-cols-2 gap-3">
              <div className="control-item">
                <label>Pc Min [Pa]</label>
                <input type="number" name="pcMin" value={params.pcMin} onChange={handleChange} step="100000" />
              </div>
              <div className="control-item">
                <label>Pc Max [Pa]</label>
                <input type="number" name="pcMax" value={params.pcMax} onChange={handleChange} step="100000" />
              </div>
              <div className="control-item">
                <label>t_w Min [m]</label>
                <input type="number" name="twMin" value={params.twMin} onChange={handleChange} step="0.001" />
              </div>
              <div className="control-item">
                <label>t_w Max [m]</label>
                <input type="number" name="twMax" value={params.twMax} onChange={handleChange} step="0.001" />
              </div>
            </div>
          </div>
        </div>

        <div className="mt-auto pt-4">
          <button 
            onClick={handleRun} 
            disabled={status !== 'CONNECTED'} 
            className="w-full py-2.5 px-4 bg-accent-red text-white border-none rounded-lg font-semibold text-sm cursor-pointer transition-all duration-200 hover:bg-[#FF3B30] active:scale-[0.98] shadow-[0_0_15px_rgba(255,69,58,0.3)] disabled:opacity-50 disabled:cursor-not-allowed"
          >
            RUN VECTOR SOLVER
          </button>
        </div>
      </aside>

      {/* Main Viewport */}
      <div className="flex-1 flex flex-col gap-6 overflow-hidden">
        
        {/* Tab Navigation (Glass) */}
        <div className="flex gap-4">
          <button 
            className={`px-6 py-2.5 rounded-xl text-sm font-semibold transition-all duration-300 border ${activeTab === 'analytics' ? 'bg-surface-hover border-border-strong text-text-primary shadow-lg' : 'bg-surface border-border/50 text-text-secondary hover:text-text-primary hover:bg-white/[0.05]'}`}
            onClick={() => setActiveTab('analytics')}
          >
            Optimizer Analytics
          </button>
          <button 
            className={`px-6 py-2.5 rounded-xl text-sm font-semibold transition-all duration-300 border ${activeTab === 'cooling' ? 'bg-surface-hover border-border-strong text-text-primary shadow-lg' : 'bg-surface border-border/50 text-text-secondary hover:text-text-primary hover:bg-white/[0.05]'}`}
            onClick={() => setActiveTab('cooling')}
          >
            Thermal & Cooling
          </button>
        </div>
        
        {activeTab === 'analytics' && (
          <div className="grid grid-cols-2 grid-rows-2 gap-6 flex-1 min-h-0">
            <div className="card p-2">
              <PlotCard 
                title="Rao TOP Geometry" 
                data={data?.geometry_x ? [
                  { x: data.geometry_x, y: data.geometry_y, name: 'Nozzle Wall', line: {color: '#FF453A', width: 3} },
                  { x: data.geometry_x, y: (data.geometry_y as number[]).map((y: number) => -y), showlegend: false, line: {color: '#FF453A', width: 3} }
                ] : []}
                layout={{ xaxis: {scaleratio: 1, scaleanchor: 'y', gridcolor: '#2c2c2e'}, yaxis: {gridcolor: '#2c2c2e'} }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="Centerline Mach Number" 
                data={data?.mach_dist ? [{ x: data.geometry_x, y: data.mach_dist, line: {color: '#0A84FF'} }] : []}
                layout={{ xaxis: {gridcolor: '#2c2c2e'}, yaxis: {gridcolor: '#2c2c2e'} }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="Thermal Environment (K)" 
                data={data?.temp ? [
                  { x: data.geometry_x, y: data.temp, name: 'Gas Temp', line: {color: '#FF9F0A'} },
                  { x: data.geometry_x, y: data.t_hw, name: 'Hot Wall', line: {color: '#FF453A'} }
                ] : []}
                layout={{ xaxis: {gridcolor: '#2c2c2e'}, yaxis: {gridcolor: '#2c2c2e'} }}
              />
            </div>
            <div className="card p-2">
              <PlotCard 
                title="Structural Margin of Safety" 
                data={data?.margin_of_safety ? [{
                  x: data.geometry_x, y: data.margin_of_safety, fill: 'tozeroy', line: {color: '#32D74B'}, fillcolor: 'rgba(50, 215, 75, 0.2)'
                }] : []}
                layout={{ xaxis: {gridcolor: '#2c2c2e'}, yaxis: {gridcolor: '#2c2c2e'} }}
              />
            </div>
          </div>
        )}

        {activeTab === 'cooling' && (
          <div className="flex-1 card p-8 min-h-0 flex items-center justify-center">
            <div className="font-mono text-sm text-accent-red flex flex-col gap-3">
              <div>{'>'} [SYSTEM] Thermal analysis indicates T_hw exceeds melting point limits in throat region.</div>
              <div className="text-text-primary">{'>'} [RECOMMENDATION] Increase wall thickness or switch to C103 Niobium.</div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
