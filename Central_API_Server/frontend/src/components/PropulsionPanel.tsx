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
    <div className="flex h-full w-full">
      {/* Sidebar */}
      <aside className="w-80 bg-sidebar border-r border-border p-4 overflow-y-auto flex flex-col gap-6">
        <div>
          <div className="panel-title">🔥 Propellant & Combustion</div>
          <div className="control-group">
            <div className="grid grid-cols-2 gap-2">
              <div className="control-item mb-0">
                <label>Thrust [N]</label>
                <input type="number" name="thrust" value={params.thrust} onChange={handleChange} step="1000" />
              </div>
              <div className="control-item mb-0">
                <label>Pressure [Pa]</label>
                <input type="number" name="pa" value={params.pa} onChange={handleChange} step="100" />
              </div>
              <div className="control-item mb-0 col-span-2">
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
          <div className="panel-title">🛠️ Material & Structure</div>
          <div className="control-group">
            <div className="grid grid-cols-2 gap-2">
              <div className="control-item mb-0 col-span-2">
                <label>Wall Material</label>
                <select name="mat" value={params.mat} onChange={handleChange}>
                  <option value={0}>Inconel 718</option>
                  <option value={1}>GRCop-42</option>
                  <option value={2}>C103 Niobium</option>
                </select>
              </div>
              <div className="control-item mb-0 col-span-2">
                <label>Max GA Generations</label>
                <input type="number" name="gens" value={params.gens} onChange={handleChange} />
              </div>
            </div>
          </div>
        </div>

        <div>
          <div className="panel-title">⚙️ Advanced Constraints</div>
          <div className="control-group">
            <div className="grid grid-cols-2 gap-2">
              <div className="control-item mb-0">
                <label>Pc Min [Pa]</label>
                <input type="number" name="pcMin" value={params.pcMin} onChange={handleChange} step="100000" />
              </div>
              <div className="control-item mb-0">
                <label>Pc Max [Pa]</label>
                <input type="number" name="pcMax" value={params.pcMax} onChange={handleChange} step="100000" />
              </div>
              <div className="control-item mb-0">
                <label>t_w Min [m]</label>
                <input type="number" name="twMin" value={params.twMin} onChange={handleChange} step="0.001" />
              </div>
              <div className="control-item mb-0">
                <label>t_w Max [m]</label>
                <input type="number" name="twMax" value={params.twMax} onChange={handleChange} step="0.001" />
              </div>
            </div>
          </div>
        </div>

        <button onClick={handleRun} className="btn-primary bg-accent-red text-white mt-auto">
          🚀 RUN VECTOR SOLVER
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
            onClick={() => setActiveTab('analytics')}
            className={`px-6 py-2 text-sm font-bold rounded-t-md cursor-pointer transition-colors ${activeTab === 'analytics' ? 'bg-accent-red text-white' : 'bg-sidebar text-secondary hover:bg-panel'}`}
          >
            Optimizer Analytics
          </div>
          <div 
            onClick={() => setActiveTab('cooling')}
            className={`px-6 py-2 text-sm font-bold rounded-t-md cursor-pointer transition-colors ${activeTab === 'cooling' ? 'bg-accent-red text-white' : 'bg-sidebar text-secondary hover:bg-panel'}`}
          >
            Cooling Jackets
          </div>
        </div>
        
        {activeTab === 'analytics' && (
          <div className="grid grid-cols-2 grid-rows-2 gap-4 flex-1 min-h-0">
            <PlotCard 
              title="Rao TOP Geometry" 
              data={data?.geometry_x ? [
                { x: data.geometry_x, y: data.geometry_y, name: 'Nozzle Wall', line: {color: '#f85149', width: 3} },
                { x: data.geometry_x, y: data.geometry_y.map((y: number) => -y), showlegend: false, line: {color: '#f85149', width: 3} }
              ] : []}
              layout={{ xaxis: {scaleratio: 1, scaleanchor: 'y'} }}
            />
            <PlotCard 
              title="Centerline Mach Number" 
              data={data?.mach_dist ? [{ x: data.geometry_x, y: data.mach_dist, line: {color: '#2f81f7'} }] : []}
            />
            <PlotCard 
              title="Thermal Environment (K)" 
              data={data?.temp ? [
                { x: data.geometry_x, y: data.temp, name: 'Gas Temp', line: {color: '#e3b341'} },
                { x: data.geometry_x, y: data.t_hw, name: 'Hot Wall', line: {color: '#f85149'} }
              ] : []}
            />
            <PlotCard 
              title="Structural Margin of Safety" 
              data={data?.margin_of_safety ? [{
                x: data.geometry_x, y: data.margin_of_safety, fill: 'tozeroy', line: {color: '#2ea043'}
              }] : []}
            />
          </div>
        )}

        {activeTab === 'cooling' && (
          <div className="grid grid-cols-1 gap-4 flex-1 min-h-0">
            <div className="card p-4 font-mono text-xs text-accent-red flex items-center justify-center">
              <div>
                [SYSTEM] Thermal analysis indicates T_hw exceeds melting point limits in throat region.<br/>
                [RECOMMENDATION] Increase wall thickness or switch to C103 Niobium.
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
