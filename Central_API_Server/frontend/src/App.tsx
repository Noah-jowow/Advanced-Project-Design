import { useState } from 'react';
import { RadarPanel } from './components/RadarPanel';
import { CFDPanel } from './components/CFDPanel';
import { PropulsionPanel } from './components/PropulsionPanel';
import { Activity, Wind, Flame } from 'lucide-react';

function App() {
  const [activeDomain, setActiveDomain] = useState<'RADAR' | 'CFD' | 'PROPULSION'>(() => {
    const params = new URLSearchParams(window.location.search);
    const proj = params.get('project');
    if (proj === 'CFD') return 'CFD';
    if (proj === 'PROPULSION') return 'PROPULSION';
    return 'RADAR'; // Default fallback
  });

  return (
    <div className="flex flex-col h-screen w-screen bg-background text-text-primary overflow-hidden font-sans">
      
      {/* Global Header (Glass Nav) */}
      <header className="flex items-center justify-between px-6 py-3 border-b border-border shadow-md z-10 bg-surface backdrop-blur-xl">
        <div className="text-xl font-bold tracking-tight flex items-center gap-3">
          <span>AEROSYS</span>
          <span className="text-text-secondary text-xs font-mono border-l border-border pl-3 hidden md:inline tracking-widest mt-1">
            {activeDomain === 'RADAR' && 'RADAR C2 WORKBENCH'}
            {activeDomain === 'CFD' && 'AERODYNAMICS WORKBENCH'}
            {activeDomain === 'PROPULSION' && 'ROCKET PROPULSION WORKBENCH'}
          </span>
        </div>
        
        {/* Navigation Controls */}
        <div className="flex items-center gap-3 bg-black/40 p-1 rounded-lg border border-border">
          <button 
            onClick={() => setActiveDomain('RADAR')}
            className={`flex items-center gap-2 px-4 py-1.5 rounded-md text-sm font-medium transition-all duration-300 ${activeDomain === 'RADAR' ? 'bg-surface-solid shadow-sm text-text-primary' : 'text-text-secondary hover:text-text-primary'}`}
          >
            <Activity size={16} /> Radar C2
          </button>
          <button 
            onClick={() => setActiveDomain('CFD')}
            className={`flex items-center gap-2 px-4 py-1.5 rounded-md text-sm font-medium transition-all duration-300 ${activeDomain === 'CFD' ? 'bg-surface-solid shadow-sm text-text-primary' : 'text-text-secondary hover:text-text-primary'}`}
          >
            <Wind size={16} /> Aerodynamics
          </button>
          <button 
            onClick={() => setActiveDomain('PROPULSION')}
            className={`flex items-center gap-2 px-4 py-1.5 rounded-md text-sm font-medium transition-all duration-300 ${activeDomain === 'PROPULSION' ? 'bg-surface-solid shadow-sm text-text-primary' : 'text-text-secondary hover:text-text-primary'}`}
          >
            <Flame size={16} /> Propulsion
          </button>
        </div>
      </header>

      {/* Domain Workbenches */}
      <main className="flex-1 overflow-hidden relative">
        {activeDomain === 'RADAR' && <RadarPanel />}
        {activeDomain === 'CFD' && <CFDPanel />}
        {activeDomain === 'PROPULSION' && <PropulsionPanel />}
      </main>
      
    </div>
  );
}

export default App;
