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
    <div className="flex flex-col h-screen w-screen bg-background text-primary overflow-hidden font-sans">
      
      {/* Global Header */}
      <header className="flex items-center justify-between px-4 py-2 bg-sidebar border-b border-border shadow-md z-10">
        <div className="text-lg font-extrabold tracking-wider flex items-center gap-3">
          <span>AEROSYS <span className="text-accent-blue">SYSTEM OF SYSTEMS</span></span>
          <span className="text-secondary text-sm font-mono border-l border-border pl-3">
            {activeDomain === 'RADAR' && 'RADAR C2 WORKBENCH'}
            {activeDomain === 'CFD' && 'AERODYNAMICS WORKBENCH'}
            {activeDomain === 'PROPULSION' && 'ROCKET PROPULSION WORKBENCH'}
          </span>
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
