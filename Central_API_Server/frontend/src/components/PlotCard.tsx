import React from 'react';
import PlotlyComponent from 'react-plotly.js';

// Vite often bundles CommonJS default exports as an object { default: Component }
const Plot = (PlotlyComponent as any).default || PlotlyComponent;

interface PlotCardProps {
  title: string;
  data: any[];
  layout?: any;
  className?: string;
  scene?: any;
}

const themeLayout = {
  paper_bgcolor: 'rgba(0,0,0,0)',
  plot_bgcolor: 'rgba(0,0,0,0)',
  font: { family: 'Outfit, sans-serif', color: '#c9d1d9' },
  margin: { t: 40, r: 20, b: 40, l: 40 },
  xaxis: { gridcolor: '#30363d', zerolinecolor: '#30363d' },
  yaxis: { gridcolor: '#30363d', zerolinecolor: '#30363d' },
  autosize: true
};

const defaultScene = {
  xaxis: { title: 'X', color: '#ff4444', gridcolor: '#30363d', zerolinecolor: '#ff4444', zerolinewidth: 4, showspikes: false },
  yaxis: { title: 'Y', color: '#44ff44', gridcolor: '#30363d', zerolinecolor: '#44ff44', zerolinewidth: 4, showspikes: false },
  zaxis: { title: 'Z', color: '#4444ff', gridcolor: '#30363d', zerolinecolor: '#4444ff', zerolinewidth: 4, showspikes: false }
};

export function PlotCard({ title, data, layout = {}, className = '', scene = {} }: PlotCardProps) {
  return (
    <div className={`card w-full h-full ${className}`}>
      <div className="absolute top-2 left-4 text-xs font-bold text-secondary z-10 bg-panel px-2 py-1 rounded">
        {title}
      </div>
      <div className="flex-1 w-full h-full p-2">
        <Plot
          data={data}
          layout={{ ...themeLayout, ...layout, scene: { ...defaultScene, ...scene } }}
          config={{ responsive: true, displayModeBar: true }}
          style={{ width: '100%', height: '100%' }}
          useResizeHandler={true}
        />
      </div>
    </div>
  );
}
