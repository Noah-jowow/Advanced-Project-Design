
import PlotlyComponent from 'react-plotly.js';
import type { Data, Layout } from 'plotly.js';

// eslint-disable-next-line @typescript-eslint/no-explicit-any
const Plot = (PlotlyComponent as any).default || PlotlyComponent;

interface PlotCardProps {
  title: string;
  data: Partial<Data>[] | Record<string, unknown>[];
  layout?: Partial<Layout> | Record<string, unknown>;
  className?: string;
  scene?: Record<string, unknown>;
}

const themeLayout = {
  paper_bgcolor: 'rgba(0,0,0,0)',
  plot_bgcolor: 'rgba(0,0,0,0)',
  font: { family: 'Inter, sans-serif', color: 'rgba(235, 235, 245, 0.6)' },
  margin: { t: 40, r: 20, b: 40, l: 40 },
  xaxis: { gridcolor: 'rgba(255, 255, 255, 0.1)', zerolinecolor: 'rgba(255, 255, 255, 0.2)' },
  yaxis: { gridcolor: 'rgba(255, 255, 255, 0.1)', zerolinecolor: 'rgba(255, 255, 255, 0.2)' },
  autosize: true
};

const defaultScene = {
  xaxis: { title: 'X', color: '#FF453A', gridcolor: 'rgba(255, 255, 255, 0.1)', zerolinecolor: '#FF453A', zerolinewidth: 2, showspikes: false },
  yaxis: { title: 'Y', color: '#32D74B', gridcolor: 'rgba(255, 255, 255, 0.1)', zerolinecolor: '#32D74B', zerolinewidth: 2, showspikes: false },
  zaxis: { title: 'Z', color: '#0A84FF', gridcolor: 'rgba(255, 255, 255, 0.1)', zerolinecolor: '#0A84FF', zerolinewidth: 2, showspikes: false }
};

export function PlotCard({ title, data, layout = {}, className = '', scene = {} }: PlotCardProps) {
  return (
    <div className={`w-full h-full relative ${className}`}>
      <div className="absolute top-2 left-4 text-[11px] font-semibold text-text-secondary z-10 uppercase tracking-widest bg-surface px-2 py-1 rounded-md border border-border/50 backdrop-blur-md">
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
