import React from 'react';
import type { ParameterSummary } from '../types';

interface Props {
  posterior?: Record<string, ParameterSummary>;
}

const panelStyle: React.CSSProperties = {
  background: '#1a1b26',
  display: 'flex',
  flexDirection: 'column',
  overflow: 'hidden',
};

const headerStyle: React.CSSProperties = {
  padding: '8px 12px',
  fontSize: '12px',
  fontWeight: 600,
  color: '#e0af68',
  textTransform: 'uppercase',
  letterSpacing: '0.05em',
  borderBottom: '1px solid #2a2b3d',
  flexShrink: 0,
};

const PARAM_COLORS: Record<string, string> = {
  velocity_persistence: '#7aa2f7',
  lateral_damping: '#f7768e',
  force_magnitude: '#9ece6a',
  mass: '#e0af68',
  sigma: '#bb9af7',
};

const PRIOR_INFO: Record<string, { dist: string; range: [number, number] }> = {
  velocity_persistence: { dist: 'Beta(2,2)', range: [0, 1] },
  lateral_damping: { dist: 'LogNormal(0,1)', range: [0, 10] },
  force_magnitude: { dist: 'LogNormal(1,0.5)', range: [0, 20] },
  mass: { dist: 'LogNormal(0,0.5)', range: [0, 5] },
  sigma: { dist: 'LogNormal(-2,1)', range: [0, 1] },
};

function MiniHistogram({ samples, color, range }: { samples: number[]; color: string; range: [number, number] }) {
  const nBins = 30;
  const [lo, hi] = range;
  const bins = new Array(nBins).fill(0);
  for (const v of samples) {
    const idx = Math.floor(((v - lo) / (hi - lo)) * nBins);
    if (idx >= 0 && idx < nBins) bins[idx]++;
  }
  const maxBin = Math.max(...bins, 1);
  const barW = 100 / nBins;

  return (
    <svg viewBox="0 0 100 30" style={{ width: '100%', height: '40px' }} preserveAspectRatio="none">
      {bins.map((count, i) => (
        <rect
          key={i}
          x={i * barW}
          y={30 - (count / maxBin) * 28}
          width={barW * 0.85}
          height={(count / maxBin) * 28}
          fill={color}
          opacity={0.7}
        />
      ))}
    </svg>
  );
}

export default function PriorPosteriorPanel({ posterior }: Props) {
  const params = posterior ? Object.entries(posterior) : [];

  return (
    <div style={panelStyle}>
      <div style={headerStyle}>Prior vs Posterior</div>
      <div style={{ flex: 1, overflow: 'auto', padding: '8px 12px' }}>
        {params.length === 0 ? (
          <div style={{ color: '#565f89', fontSize: '13px', textAlign: 'center', paddingTop: '40px' }}>
            Run inference to see posterior distributions
          </div>
        ) : (
          params.map(([name, summary]) => {
            const color = PARAM_COLORS[name] || '#a9b1d6';
            const info = PRIOR_INFO[name];
            return (
              <div key={name} style={{ marginBottom: '12px' }}>
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'baseline', marginBottom: '2px' }}>
                  <span style={{ fontSize: '12px', fontWeight: 600, color }}>{name}</span>
                  <span style={{ fontSize: '10px', color: '#565f89' }}>
                    Prior: {info?.dist || '?'}
                  </span>
                </div>
                <MiniHistogram
                  samples={summary.samples}
                  color={color}
                  range={info?.range || [Math.min(...summary.samples), Math.max(...summary.samples)]}
                />
                <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '10px', color: '#a9b1d6' }}>
                  <span>mean: {summary.mean.toFixed(3)}</span>
                  <span>std: {summary.std.toFixed(3)}</span>
                  <span>[{summary.q05.toFixed(3)}, {summary.q95.toFixed(3)}]</span>
                </div>
              </div>
            );
          })
        )}
      </div>
    </div>
  );
}
