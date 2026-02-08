import React from 'react';
import type { ParameterSummary, PPrimConfig } from '../types';

const AUTO_COLORS = ['#7aa2f7', '#f7768e', '#9ece6a', '#e0af68', '#bb9af7', '#73daca', '#ff9e64'];

interface Props {
  posterior?: Record<string, ParameterSummary>;
  pprimConfig?: PPrimConfig | null;
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

export default function PriorPosteriorPanel({ posterior, pprimConfig }: Props) {
  const params = posterior ? Object.entries(posterior) : [];

  // Build lookup from config
  const configLookup = new Map(
    (pprimConfig?.mappings ?? []).map(m => [m.parameter, m])
  );

  return (
    <div style={panelStyle}>
      <div style={headerStyle}>Prior vs Posterior</div>
      <div style={{ flex: 1, overflow: 'auto', padding: '8px 12px' }}>
        {params.length === 0 ? (
          <div style={{ color: '#565f89', fontSize: '13px', textAlign: 'center', paddingTop: '40px' }}>
            Run inference to see posterior distributions
          </div>
        ) : (
          params.map(([name, summary], idx) => {
            const mapping = configLookup.get(name);
            const color = mapping?.color || AUTO_COLORS[idx % AUTO_COLORS.length];
            const dist = mapping?.priorDist || '?';
            const range: [number, number] = mapping?.range || [
              Math.min(...summary.samples),
              Math.max(...summary.samples),
            ];
            return (
              <div key={name} style={{ marginBottom: '12px' }}>
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'baseline', marginBottom: '2px' }}>
                  <span style={{ fontSize: '12px', fontWeight: 600, color }}>{name}</span>
                  <span style={{ fontSize: '10px', color: '#565f89' }}>
                    Prior: {dist}
                  </span>
                </div>
                <MiniHistogram
                  samples={summary.samples}
                  color={color}
                  range={range}
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
