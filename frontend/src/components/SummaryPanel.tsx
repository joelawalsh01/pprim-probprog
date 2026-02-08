import React, { useRef, useEffect } from 'react';
import type { InferenceResult } from '../types';

interface Props {
  inferResult: InferenceResult | null;
  loading: boolean;
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
  color: '#f7768e',
  textTransform: 'uppercase',
  letterSpacing: '0.05em',
  borderBottom: '1px solid #2a2b3d',
  flexShrink: 0,
};

function ConvergencePlot({ losses }: { losses: number[] }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || losses.length === 0) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    const maxLoss = Math.max(...losses);
    const minLoss = Math.min(...losses);
    const range = maxLoss - minLoss || 1;

    ctx.beginPath();
    ctx.strokeStyle = '#e0af68';
    ctx.lineWidth = 1.5;
    losses.forEach((loss, i) => {
      const x = (i / (losses.length - 1)) * w;
      const y = h - ((loss - minLoss) / range) * (h - 10) - 5;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }, [losses]);

  return (
    <canvas
      ref={canvasRef}
      width={250}
      height={60}
      style={{ width: '100%', height: '60px', borderRadius: '4px', background: '#16161e' }}
    />
  );
}

export default function SummaryPanel({ inferResult, loading }: Props) {
  if (loading) {
    return (
      <div style={panelStyle}>
        <div style={headerStyle}>Summary</div>
        <div style={{ flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
          <div style={{ color: '#bb9af7', fontSize: '14px' }}>
            Running inference...
            <div style={{ fontSize: '12px', color: '#565f89', marginTop: '8px' }}>
              This may take a minute
            </div>
          </div>
        </div>
      </div>
    );
  }

  if (!inferResult) {
    return (
      <div style={panelStyle}>
        <div style={headerStyle}>Summary</div>
        <div style={{ flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
          <div style={{ color: '#565f89', fontSize: '13px', textAlign: 'center', padding: '0 20px' }}>
            Run inference to see what physics priors explain the naive trajectory
          </div>
        </div>
      </div>
    );
  }

  const { posterior, interpretation, losses, method, num_steps } = inferResult;
  const params = Object.entries(posterior);

  return (
    <div style={panelStyle}>
      <div style={headerStyle}>Summary</div>
      <div style={{ flex: 1, overflow: 'auto', padding: '12px', fontSize: '13px' }}>
        {/* Parameter table */}
        <table style={{ width: '100%', borderCollapse: 'collapse', marginBottom: '12px' }}>
          <thead>
            <tr style={{ borderBottom: '1px solid #2a2b3d' }}>
              <th style={{ textAlign: 'left', padding: '4px 8px', color: '#565f89', fontWeight: 500 }}>Parameter</th>
              <th style={{ textAlign: 'right', padding: '4px 8px', color: '#565f89', fontWeight: 500 }}>Mean</th>
              <th style={{ textAlign: 'right', padding: '4px 8px', color: '#565f89', fontWeight: 500 }}>Std</th>
              <th style={{ textAlign: 'right', padding: '4px 8px', color: '#565f89', fontWeight: 500 }}>90% CI</th>
            </tr>
          </thead>
          <tbody>
            {params.map(([name, s]) => (
              <tr key={name} style={{ borderBottom: '1px solid #1f2030' }}>
                <td style={{ padding: '4px 8px', color: '#a9b1d6', fontFamily: 'monospace', fontSize: '11px' }}>{name}</td>
                <td style={{ textAlign: 'right', padding: '4px 8px', color: '#c0caf5' }}>{s.mean.toFixed(4)}</td>
                <td style={{ textAlign: 'right', padding: '4px 8px', color: '#7a7f96' }}>{s.std.toFixed(4)}</td>
                <td style={{ textAlign: 'right', padding: '4px 8px', color: '#7a7f96', fontSize: '11px' }}>
                  [{s.q05.toFixed(3)}, {s.q95.toFixed(3)}]
                </td>
              </tr>
            ))}
          </tbody>
        </table>

        {/* Convergence */}
        {losses && losses.length > 0 && (
          <div style={{ marginBottom: '12px' }}>
            <div style={{ fontSize: '11px', color: '#565f89', marginBottom: '4px' }}>
              ELBO Loss ({method?.toUpperCase()}, {num_steps} steps)
            </div>
            <ConvergencePlot losses={losses} />
          </div>
        )}

        {/* Interpretation */}
        {interpretation && (
          <div>
            <div style={{ fontSize: '11px', color: '#565f89', marginBottom: '6px', textTransform: 'uppercase' }}>
              Interpretation
            </div>
            {interpretation.findings.map((f, i) => (
              <p key={i} style={{ color: '#a9b1d6', lineHeight: 1.5, marginBottom: '8px' }}>{f}</p>
            ))}
            <p style={{
              color: '#c0caf5',
              lineHeight: 1.5,
              padding: '8px',
              background: '#1f2030',
              borderRadius: '4px',
              borderLeft: '3px solid #7aa2f7',
            }}>
              {interpretation.summary}
            </p>
          </div>
        )}
      </div>
    </div>
  );
}
