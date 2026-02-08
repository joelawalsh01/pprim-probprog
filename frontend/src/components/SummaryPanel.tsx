import React, { useState, useRef, useEffect } from 'react';
import type { InferenceResult, ReasoningStep } from '../types';

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
  display: 'flex',
  justifyContent: 'space-between',
  alignItems: 'center',
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

function TabButton({ active, onClick, children }: { active: boolean; onClick: () => void; children: React.ReactNode }) {
  return (
    <button
      onClick={onClick}
      style={{
        padding: '3px 10px',
        fontSize: '11px',
        fontWeight: active ? 600 : 400,
        background: active ? '#2a2b3d' : 'transparent',
        color: active ? '#c0caf5' : '#565f89',
        border: '1px solid',
        borderColor: active ? '#3b3d57' : 'transparent',
        borderRadius: '3px',
        cursor: 'pointer',
      }}
    >
      {children}
    </button>
  );
}

function ReasoningView({ steps }: { steps: ReasoningStep[] }) {
  const [expanded, setExpanded] = useState<string | null>(steps[0]?.parameter || null);

  return (
    <div style={{ padding: '12px', fontSize: '13px' }}>
      <p style={{ color: '#565f89', fontSize: '11px', marginBottom: '12px', lineHeight: 1.5 }}>
        Each parameter below was inferred by Bayesian inference (SVI). The system started from a
        prior distribution (initial assumption), observed the target trajectory, and found the
        posterior distribution (updated belief). Click a parameter to see how its interpretation was derived.
      </p>
      {steps.map((step) => {
        const isOpen = expanded === step.parameter;
        return (
          <div
            key={step.parameter}
            style={{
              marginBottom: '8px',
              border: '1px solid',
              borderColor: isOpen ? '#3b3d57' : '#2a2b3d',
              borderRadius: '6px',
              overflow: 'hidden',
            }}
          >
            {/* Accordion header */}
            <button
              onClick={() => setExpanded(isOpen ? null : step.parameter)}
              style={{
                width: '100%',
                padding: '8px 12px',
                background: isOpen ? '#1f2030' : '#16161e',
                border: 'none',
                cursor: 'pointer',
                display: 'flex',
                justifyContent: 'space-between',
                alignItems: 'center',
                textAlign: 'left',
              }}
            >
              <span style={{ color: '#c0caf5', fontFamily: 'monospace', fontSize: '12px', fontWeight: 600 }}>
                {step.parameter}
              </span>
              <span style={{ color: '#565f89', fontSize: '11px' }}>
                {step.prior_mean.toFixed(2)} → {step.posterior_mean.toFixed(4)}
                {' '}
                <span style={{ color: step.shift < 0 ? '#f7768e' : '#9ece6a' }}>
                  ({step.shift > 0 ? '+' : ''}{step.shift.toFixed(4)})
                </span>
              </span>
            </button>

            {/* Expanded content */}
            {isOpen && (
              <div style={{ padding: '12px', background: '#1a1b26' }}>
                {/* What this parameter means */}
                <div style={{ marginBottom: '10px' }}>
                  <div style={{ fontSize: '10px', color: '#7aa2f7', textTransform: 'uppercase', marginBottom: '4px', fontWeight: 600 }}>
                    What this parameter means
                  </div>
                  <p style={{ color: '#a9b1d6', lineHeight: 1.6, fontSize: '12px' }}>{step.rule}</p>
                </div>

                {/* Prior → Posterior shift */}
                <div style={{ marginBottom: '10px' }}>
                  <div style={{ fontSize: '10px', color: '#7aa2f7', textTransform: 'uppercase', marginBottom: '4px', fontWeight: 600 }}>
                    Prior → Posterior
                  </div>
                  <div style={{
                    display: 'grid',
                    gridTemplateColumns: '1fr auto 1fr',
                    gap: '8px',
                    alignItems: 'center',
                    padding: '8px',
                    background: '#16161e',
                    borderRadius: '4px',
                    fontSize: '12px',
                  }}>
                    <div style={{ textAlign: 'center' }}>
                      <div style={{ color: '#565f89', fontSize: '10px' }}>Prior</div>
                      <div style={{ color: '#a9b1d6', fontFamily: 'monospace' }}>{step.prior}</div>
                      <div style={{ color: '#7a7f96', fontSize: '11px' }}>mean ≈ {step.prior_mean.toFixed(2)}</div>
                    </div>
                    <div style={{ color: '#565f89', fontSize: '16px' }}>→</div>
                    <div style={{ textAlign: 'center' }}>
                      <div style={{ color: '#565f89', fontSize: '10px' }}>Posterior</div>
                      <div style={{ color: '#c0caf5', fontFamily: 'monospace', fontWeight: 600 }}>
                        {step.posterior_mean.toFixed(4)} ± {step.posterior_std.toFixed(4)}
                      </div>
                    </div>
                  </div>
                </div>

                {/* Threshold rules */}
                {step.thresholds.length > 0 && (
                  <div style={{ marginBottom: '10px' }}>
                    <div style={{ fontSize: '10px', color: '#7aa2f7', textTransform: 'uppercase', marginBottom: '4px', fontWeight: 600 }}>
                      Interpretation rules
                    </div>
                    {step.thresholds.map((t, i) => (
                      <div
                        key={i}
                        style={{
                          display: 'flex',
                          alignItems: 'flex-start',
                          gap: '8px',
                          padding: '4px 8px',
                          marginBottom: '2px',
                          background: t.triggered ? '#1a2030' : 'transparent',
                          borderRadius: '3px',
                          borderLeft: t.triggered ? '3px solid #7aa2f7' : '3px solid transparent',
                        }}
                      >
                        <span style={{
                          fontSize: '11px',
                          fontFamily: 'monospace',
                          color: t.triggered ? '#9ece6a' : '#3b3d57',
                          flexShrink: 0,
                          marginTop: '1px',
                        }}>
                          {t.triggered ? '●' : '○'}
                        </span>
                        <div>
                          <span style={{
                            fontSize: '11px',
                            fontFamily: 'monospace',
                            color: t.triggered ? '#c0caf5' : '#3b3d57',
                          }}>
                            {t.condition}
                          </span>
                          <span style={{
                            fontSize: '11px',
                            color: t.triggered ? '#a9b1d6' : '#3b3d57',
                            marginLeft: '8px',
                          }}>
                            — {t.label}
                          </span>
                        </div>
                      </div>
                    ))}
                  </div>
                )}

                {/* Conclusion */}
                <div style={{
                  padding: '8px',
                  background: '#1f2030',
                  borderRadius: '4px',
                  borderLeft: '3px solid #e0af68',
                }}>
                  <div style={{ fontSize: '10px', color: '#e0af68', textTransform: 'uppercase', marginBottom: '4px', fontWeight: 600 }}>
                    Conclusion
                  </div>
                  <p style={{ color: '#c0caf5', lineHeight: 1.5, fontSize: '12px', margin: 0 }}>
                    {step.conclusion}
                  </p>
                </div>
              </div>
            )}
          </div>
        );
      })}
    </div>
  );
}

function SummaryView({ inferResult }: { inferResult: InferenceResult }) {
  const { posterior, interpretation, losses, method, num_steps } = inferResult;
  const params = Object.entries(posterior);

  return (
    <div style={{ padding: '12px', fontSize: '13px' }}>
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
  );
}

export default function SummaryPanel({ inferResult, loading }: Props) {
  const [tab, setTab] = useState<'summary' | 'reasoning'>('summary');

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
            Run inference to see what physics priors explain the alternative trajectory
          </div>
        </div>
      </div>
    );
  }

  const hasReasoning = inferResult.interpretation?.reasoning && inferResult.interpretation.reasoning.length > 0;

  return (
    <div style={panelStyle}>
      <div style={headerStyle}>
        <span>Summary</span>
        {hasReasoning && (
          <div style={{ display: 'flex', gap: '4px' }}>
            <TabButton active={tab === 'summary'} onClick={() => setTab('summary')}>Results</TabButton>
            <TabButton active={tab === 'reasoning'} onClick={() => setTab('reasoning')}>Reasoning</TabButton>
          </div>
        )}
      </div>
      <div style={{ flex: 1, overflow: 'auto' }}>
        {tab === 'summary' ? (
          <SummaryView inferResult={inferResult} />
        ) : (
          <ReasoningView steps={inferResult.interpretation.reasoning || []} />
        )}
      </div>
    </div>
  );
}
