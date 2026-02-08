import React, { Suspense, useState, useCallback } from 'react';
import type { PPrimConfig, PPrimMapping, PPrimThreshold } from '../types';

const MonacoEditor = React.lazy(() => import('@monaco-editor/react'));

interface Props {
  code: string;
  onChange: (code: string) => void;
  pprimConfig: PPrimConfig | null;
  onPPrimConfigChange: (config: PPrimConfig | null) => void;
  onInfer: () => void;
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
  color: '#bb9af7',
  textTransform: 'uppercase',
  letterSpacing: '0.05em',
  borderBottom: '1px solid #2a2b3d',
  display: 'flex',
  justifyContent: 'space-between',
  alignItems: 'center',
  flexShrink: 0,
};

const buttonStyle: React.CSSProperties = {
  padding: '4px 14px',
  fontSize: '12px',
  fontWeight: 600,
  background: '#5a3d7a',
  color: '#c0caf5',
  border: 'none',
  borderRadius: '4px',
  cursor: 'pointer',
};

const tabBtnStyle = (active: boolean): React.CSSProperties => ({
  padding: '3px 10px',
  fontSize: '11px',
  fontWeight: active ? 600 : 400,
  background: active ? '#2a2b3d' : 'transparent',
  color: active ? '#c0caf5' : '#565f89',
  border: '1px solid',
  borderColor: active ? '#3b3d57' : 'transparent',
  borderRadius: '3px',
  cursor: 'pointer',
});

const smallBtnStyle: React.CSSProperties = {
  padding: '2px 8px',
  fontSize: '10px',
  fontWeight: 600,
  background: '#2a2b3d',
  color: '#a9b1d6',
  border: '1px solid #3b3d57',
  borderRadius: '3px',
  cursor: 'pointer',
};

const inputStyle: React.CSSProperties = {
  background: '#16161e',
  border: '1px solid #2a2b3d',
  borderRadius: '3px',
  color: '#c0caf5',
  fontSize: '11px',
  padding: '3px 6px',
  width: '100%',
  outline: 'none',
  fontFamily: 'monospace',
};

const labelStyle: React.CSSProperties = {
  fontSize: '10px',
  color: '#565f89',
  marginBottom: '2px',
  display: 'block',
};

function emptyMapping(): PPrimMapping {
  return {
    parameter: '',
    pprimName: '',
    description: '',
    priorDist: '',
    priorMean: 0,
    range: [0, 1],
    color: undefined,
    thresholds: [],
    lowConclusion: '',
    highConclusion: '',
  };
}

function emptyThreshold(): PPrimThreshold {
  return { min: null, max: null, label: '', conception: 'neutral' };
}

function scanCodeForSamples(code: string): PPrimMapping[] {
  const regex = /pyro\.sample\(\s*["'](\w+)["']\s*,\s*dist\.(\w+)\(([^)]*)\)/g;
  const mappings: PPrimMapping[] = [];
  let match;
  while ((match = regex.exec(code)) !== null) {
    const name = match[1];
    if (name === 'obs') continue;
    const distName = match[2];
    const args = match[3];
    mappings.push({
      parameter: name,
      pprimName: '',
      description: '',
      priorDist: `${distName}(${args.trim()})`,
      priorMean: 0,
      range: [0, 1],
      thresholds: [],
      lowConclusion: '',
      highConclusion: '',
    });
  }
  return mappings;
}

function ThresholdRow({
  threshold,
  onChange,
  onDelete,
}: {
  threshold: PPrimThreshold;
  onChange: (t: PPrimThreshold) => void;
  onDelete: () => void;
}) {
  return (
    <div style={{ display: 'flex', gap: '4px', alignItems: 'center', marginBottom: '4px' }}>
      <input
        style={{ ...inputStyle, width: '50px' }}
        placeholder="min"
        value={threshold.min ?? ''}
        onChange={e => onChange({ ...threshold, min: e.target.value === '' ? null : Number(e.target.value) })}
      />
      <input
        style={{ ...inputStyle, width: '50px' }}
        placeholder="max"
        value={threshold.max ?? ''}
        onChange={e => onChange({ ...threshold, max: e.target.value === '' ? null : Number(e.target.value) })}
      />
      <input
        style={{ ...inputStyle, flex: 1 }}
        placeholder="label"
        value={threshold.label}
        onChange={e => onChange({ ...threshold, label: e.target.value })}
      />
      <select
        style={{ ...inputStyle, width: '80px' }}
        value={threshold.conception}
        onChange={e => onChange({ ...threshold, conception: e.target.value })}
      >
        <option value="neutral">neutral</option>
        <option value="alternative">alternative</option>
        <option value="newtonian">newtonian</option>
        <option value="mixed">mixed</option>
      </select>
      <button
        onClick={onDelete}
        style={{ ...smallBtnStyle, color: '#f7768e', padding: '2px 5px' }}
        title="Remove threshold"
      >
        x
      </button>
    </div>
  );
}

function MappingCard({
  mapping,
  onChange,
  onDelete,
}: {
  mapping: PPrimMapping;
  onChange: (m: PPrimMapping) => void;
  onDelete: () => void;
}) {
  const [collapsed, setCollapsed] = useState(false);

  const updateThreshold = (idx: number, t: PPrimThreshold) => {
    const next = [...mapping.thresholds];
    next[idx] = t;
    onChange({ ...mapping, thresholds: next });
  };

  const deleteThreshold = (idx: number) => {
    onChange({ ...mapping, thresholds: mapping.thresholds.filter((_, i) => i !== idx) });
  };

  return (
    <div style={{
      border: '1px solid #2a2b3d',
      borderRadius: '6px',
      marginBottom: '8px',
      overflow: 'hidden',
    }}>
      <div
        onClick={() => setCollapsed(!collapsed)}
        style={{
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
          padding: '6px 10px',
          background: '#16161e',
          cursor: 'pointer',
        }}
      >
        <span style={{ fontSize: '12px', fontFamily: 'monospace', color: '#c0caf5', fontWeight: 600 }}>
          {mapping.parameter || '(unnamed)'}
          {mapping.pprimName && (
            <span style={{ color: '#7aa2f7', fontWeight: 400, marginLeft: '8px', fontFamily: 'inherit' }}>
              {mapping.pprimName}
            </span>
          )}
        </span>
        <div style={{ display: 'flex', gap: '4px', alignItems: 'center' }}>
          <span style={{ fontSize: '10px', color: '#565f89' }}>{collapsed ? '+' : '-'}</span>
          <button
            onClick={e => { e.stopPropagation(); onDelete(); }}
            style={{ ...smallBtnStyle, color: '#f7768e', padding: '2px 5px' }}
            title="Remove mapping"
          >
            x
          </button>
        </div>
      </div>
      {!collapsed && (
        <div style={{ padding: '8px 10px', display: 'flex', flexDirection: 'column', gap: '6px' }}>
          <div style={{ display: 'flex', gap: '6px' }}>
            <div style={{ flex: 1 }}>
              <label style={labelStyle}>Parameter name</label>
              <input
                style={inputStyle}
                value={mapping.parameter}
                onChange={e => onChange({ ...mapping, parameter: e.target.value })}
              />
            </div>
            <div style={{ flex: 1 }}>
              <label style={labelStyle}>P-prim name</label>
              <input
                style={inputStyle}
                value={mapping.pprimName}
                onChange={e => onChange({ ...mapping, pprimName: e.target.value })}
              />
            </div>
          </div>
          <div>
            <label style={labelStyle}>Description</label>
            <textarea
              style={{ ...inputStyle, height: '40px', resize: 'vertical', fontFamily: 'inherit' }}
              value={mapping.description}
              onChange={e => onChange({ ...mapping, description: e.target.value })}
            />
          </div>
          <div style={{ display: 'flex', gap: '6px' }}>
            <div style={{ flex: 1 }}>
              <label style={labelStyle}>Prior dist</label>
              <input
                style={inputStyle}
                value={mapping.priorDist}
                onChange={e => onChange({ ...mapping, priorDist: e.target.value })}
                placeholder="e.g. Beta(2,2)"
              />
            </div>
            <div style={{ width: '60px' }}>
              <label style={labelStyle}>Prior mean</label>
              <input
                style={inputStyle}
                type="number"
                step="any"
                value={mapping.priorMean}
                onChange={e => onChange({ ...mapping, priorMean: Number(e.target.value) })}
              />
            </div>
            <div style={{ width: '60px' }}>
              <label style={labelStyle}>Range min</label>
              <input
                style={inputStyle}
                type="number"
                step="any"
                value={mapping.range[0]}
                onChange={e => onChange({ ...mapping, range: [Number(e.target.value), mapping.range[1]] })}
              />
            </div>
            <div style={{ width: '60px' }}>
              <label style={labelStyle}>Range max</label>
              <input
                style={inputStyle}
                type="number"
                step="any"
                value={mapping.range[1]}
                onChange={e => onChange({ ...mapping, range: [mapping.range[0], Number(e.target.value)] })}
              />
            </div>
            <div style={{ width: '70px' }}>
              <label style={labelStyle}>Color</label>
              <input
                style={{ ...inputStyle, padding: '1px' }}
                type="color"
                value={mapping.color || '#7aa2f7'}
                onChange={e => onChange({ ...mapping, color: e.target.value })}
              />
            </div>
          </div>
          <div>
            <label style={labelStyle}>Thresholds</label>
            <div style={{ fontSize: '9px', color: '#3b3d57', marginBottom: '4px' }}>
              min | max | label | conception
            </div>
            {mapping.thresholds.map((t, i) => (
              <ThresholdRow
                key={i}
                threshold={t}
                onChange={nt => updateThreshold(i, nt)}
                onDelete={() => deleteThreshold(i)}
              />
            ))}
            <button
              style={smallBtnStyle}
              onClick={() => onChange({ ...mapping, thresholds: [...mapping.thresholds, emptyThreshold()] })}
            >
              + Threshold
            </button>
          </div>
          <div style={{ display: 'flex', gap: '6px' }}>
            <div style={{ flex: 1 }}>
              <label style={labelStyle}>Low conclusion</label>
              <textarea
                style={{ ...inputStyle, height: '32px', resize: 'vertical', fontFamily: 'inherit' }}
                value={mapping.lowConclusion}
                onChange={e => onChange({ ...mapping, lowConclusion: e.target.value })}
              />
            </div>
            <div style={{ flex: 1 }}>
              <label style={labelStyle}>High conclusion</label>
              <textarea
                style={{ ...inputStyle, height: '32px', resize: 'vertical', fontFamily: 'inherit' }}
                value={mapping.highConclusion}
                onChange={e => onChange({ ...mapping, highConclusion: e.target.value })}
              />
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

function PPrimEditor({
  config,
  onConfigChange,
  code,
}: {
  config: PPrimConfig | null;
  onConfigChange: (config: PPrimConfig | null) => void;
  code: string;
}) {
  const [jsonMode, setJsonMode] = useState(false);
  const [jsonText, setJsonText] = useState('');
  const [jsonError, setJsonError] = useState<string | null>(null);

  const currentConfig: PPrimConfig = config || { version: 1, mappings: [] };

  const updateMapping = (idx: number, m: PPrimMapping) => {
    const next = [...currentConfig.mappings];
    next[idx] = m;
    onConfigChange({ ...currentConfig, mappings: next });
  };

  const deleteMapping = (idx: number) => {
    onConfigChange({ ...currentConfig, mappings: currentConfig.mappings.filter((_, i) => i !== idx) });
  };

  const addMapping = () => {
    onConfigChange({ ...currentConfig, mappings: [...currentConfig.mappings, emptyMapping()] });
  };

  const handleScanCode = () => {
    const found = scanCodeForSamples(code);
    if (found.length === 0) return;
    // Merge: keep existing mappings by parameter name, add new stubs
    const existing = new Set(currentConfig.mappings.map(m => m.parameter));
    const newMappings = found.filter(m => !existing.has(m.parameter));
    if (newMappings.length > 0) {
      onConfigChange({ ...currentConfig, mappings: [...currentConfig.mappings, ...newMappings] });
    }
  };

  const handlePasteJSON = useCallback(() => {
    setJsonMode(true);
    setJsonText(JSON.stringify(currentConfig, null, 2));
    setJsonError(null);
  }, [currentConfig]);

  const handleApplyJSON = () => {
    try {
      const parsed = JSON.parse(jsonText);
      // Accept either a full config or just an array of mappings
      if (Array.isArray(parsed)) {
        onConfigChange({ version: 1, mappings: parsed });
      } else if (parsed.mappings) {
        onConfigChange(parsed);
      } else {
        setJsonError('Expected { version, mappings: [...] } or an array of mappings');
        return;
      }
      setJsonMode(false);
      setJsonError(null);
    } catch (e: any) {
      setJsonError(e.message);
    }
  };

  if (jsonMode) {
    return (
      <div style={{ flex: 1, display: 'flex', flexDirection: 'column', overflow: 'hidden' }}>
        <div style={{ padding: '6px 10px', display: 'flex', gap: '6px', alignItems: 'center', flexShrink: 0 }}>
          <button style={{ ...smallBtnStyle, background: '#2a4a2a', color: '#9ece6a' }} onClick={handleApplyJSON}>
            Apply JSON
          </button>
          <button style={smallBtnStyle} onClick={() => { setJsonMode(false); setJsonError(null); }}>
            Cancel
          </button>
          {jsonError && <span style={{ fontSize: '10px', color: '#f7768e' }}>{jsonError}</span>}
        </div>
        <textarea
          style={{
            flex: 1,
            background: '#16161e',
            color: '#c0caf5',
            border: 'none',
            padding: '10px',
            fontFamily: 'monospace',
            fontSize: '12px',
            resize: 'none',
            outline: 'none',
          }}
          value={jsonText}
          onChange={e => setJsonText(e.target.value)}
        />
      </div>
    );
  }

  return (
    <div style={{ flex: 1, overflow: 'auto', padding: '8px 10px' }}>
      {currentConfig.mappings.length === 0 ? (
        <div style={{ color: '#565f89', fontSize: '12px', textAlign: 'center', padding: '20px 10px' }}>
          No p-prim mappings configured. Use "Scan Code" to detect parameters from the Pyro model,
          "Paste JSON" to paste an LLM-generated config, or "Add Parameter" to add one manually.
        </div>
      ) : (
        currentConfig.mappings.map((m, i) => (
          <MappingCard
            key={`${m.parameter}-${i}`}
            mapping={m}
            onChange={nm => updateMapping(i, nm)}
            onDelete={() => deleteMapping(i)}
          />
        ))
      )}
      <div style={{ display: 'flex', gap: '6px', marginTop: '8px' }}>
        <button style={smallBtnStyle} onClick={addMapping}>+ Add Parameter</button>
        <button style={smallBtnStyle} onClick={handleScanCode}>Scan Code</button>
        <button style={smallBtnStyle} onClick={handlePasteJSON}>Paste JSON</button>
      </div>
    </div>
  );
}

export default function PyroCodePanel({ code, onChange, pprimConfig, onPPrimConfigChange, onInfer, loading }: Props) {
  const [tab, setTab] = useState<'code' | 'pprims'>('code');

  return (
    <div style={panelStyle}>
      <div style={headerStyle}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
          <span>Pyro Model</span>
          <div style={{ display: 'flex', gap: '4px' }}>
            <button style={tabBtnStyle(tab === 'code')} onClick={() => setTab('code')}>Code</button>
            <button style={tabBtnStyle(tab === 'pprims')} onClick={() => setTab('pprims')}>P-Prims</button>
          </div>
        </div>
        <button
          onClick={onInfer}
          disabled={loading}
          style={{
            ...buttonStyle,
            opacity: loading ? 0.6 : 1,
            cursor: loading ? 'wait' : 'pointer',
          }}
        >
          {loading ? 'Running Inference...' : 'Run Inference'}
        </button>
      </div>
      {tab === 'code' ? (
        <div style={{ flex: 1, overflow: 'hidden' }}>
          <Suspense fallback={
            <textarea
              value={code}
              onChange={e => onChange(e.target.value)}
              style={{
                width: '100%', height: '100%', background: '#1a1b26', color: '#a9b1d6',
                border: 'none', padding: '12px', fontFamily: 'monospace', fontSize: '13px',
                resize: 'none',
              }}
            />
          }>
            <MonacoEditor
              height="100%"
              language="python"
              theme="vs-dark"
              value={code}
              onChange={(val) => onChange(val || '')}
              options={{
                minimap: { enabled: false },
                fontSize: 13,
                lineNumbers: 'on',
                scrollBeyondLastLine: false,
                wordWrap: 'on',
                automaticLayout: true,
              }}
            />
          </Suspense>
        </div>
      ) : (
        <PPrimEditor config={pprimConfig} onConfigChange={onPPrimConfigChange} code={code} />
      )}
    </div>
  );
}
