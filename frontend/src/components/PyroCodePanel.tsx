import React, { Suspense } from 'react';

const MonacoEditor = React.lazy(() => import('@monaco-editor/react'));

interface Props {
  code: string;
  onChange: (code: string) => void;
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

export default function PyroCodePanel({ code, onChange, onInfer, loading }: Props) {
  return (
    <div style={panelStyle}>
      <div style={headerStyle}>
        <span>Pyro Model</span>
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
    </div>
  );
}
