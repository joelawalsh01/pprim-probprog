import React, { useState, useRef, useEffect } from 'react';
import type { SimulationResult, InferenceResult, Project } from '../types';
import AnimationPanel from './AnimationPanel';
import MujocoCodePanel from './MujocoCodePanel';
import PyroCodePanel from './PyroCodePanel';
import PriorPosteriorPanel from './PriorPosteriorPanel';
import TrajectoryPanel from './TrajectoryPanel';
import SummaryPanel from './SummaryPanel';

interface Props {
  projects: Project[];
  activeProjectId: string;
  onSwitchProject: (id: string) => void;
  onAddProject: () => void;
  onCloseProject: (id: string) => void;
  onRenameProject: (id: string, name: string) => void;
  mjcfXml: string;
  onMjcfChange: (xml: string) => void;
  pyroCode: string;
  onPyroCodeChange: (code: string) => void;
  simResult: SimulationResult | null;
  inferResult: InferenceResult | null;
  onSimulate: () => void;
  onInfer: () => void;
  simLoading: boolean;
  inferLoading: boolean;
  error: string | null;
}

const styles: Record<string, React.CSSProperties> = {
  container: {
    display: 'flex',
    flexDirection: 'column',
    height: '100vh',
    overflow: 'hidden',
  },
  header: {
    padding: '12px 24px',
    background: '#1a1b26',
    borderBottom: '1px solid #2a2b3d',
    display: 'flex',
    alignItems: 'center',
    gap: '16px',
    flexShrink: 0,
  },
  title: {
    fontSize: '18px',
    fontWeight: 700,
    color: '#c0caf5',
    margin: 0,
  },
  subtitle: {
    fontSize: '12px',
    color: '#565f89',
    margin: 0,
  },
  tabBar: {
    display: 'flex',
    alignItems: 'center',
    gap: '2px',
    padding: '0 24px',
    background: '#16161e',
    borderBottom: '1px solid #2a2b3d',
    flexShrink: 0,
    minHeight: '34px',
  },
  errorBanner: {
    padding: '8px 24px',
    background: '#3b2030',
    color: '#f7768e',
    fontSize: '13px',
    borderBottom: '1px solid #52293e',
  },
  grid: {
    display: 'grid',
    gridTemplateColumns: '1fr 1fr 1fr',
    gridTemplateRows: '1fr 1fr',
    gap: '1px',
    flex: 1,
    background: '#2a2b3d',
    overflow: 'hidden',
  },
};

function ProjectTab({
  project,
  active,
  canClose,
  onSwitch,
  onClose,
  onRename,
}: {
  project: Project;
  active: boolean;
  canClose: boolean;
  onSwitch: () => void;
  onClose: () => void;
  onRename: (name: string) => void;
}) {
  const [editing, setEditing] = useState(false);
  const [editName, setEditName] = useState(project.name);
  const inputRef = useRef<HTMLInputElement>(null);

  useEffect(() => {
    if (editing && inputRef.current) {
      inputRef.current.focus();
      inputRef.current.select();
    }
  }, [editing]);

  function commitRename() {
    const trimmed = editName.trim();
    if (trimmed && trimmed !== project.name) {
      onRename(trimmed);
    }
    setEditing(false);
  }

  return (
    <div
      onClick={onSwitch}
      onDoubleClick={() => {
        setEditName(project.name);
        setEditing(true);
      }}
      style={{
        display: 'flex',
        alignItems: 'center',
        gap: '6px',
        padding: '6px 12px',
        fontSize: '12px',
        fontWeight: active ? 600 : 400,
        color: active ? '#c0caf5' : '#565f89',
        background: active ? '#1a1b26' : 'transparent',
        borderTop: active ? '2px solid #7aa2f7' : '2px solid transparent',
        borderLeft: '1px solid transparent',
        borderRight: '1px solid transparent',
        borderBottom: active ? '1px solid #1a1b26' : '1px solid transparent',
        marginBottom: '-1px',
        cursor: 'pointer',
        userSelect: 'none',
        whiteSpace: 'nowrap',
      }}
    >
      {editing ? (
        <input
          ref={inputRef}
          value={editName}
          onChange={e => setEditName(e.target.value)}
          onBlur={commitRename}
          onKeyDown={e => {
            if (e.key === 'Enter') commitRename();
            if (e.key === 'Escape') setEditing(false);
          }}
          onClick={e => e.stopPropagation()}
          style={{
            background: '#2a2b3d',
            border: '1px solid #3b3d57',
            borderRadius: '2px',
            color: '#c0caf5',
            fontSize: '12px',
            padding: '1px 4px',
            width: '100px',
            outline: 'none',
          }}
        />
      ) : (
        <span>{project.name}</span>
      )}
      {canClose && (
        <span
          onClick={e => {
            e.stopPropagation();
            onClose();
          }}
          style={{
            fontSize: '14px',
            lineHeight: '1',
            color: active ? '#565f89' : '#3b3d57',
            cursor: 'pointer',
            padding: '0 2px',
            borderRadius: '2px',
          }}
          onMouseEnter={e => (e.currentTarget.style.color = '#f7768e')}
          onMouseLeave={e => (e.currentTarget.style.color = active ? '#565f89' : '#3b3d57')}
        >
          ×
        </span>
      )}
    </div>
  );
}

export default function Dashboard(props: Props) {
  const canClose = props.projects.length > 1;

  return (
    <div style={styles.container}>
      <div style={styles.header}>
        <div>
          <h1 style={styles.title}>DiSessa Balls</h1>
          <p style={styles.subtitle}>Analysis-by-Synthesis Physics Prior Extraction</p>
        </div>
      </div>

      <div style={styles.tabBar}>
        {props.projects.map(project => (
          <ProjectTab
            key={project.id}
            project={project}
            active={project.id === props.activeProjectId}
            canClose={canClose}
            onSwitch={() => props.onSwitchProject(project.id)}
            onClose={() => props.onCloseProject(project.id)}
            onRename={(name) => props.onRenameProject(project.id, name)}
          />
        ))}
        <button
          onClick={props.onAddProject}
          style={{
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            width: '24px',
            height: '24px',
            marginLeft: '4px',
            fontSize: '16px',
            color: '#565f89',
            background: 'transparent',
            border: '1px solid transparent',
            borderRadius: '3px',
            cursor: 'pointer',
            flexShrink: 0,
          }}
          onMouseEnter={e => {
            e.currentTarget.style.background = '#2a2b3d';
            e.currentTarget.style.color = '#c0caf5';
          }}
          onMouseLeave={e => {
            e.currentTarget.style.background = 'transparent';
            e.currentTarget.style.color = '#565f89';
          }}
          title="New project"
        >
          +
        </button>
      </div>

      {props.error && <div style={styles.errorBanner}>{props.error}</div>}

      <div style={styles.grid}>
        <AnimationPanel
          frames={props.simResult?.frames}
          trajectory={props.simResult?.trajectory}
          naiveTrajectory={props.simResult?.naive_trajectory}
          loading={props.simLoading}
        />

        <MujocoCodePanel
          xml={props.mjcfXml}
          onChange={props.onMjcfChange}
          onSimulate={props.onSimulate}
          loading={props.simLoading}
        />

        <PyroCodePanel
          code={props.pyroCode}
          onChange={props.onPyroCodeChange}
          onInfer={props.onInfer}
          loading={props.inferLoading}
        />

        <PriorPosteriorPanel
          posterior={props.inferResult?.posterior}
        />

        <TrajectoryPanel
          simTrajectory={props.simResult?.trajectory}
          naiveTrajectory={props.simResult?.naive_trajectory || props.inferResult?.observed_trajectory}
          newtonianTrajectory={props.inferResult?.newtonian_trajectory}
        />

        <SummaryPanel
          inferResult={props.inferResult}
          loading={props.inferLoading}
        />
      </div>
    </div>
  );
}
