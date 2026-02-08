import React from 'react';
import type { SimulationResult, InferenceResult } from '../types';
import AnimationPanel from './AnimationPanel';
import MujocoCodePanel from './MujocoCodePanel';
import PyroCodePanel from './PyroCodePanel';
import PriorPosteriorPanel from './PriorPosteriorPanel';
import TrajectoryPanel from './TrajectoryPanel';
import SummaryPanel from './SummaryPanel';

interface Props {
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

export default function Dashboard(props: Props) {
  return (
    <div style={styles.container}>
      <div style={styles.header}>
        <div>
          <h1 style={styles.title}>DiSessa Balls</h1>
          <p style={styles.subtitle}>Analysis-by-Synthesis Physics Prior Extraction</p>
        </div>
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
