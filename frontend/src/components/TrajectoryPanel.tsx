import React, { useRef, useEffect } from 'react';

interface Point {
  t?: number;
  x: number;
  z: number;
}

interface Props {
  simTrajectory?: Point[];
  naiveTrajectory?: Point[];
  newtonianTrajectory?: Point[];
  animationProgress: number; // 0–1 for progressive draw, -1 means no animation yet
  onClear: () => void;
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
  color: '#9ece6a',
  textTransform: 'uppercase',
  letterSpacing: '0.05em',
  borderBottom: '1px solid #2a2b3d',
  flexShrink: 0,
};

export default function TrajectoryPanel({ simTrajectory, naiveTrajectory, newtonianTrajectory, animationProgress, onClear }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#1a1b26';
    ctx.fillRect(0, 0, w, h);

    const all: Point[] = [
      ...(simTrajectory || []),
      ...(naiveTrajectory || []),
      ...(newtonianTrajectory || []),
    ];

    // Blank until animation has started
    if (all.length === 0 || animationProgress < 0) {
      ctx.fillStyle = '#565f89';
      ctx.font = '13px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(
        all.length === 0
          ? 'Simulate and run inference to compare trajectories'
          : 'Play the animation to see trajectories',
        w / 2, h / 2,
      );
      return;
    }

    const xs = all.map(p => p.x);
    const zs = all.map(p => p.z);
    const xMin = Math.min(...xs) - 0.3;
    const xMax = Math.max(...xs) + 0.3;
    const zMin = Math.min(...zs) - 0.3;
    const zMax = Math.max(...zs) + 0.3;

    const margin = 40;
    const toX = (x: number) => margin + ((x - xMin) / (xMax - xMin)) * (w - 2 * margin);
    const toZ = (z: number) => h - margin - ((z - zMin) / (zMax - zMin)) * (h - 2 * margin);

    // Draw grid
    ctx.strokeStyle = '#2a2b3d';
    ctx.lineWidth = 0.5;
    for (let i = 0; i <= 5; i++) {
      const gx = margin + (i / 5) * (w - 2 * margin);
      ctx.beginPath(); ctx.moveTo(gx, margin); ctx.lineTo(gx, h - margin); ctx.stroke();
      const gz = margin + (i / 5) * (h - 2 * margin);
      ctx.beginPath(); ctx.moveTo(margin, gz); ctx.lineTo(w - margin, gz); ctx.stroke();
    }

    // Axes labels
    ctx.fillStyle = '#565f89';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('x position', w / 2, h - 8);
    ctx.save();
    ctx.translate(12, h / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText('z position', 0, 0);
    ctx.restore();

    // Slice trajectory to current progress
    function sliceToProgress(points: Point[]): Point[] {
      if (animationProgress >= 1) return points;
      const endIdx = Math.max(1, Math.ceil(animationProgress * points.length));
      return points.slice(0, endIdx);
    }

    function drawTraj(points: Point[], color: string, lineWidth: number, dash: number[] = []) {
      const visible = sliceToProgress(points);
      if (visible.length === 0) return;
      ctx!.beginPath();
      ctx!.strokeStyle = color;
      ctx!.lineWidth = lineWidth;
      ctx!.setLineDash(dash);
      visible.forEach((p, i) => {
        const cx = toX(p.x);
        const cz = toZ(p.z);
        if (i === 0) ctx!.moveTo(cx, cz);
        else ctx!.lineTo(cx, cz);
      });
      ctx!.stroke();
      ctx!.setLineDash([]);

      // Draw ball at end of visible trail
      const tip = visible[visible.length - 1];
      ctx!.beginPath();
      ctx!.arc(toX(tip.x), toZ(tip.z), 5, 0, Math.PI * 2);
      ctx!.fillStyle = color;
      ctx!.fill();
    }

    // Draw in order: sim, newtonian, naive
    if (simTrajectory) drawTraj(simTrajectory, '#7aa2f7', 2.5);
    if (newtonianTrajectory) drawTraj(newtonianTrajectory, '#9ece6a', 2, [4, 2]);
    if (naiveTrajectory) drawTraj(naiveTrajectory, '#f7768e', 2, [6, 4]);

    // Legend
    const legendY = 16;
    let legendX = margin;
    const items = [
      { label: 'MuJoCo sim', color: '#7aa2f7', has: !!simTrajectory },
      { label: 'Newtonian', color: '#9ece6a', has: !!newtonianTrajectory },
      { label: 'Alternative', color: '#f7768e', has: !!naiveTrajectory },
    ];
    ctx.font = '11px sans-serif';
    for (const item of items) {
      if (!item.has) continue;
      ctx.fillStyle = item.color;
      ctx.fillRect(legendX, legendY - 6, 12, 3);
      ctx.fillText(item.label, legendX + 16, legendY);
      legendX += ctx.measureText(item.label).width + 30;
    }
  }, [simTrajectory, naiveTrajectory, newtonianTrajectory, animationProgress]);

  return (
    <div style={panelStyle}>
      <div style={{ ...headerStyle, display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
        <span>Trajectory Comparison</span>
        {(simTrajectory || naiveTrajectory || newtonianTrajectory) && (
          <button
            onClick={() => { if (window.confirm('Clear all trajectory data?')) onClear(); }}
            style={{
              padding: '2px 8px',
              fontSize: '11px',
              background: 'transparent',
              color: '#565f89',
              border: '1px solid #2a2b3d',
              borderRadius: '3px',
              cursor: 'pointer',
            }}
            title="Clear trajectory data"
          >
            Clear
          </button>
        )}
      </div>
      <div style={{ flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center', padding: '8px' }}>
        <canvas
          ref={canvasRef}
          width={600}
          height={400}
          style={{ maxWidth: '100%', maxHeight: '100%', borderRadius: '4px' }}
        />
      </div>
    </div>
  );
}
