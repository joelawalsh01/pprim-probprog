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

export default function TrajectoryPanel({ simTrajectory, naiveTrajectory, newtonianTrajectory }: Props) {
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

    if (all.length === 0) {
      ctx.fillStyle = '#565f89';
      ctx.font = '13px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Simulate and run inference to compare trajectories', w / 2, h / 2);
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

    function drawTraj(points: Point[], color: string, lineWidth: number, dash: number[] = []) {
      if (points.length === 0) return;
      ctx!.beginPath();
      ctx!.strokeStyle = color;
      ctx!.lineWidth = lineWidth;
      ctx!.setLineDash(dash);
      points.forEach((p, i) => {
        const cx = toX(p.x);
        const cz = toZ(p.z);
        if (i === 0) ctx!.moveTo(cx, cz);
        else ctx!.lineTo(cx, cz);
      });
      ctx!.stroke();
      ctx!.setLineDash([]);
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
  }, [simTrajectory, naiveTrajectory, newtonianTrajectory]);

  return (
    <div style={panelStyle}>
      <div style={headerStyle}>Trajectory Comparison</div>
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
