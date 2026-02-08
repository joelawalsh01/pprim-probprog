import React, { useState, useEffect, useRef, useCallback } from 'react';
import type { TrajectoryPoint } from '../types';

interface Props {
  frames?: string[];
  trajectory?: TrajectoryPoint[];
  naiveTrajectory?: TrajectoryPoint[];
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
  color: '#7aa2f7',
  textTransform: 'uppercase',
  letterSpacing: '0.05em',
  borderBottom: '1px solid #2a2b3d',
  display: 'flex',
  justifyContent: 'space-between',
  alignItems: 'center',
  flexShrink: 0,
};

export default function AnimationPanel({ frames, trajectory, naiveTrajectory, loading }: Props) {
  const [frameIdx, setFrameIdx] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [showNaive, setShowNaive] = useState(true);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const intervalRef = useRef<number | null>(null);

  // Draw current frame + trajectory overlay
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    if (frames && frames.length > 0 && frameIdx < frames.length) {
      const img = new Image();
      img.onload = () => {
        ctx.drawImage(img, 0, 0, w, h);
        drawTrajectoryOverlay(ctx, w, h);
      };
      img.src = `data:image/png;base64,${frames[frameIdx]}`;
    } else {
      // No frames — draw trajectory visualization
      ctx.fillStyle = '#1a1b26';
      ctx.fillRect(0, 0, w, h);
      drawTrajectoryOverlay(ctx, w, h);

      if (!trajectory && !loading) {
        ctx.fillStyle = '#565f89';
        ctx.font = '14px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('Click "Simulate" to run', w / 2, h / 2);
      }
      if (loading) {
        ctx.fillStyle = '#7aa2f7';
        ctx.font = '14px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('Simulating...', w / 2, h / 2);
      }
    }
  }, [frames, frameIdx, trajectory, naiveTrajectory, showNaive, loading]);

  function drawTrajectoryOverlay(ctx: CanvasRenderingContext2D, w: number, h: number) {
    if (!trajectory || trajectory.length === 0) return;

    // Compute bounds for mapping
    const allPoints = [...trajectory, ...(showNaive && naiveTrajectory ? naiveTrajectory : [])];
    const xs = allPoints.map(p => p.x);
    const zs = allPoints.map(p => p.z);
    const xMin = Math.min(...xs) - 0.5;
    const xMax = Math.max(...xs) + 0.5;
    const zMin = Math.min(...zs) - 0.5;
    const zMax = Math.max(...zs) + 0.5;

    const margin = 40;
    const toCanvasX = (x: number) => margin + ((x - xMin) / (xMax - xMin)) * (w - 2 * margin);
    const toCanvasZ = (z: number) => h - margin - ((z - zMin) / (zMax - zMin)) * (h - 2 * margin);

    // Draw Newtonian trajectory (solid blue)
    ctx.beginPath();
    ctx.strokeStyle = '#7aa2f7';
    ctx.lineWidth = 2.5;
    trajectory.forEach((p, i) => {
      const cx = toCanvasX(p.x);
      const cy = toCanvasZ(p.z);
      if (i === 0) ctx.moveTo(cx, cy);
      else ctx.lineTo(cx, cy);
    });
    ctx.stroke();

    // Draw ball at current animation position
    if (frames && frames.length > 0) {
      const progress = frameIdx / (frames.length - 1);
      const idx = Math.floor(progress * (trajectory.length - 1));
      const p = trajectory[idx];
      ctx.beginPath();
      ctx.arc(toCanvasX(p.x), toCanvasZ(p.z), 6, 0, Math.PI * 2);
      ctx.fillStyle = '#7aa2f7';
      ctx.fill();
    }

    // Draw naive trajectory (dotted red)
    if (showNaive && naiveTrajectory && naiveTrajectory.length > 0) {
      ctx.beginPath();
      ctx.strokeStyle = '#f7768e';
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 4]);
      naiveTrajectory.forEach((p, i) => {
        const cx = toCanvasX(p.x);
        const cy = toCanvasZ(p.z);
        if (i === 0) ctx.moveTo(cx, cy);
        else ctx.lineTo(cx, cy);
      });
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Legend
    ctx.font = '11px sans-serif';
    ctx.fillStyle = '#7aa2f7';
    ctx.fillText('Newtonian', margin, 20);
    if (showNaive) {
      ctx.fillStyle = '#f7768e';
      ctx.fillText('Alternative', margin + 80, 20);
    }
  }

  // Playback controls
  const play = useCallback(() => {
    if (!frames || frames.length === 0) return;
    setPlaying(true);
    setFrameIdx(0);
    intervalRef.current = window.setInterval(() => {
      setFrameIdx(prev => {
        if (prev >= (frames?.length ?? 1) - 1) {
          if (intervalRef.current) window.clearInterval(intervalRef.current);
          setPlaying(false);
          return prev;
        }
        return prev + 1;
      });
    }, 50);
  }, [frames]);

  useEffect(() => {
    return () => {
      if (intervalRef.current) window.clearInterval(intervalRef.current);
    };
  }, []);

  return (
    <div style={panelStyle}>
      <div style={headerStyle}>
        <span>Animation</span>
        <div style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
          <label style={{ fontSize: '11px', color: '#a9b1d6', cursor: 'pointer' }}>
            <input
              type="checkbox"
              checked={showNaive}
              onChange={e => setShowNaive(e.target.checked)}
              style={{ marginRight: '4px' }}
            />
            Show alternative
          </label>
          {frames && frames.length > 0 && (
            <button
              onClick={play}
              disabled={playing}
              style={{
                padding: '2px 10px',
                fontSize: '11px',
                background: playing ? '#2a2b3d' : '#364a82',
                color: '#c0caf5',
                border: 'none',
                borderRadius: '3px',
                cursor: playing ? 'default' : 'pointer',
              }}
            >
              {playing ? 'Playing...' : 'Play'}
            </button>
          )}
        </div>
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
