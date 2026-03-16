import React, { useState, useEffect, useRef, useCallback } from 'react';
import type { TrajectoryPoint } from '../types';

interface Props {
  frames?: string[];
  trajectory?: TrajectoryPoint[];
  naiveTrajectory?: TrajectoryPoint[];
  loading: boolean;
  onClear: () => void;
  onAnimationProgress?: (progress: number) => void; // 0–1, -1 means no animation yet
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

function getVideoMimeType(): string | null {
  const candidates = [
    'video/webm;codecs=vp9',
    'video/webm;codecs=vp8',
    'video/webm',
    'video/mp4',
  ];
  for (const mime of candidates) {
    if (MediaRecorder.isTypeSupported(mime)) return mime;
  }
  return null;
}


export default function AnimationPanel({ frames, trajectory, naiveTrajectory, loading, onClear, onAnimationProgress }: Props) {
  const [frameIdx, setFrameIdx] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [showNaive, setShowNaive] = useState(true);
  const [exporting, setExporting] = useState(false);
  const [exportProgress, setExportProgress] = useState(0);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const intervalRef = useRef<number | null>(null);
  const frameImagesRef = useRef<HTMLImageElement[]>([]);
  const [frameImagesLoaded, setFrameImagesLoaded] = useState(false);

  // Pre-load frame images from base64 PNGs
  useEffect(() => {
    if (!frames || frames.length === 0) {
      frameImagesRef.current = [];
      setFrameImagesLoaded(false);
      return;
    }
    let cancelled = false;
    const images: HTMLImageElement[] = [];
    let loaded = 0;
    frames.forEach((b64, i) => {
      const img = new Image();
      img.onload = () => {
        if (cancelled) return;
        images[i] = img;
        loaded++;
        if (loaded === frames.length) {
          frameImagesRef.current = images;
          setFrameImagesLoaded(true);
        }
      };
      img.onerror = () => {
        if (cancelled) return;
        loaded++;
        if (loaded === frames.length) {
          frameImagesRef.current = images;
          setFrameImagesLoaded(true);
        }
      };
      img.src = `data:image/png;base64,${b64}`;
    });
    return () => { cancelled = true; };
  }, [frames]);

  // Draw current frame — MuJoCo rendered frame as background + trajectory overlay
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const w = canvas.width;
    const h = canvas.height;

    ctx.fillStyle = '#1a1b26';
    ctx.fillRect(0, 0, w, h);

    if (frames && frames.length > 0) {
      // Canvas draws all trajectory visualization — skip MuJoCo frame backgrounds
      // to avoid a duplicate rendered ball appearing alongside the canvas balls.
      drawTrajectoryOverlay(ctx, w, h, frameIdx, frames.length, true);
    } else if (trajectory && trajectory.length > 0) {
      // No rendered frames but we have trajectory data — show final state
      drawTrajectoryOverlay(ctx, w, h, trajectory.length - 1, trajectory.length, false);
    } else if (loading) {
      ctx.fillStyle = '#7aa2f7';
      ctx.font = '14px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Simulating...', w / 2, h / 2);
    } else {
      ctx.fillStyle = '#565f89';
      ctx.font = '14px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('Click "Simulate" to run', w / 2, h / 2);
    }
  }, [frames, frameIdx, frameImagesLoaded, trajectory, naiveTrajectory, showNaive, loading]);

  // Report animation progress to parent
  useEffect(() => {
    if (!onAnimationProgress) return;
    if (!frames || frames.length === 0) {
      onAnimationProgress(-1);
    } else if (playing || exporting) {
      onAnimationProgress(frameIdx / (frames.length - 1));
    } else if (frameIdx >= frames.length - 1) {
      // Finished playing — show full trajectory
      onAnimationProgress(1);
    }
  }, [frameIdx, frames, playing, exporting, onAnimationProgress]);

  // Compute coordinate mapping from trajectory bounds to canvas pixels.
  // Uses all trajectories (Newtonian + alternative) for consistent bounds.
  function getTrajectoryMapping(w: number, h: number) {
    if (!trajectory || trajectory.length === 0) return null;
    const allPoints = [...trajectory, ...(showNaive && naiveTrajectory ? naiveTrajectory : [])];
    const xs = allPoints.map(p => p.x);
    const zs = allPoints.map(p => p.z);
    const xMin = Math.min(...xs) - 0.5;
    const xMax = Math.max(...xs) + 0.5;
    // Extra space below for the incoming "shove" ball
    const zRange = Math.max(...zs) - Math.min(...zs);
    const zMin = Math.min(...zs) - Math.max(0.5, zRange * 0.4);
    const zMax = Math.max(...zs) + 0.5;
    const margin = 40;
    return {
      toCanvasX: (x: number) => margin + ((x - xMin) / (xMax - xMin)) * (w - 2 * margin),
      toCanvasZ: (z: number) => h - margin - ((z - zMin) / (zMax - zMin)) * (h - 2 * margin),
      margin,
    };
  }

  // Draw both balls with ghost trails, diSessa-diagram style.
  // Both start at same point, travel together, then diverge at the force point.
  function drawTrajectoryOverlay(
    ctx: CanvasRenderingContext2D, w: number, h: number,
    currentFrame?: number, totalFrames?: number,
    progressive?: boolean,
  ) {
    if (!trajectory || trajectory.length === 0) return;

    const fIdx = currentFrame ?? frameIdx;
    const fTotal = totalFrames ?? (frames?.length ?? 0);
    const mapping = getTrajectoryMapping(w, h);
    if (!mapping) return;
    const { toCanvasX, toCanvasZ, margin } = mapping;

    const progress = fTotal > 1 ? fIdx / (fTotal - 1) : 1;
    const ballRadius = 12;
    const ghostRadius = 8;
    const ghostCount = 8;

    // Draw tube/channel if the trajectory starts with a curved arc phase.
    // Must curve from the very start (z changes in first 10%), not just straight-then-diagonal.
    const earlyEnd = Math.floor(trajectory.length * 0.1);
    const arcEnd = Math.floor(trajectory.length * 0.4);
    if (arcEnd > 5 && earlyEnd >= 3) {
      const startPt = trajectory[0];
      const earlyZ = Math.abs(trajectory[earlyEnd].z - startPt.z);
      const midPt = trajectory[arcEnd];
      const xTravel = Math.abs(midPt.x - startPt.x);
      const zTravel = Math.abs(midPt.z - startPt.z);
      const isArc = earlyZ > 0.1 && xTravel > 0.3 && zTravel > 0.3;
      if (isArc) {
        // Draw the tube as two parallel paths offset from the trajectory
        const tubeWidth = 14; // pixels, half-width of the tube
        ctx.save();

        // Compute normals at each point and draw inner/outer walls
        const arcPoints = trajectory.slice(0, arcEnd + 1);
        const outerPath: [number, number][] = [];
        const innerPath: [number, number][] = [];

        for (let i = 0; i < arcPoints.length; i++) {
          const cx = toCanvasX(arcPoints[i].x);
          const cz = toCanvasZ(arcPoints[i].z);

          // Compute normal direction from adjacent points
          let dx: number, dz: number;
          if (i === 0) {
            dx = toCanvasX(arcPoints[1].x) - cx;
            dz = toCanvasZ(arcPoints[1].z) - cz;
          } else if (i === arcPoints.length - 1) {
            dx = cx - toCanvasX(arcPoints[i - 1].x);
            dz = cz - toCanvasZ(arcPoints[i - 1].z);
          } else {
            dx = toCanvasX(arcPoints[i + 1].x) - toCanvasX(arcPoints[i - 1].x);
            dz = toCanvasZ(arcPoints[i + 1].z) - toCanvasZ(arcPoints[i - 1].z);
          }
          const len = Math.sqrt(dx * dx + dz * dz) || 1;
          const nx = -dz / len; // normal perpendicular to path
          const nz = dx / len;

          outerPath.push([cx + nx * tubeWidth, cz + nz * tubeWidth]);
          innerPath.push([cx - nx * tubeWidth, cz - nz * tubeWidth]);
        }

        // Draw filled tube channel
        ctx.beginPath();
        ctx.moveTo(outerPath[0][0], outerPath[0][1]);
        for (const [x, z] of outerPath) ctx.lineTo(x, z);
        // Connect to inner path in reverse
        for (let i = innerPath.length - 1; i >= 0; i--) {
          ctx.lineTo(innerPath[i][0], innerPath[i][1]);
        }
        ctx.closePath();
        ctx.fillStyle = 'rgba(60, 70, 90, 0.4)';
        ctx.fill();

        // Draw tube walls
        ctx.beginPath();
        ctx.moveTo(outerPath[0][0], outerPath[0][1]);
        for (const [x, z] of outerPath) ctx.lineTo(x, z);
        ctx.strokeStyle = 'rgba(120, 140, 180, 0.6)';
        ctx.lineWidth = 2.5;
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(innerPath[0][0], innerPath[0][1]);
        for (const [x, z] of innerPath) ctx.lineTo(x, z);
        ctx.strokeStyle = 'rgba(120, 140, 180, 0.6)';
        ctx.lineWidth = 2.5;
        ctx.stroke();

        // Draw exit opening marker
        const exitOuter = outerPath[outerPath.length - 1];
        const exitInner = innerPath[innerPath.length - 1];
        ctx.beginPath();
        ctx.moveTo(exitOuter[0], exitOuter[1]);
        ctx.lineTo(exitOuter[0] + 15, exitOuter[1]);
        ctx.strokeStyle = 'rgba(140, 200, 140, 0.5)';
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(exitInner[0], exitInner[1]);
        ctx.lineTo(exitInner[0] + 15, exitInner[1]);
        ctx.stroke();

        ctx.restore();
      }
    }

    // Draw a 3D-looking sphere with radial gradient, highlight, and shadow
    function drawSphere(
      cx: number, cy: number, r: number,
      baseColor: string, highlightColor: string, shadowColor: string,
      alpha: number = 1,
    ) {
      ctx.save();
      ctx.globalAlpha = alpha;

      // Drop shadow
      ctx.beginPath();
      ctx.arc(cx + 1, cy + 2, r, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(0,0,0,0.3)';
      ctx.fill();

      // Main sphere gradient — light source top-left
      const grad = ctx.createRadialGradient(
        cx - r * 0.3, cy - r * 0.3, r * 0.1,
        cx, cy, r,
      );
      grad.addColorStop(0, highlightColor);
      grad.addColorStop(0.6, baseColor);
      grad.addColorStop(1, shadowColor);

      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.fillStyle = grad;
      ctx.fill();

      // Specular highlight
      ctx.beginPath();
      ctx.arc(cx - r * 0.25, cy - r * 0.25, r * 0.3, 0, Math.PI * 2);
      const specGrad = ctx.createRadialGradient(
        cx - r * 0.25, cy - r * 0.25, 0,
        cx - r * 0.25, cy - r * 0.25, r * 0.3,
      );
      specGrad.addColorStop(0, 'rgba(255,255,255,0.6)');
      specGrad.addColorStop(1, 'rgba(255,255,255,0)');
      ctx.fillStyle = specGrad;
      ctx.fill();

      ctx.restore();
    }

    // Draw an outlined ghost sphere (for alternative trail)
    function drawGhostSphere(
      cx: number, cy: number, r: number,
      baseColor: string, highlightColor: string,
      alpha: number = 0.35,
    ) {
      ctx.save();
      ctx.globalAlpha = alpha;

      const grad = ctx.createRadialGradient(
        cx - r * 0.3, cy - r * 0.3, r * 0.1,
        cx, cy, r,
      );
      grad.addColorStop(0, highlightColor);
      grad.addColorStop(1, baseColor);

      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, Math.PI * 2);
      ctx.fillStyle = grad;
      ctx.fill();

      ctx.restore();
    }

    // Helper: get trajectory index for current progress
    function endIdx(pts: TrajectoryPoint[]) {
      return progressive
        ? Math.min(Math.floor(progress * (pts.length - 1)), pts.length - 1)
        : pts.length - 1;
    }

    const newtonEnd = endIdx(trajectory);

    // Newtonian ghost trail + current ball
    {
      const step = Math.max(1, Math.floor(newtonEnd / ghostCount));
      for (let i = step; i < newtonEnd; i += step) {
        const px = toCanvasX(trajectory[i].x);
        const pz = toCanvasZ(trajectory[i].z);
        const fade = 0.15 + 0.2 * (i / newtonEnd);
        drawGhostSphere(px, pz, ghostRadius, '#4a6aaf', '#8ab4ff', fade);
      }
      const p = trajectory[newtonEnd];
      drawSphere(
        toCanvasX(p.x), toCanvasZ(p.z), ballRadius,
        '#5b8af7', '#b0d0ff', '#2a4a9a',
      );
    }

    // Alternative ghost trail + current ball
    if (showNaive && naiveTrajectory && naiveTrajectory.length > 0) {
      const naiveEnd = endIdx(naiveTrajectory);
      const step = Math.max(1, Math.floor(naiveEnd / ghostCount));
      for (let i = step; i < naiveEnd; i += step) {
        const px = toCanvasX(naiveTrajectory[i].x);
        const pz = toCanvasZ(naiveTrajectory[i].z);
        const fade = 0.15 + 0.2 * (i / naiveEnd);
        drawGhostSphere(px, pz, ghostRadius, '#af4a5a', '#ff8a9a', fade);
      }
      const p = naiveTrajectory[naiveEnd];
      drawSphere(
        toCanvasX(p.x), toCanvasZ(p.z), ballRadius,
        '#f7687e', '#ffb0c0', '#9a2a3a',
      );
    }

    // "Shove" ball — approaches from below and hits at the divergence point.
    // Only shown for DiSessa-style scenarios where there's a flat horizontal
    // phase followed by an upward impulse. Skip for scenarios where z changes
    // from the start (spiral tube, galileo drop, etc.).
    if (naiveTrajectory && naiveTrajectory.length > 1 && trajectory.length > 1) {
      // Find force time: the moment the alternative trajectory's z starts changing
      // (before force, z is constant; after force, z increases)
      const baseZ = naiveTrajectory[0].z;
      let forceTime = naiveTrajectory[naiveTrajectory.length - 1].t;
      let hasHorizontalPhase = false;
      for (let i = 1; i < naiveTrajectory.length; i++) {
        if (Math.abs(naiveTrajectory[i].z - baseZ) > 0.01) {
          forceTime = naiveTrajectory[Math.max(0, i - 1)].t;
          // Only count as having a horizontal phase if z was flat for a meaningful period
          hasHorizontalPhase = i > naiveTrajectory.length * 0.1;
          break;
        }
      }

      // Skip shove ball for scenarios without a clear horizontal→force transition
      if (!hasHorizontalPhase) {
        // Still draw the legend — skip the shove ball
      } else {

      // Find the corresponding index in the Newtonian trajectory
      let forceIdx = 0;
      for (let i = 0; i < trajectory.length; i++) {
        if (trajectory[i].t >= forceTime) {
          forceIdx = i;
          break;
        }
      }

      // Force progress: what fraction of the way to the hit point are we?
      const forceProgress = forceIdx > 0
        ? Math.min(1, (progress * (trajectory.length - 1)) / forceIdx)
        : 1;

      const hitPoint = trajectory[forceIdx];
      const hitX = toCanvasX(hitPoint.x);
      const hitZ = toCanvasZ(hitPoint.z);

      // Shove ball travels vertically from well below up to the hit point
      const startBelow = 120; // pixels below the hit point
      const shoveColor = '#e0af68';

      if (forceProgress < 1) {
        // Ball is approaching — draw it rising from below
        const shoveY = hitZ + startBelow * (1 - forceProgress);
        // Ghost trail for the shove ball
        const shoveGhosts = 4;
        for (let g = 0; g < shoveGhosts; g++) {
          const gProgress = (forceProgress * g) / shoveGhosts;
          const gy = hitZ + startBelow * (1 - gProgress);
          if (gy > hitZ) {
            const fade = 0.1 + 0.15 * (g / shoveGhosts);
            drawGhostSphere(hitX, gy, ghostRadius, '#a08030', '#f0d090', fade);
          }
        }
        // Current shove ball position
        drawSphere(hitX, shoveY, ballRadius, '#e0af68', '#fff0c0', '#8a6a20');
      } else {
        // After hit — ball stopped at the hit point, show "F" label
        drawSphere(hitX, hitZ + ballRadius * 2 + 4, ballRadius * 0.7, '#e0af68', '#fff0c0', '#8a6a20', 0.5);
        ctx.font = 'bold 12px sans-serif';
        ctx.fillStyle = '#e0af68';
        ctx.textAlign = 'center';
        ctx.fillText('F', hitX, hitZ + ballRadius * 2 + 8);
      }
      } // end hasHorizontalPhase else
    }

    // Legend
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'left';
    ctx.fillStyle = '#7aa2f7';
    ctx.fillText('Newtonian', margin, 20);
    if (showNaive && naiveTrajectory && naiveTrajectory.length > 0) {
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

  const exportVideo = useCallback(async () => {
    const canvas = canvasRef.current;
    if (!canvas || !frames || frames.length === 0) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const mimeType = getVideoMimeType();
    if (!mimeType) return;

    setExporting(true);
    setExportProgress(0);

    const stream = canvas.captureStream(0);
    const recorder = new MediaRecorder(stream, { mimeType });
    const chunks: Blob[] = [];

    recorder.ondataavailable = (e) => {
      if (e.data.size > 0) chunks.push(e.data);
    };

    const done = new Promise<Blob>((resolve) => {
      recorder.onstop = () => resolve(new Blob(chunks, { type: mimeType }));
    });

    recorder.start();

    const delay = (ms: number) => new Promise((r) => setTimeout(r, ms));
    const w = canvas.width;
    const h = canvas.height;

    for (let i = 0; i < frames.length; i++) {
      ctx.fillStyle = '#1a1b26';
      ctx.fillRect(0, 0, w, h);
      drawTrajectoryOverlay(ctx, w, h, i, frames.length, true);

      const track = stream.getVideoTracks()[0];
      if (track && 'requestFrame' in track) {
        (track as unknown as { requestFrame(): void }).requestFrame();
      }
      setExportProgress((i + 1) / frames.length);
      await delay(50);
    }

    recorder.stop();
    const blob = await done;

    const ext = mimeType.startsWith('video/mp4') ? 'mp4' : 'webm';
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `simulation.${ext}`;
    a.click();
    URL.revokeObjectURL(url);

    // Restore canvas to current displayed frame
    setExporting(false);
  }, [frames, frameIdx, trajectory, naiveTrajectory, showNaive]);

  // Export a PNG filmstrip (contact sheet) of key frames — readable by Claude for troubleshooting
  const exportFilmstrip = useCallback(() => {
    if (!trajectory || trajectory.length === 0) return;

    const numKeyFrames = 6;
    const frameW = 600;
    const frameH = 400;
    const cols = 3;
    const rows = 2;
    const labelH = 20;
    const totalW = frameW * cols;
    const totalH = (frameH + labelH) * rows;

    const offscreen = document.createElement('canvas');
    offscreen.width = totalW;
    offscreen.height = totalH;
    const ctx = offscreen.getContext('2d');
    if (!ctx) return;

    ctx.fillStyle = '#1a1b26';
    ctx.fillRect(0, 0, totalW, totalH);

    const totalFrameCount = frames?.length || trajectory.length;

    for (let f = 0; f < numKeyFrames; f++) {
      const col = f % cols;
      const row = Math.floor(f / cols);
      const ox = col * frameW;
      const oy = row * (frameH + labelH);

      // Frame index: evenly spaced across the animation
      const frameIndex = Math.floor((f / (numKeyFrames - 1)) * (totalFrameCount - 1));

      // Draw into a sub-region
      ctx.save();
      ctx.beginPath();
      ctx.rect(ox, oy + labelH, frameW, frameH);
      ctx.clip();
      ctx.translate(ox, oy + labelH);

      // Background
      ctx.fillStyle = '#1a1b26';
      ctx.fillRect(0, 0, frameW, frameH);

      // Trajectory overlay at this point in time
      drawTrajectoryOverlay(ctx, frameW, frameH, frameIndex, totalFrameCount, true);

      ctx.restore();

      // Frame label
      ctx.fillStyle = '#565f89';
      ctx.font = '12px sans-serif';
      ctx.textAlign = 'left';
      const progress = Math.round((frameIndex / (totalFrameCount - 1)) * 100);
      ctx.fillText(`Frame ${frameIndex}/${totalFrameCount - 1} (${progress}%)`, ox + 8, oy + 14);

      // Border
      ctx.strokeStyle = '#2a2b3d';
      ctx.lineWidth = 1;
      ctx.strokeRect(ox, oy, frameW, frameH + labelH);
    }

    // Save as PNG to Downloads
    offscreen.toBlob(blob => {
      if (!blob) return;
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = 'simulation-filmstrip.png';
      a.click();
      URL.revokeObjectURL(url);
    }, 'image/png');
  }, [frames, trajectory, naiveTrajectory, showNaive]);

  // Snapshot: export current canvas state as PNG
  const exportSnapshot = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    canvas.toBlob(blob => {
      if (!blob) return;
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = 'simulation-snapshot.png';
      a.click();
      URL.revokeObjectURL(url);
    }, 'image/png');
  }, []);

  const canExport = typeof MediaRecorder !== 'undefined' && getVideoMimeType() !== null;

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
              disabled={playing || exporting}
              style={{
                padding: '2px 10px',
                fontSize: '11px',
                background: playing || exporting ? '#2a2b3d' : '#364a82',
                color: '#c0caf5',
                border: 'none',
                borderRadius: '3px',
                cursor: playing || exporting ? 'default' : 'pointer',
              }}
            >
              {playing ? 'Playing...' : 'Play'}
            </button>
          )}
          {(frames || trajectory) && (
            <button
              onClick={exportFilmstrip}
              disabled={playing || exporting}
              style={{
                padding: '2px 10px',
                fontSize: '11px',
                background: '#364a82',
                color: '#c0caf5',
                border: 'none',
                borderRadius: '3px',
                cursor: playing || exporting ? 'default' : 'pointer',
              }}
              title="Export 6-frame filmstrip as PNG (for troubleshooting)"
            >
              Filmstrip
            </button>
          )}
          {(frames || trajectory) && (
            <button
              onClick={exportSnapshot}
              disabled={playing || exporting}
              style={{
                padding: '2px 10px',
                fontSize: '11px',
                background: '#364a82',
                color: '#c0caf5',
                border: 'none',
                borderRadius: '3px',
                cursor: playing || exporting ? 'default' : 'pointer',
              }}
              title="Export current frame as PNG"
            >
              Snapshot
            </button>
          )}
          {frames && frames.length > 0 && canExport && (
            <button
              onClick={exportVideo}
              disabled={playing || exporting}
              style={{
                padding: '2px 10px',
                fontSize: '11px',
                background: exporting ? '#2a2b3d' : '#364a82',
                color: '#c0caf5',
                border: 'none',
                borderRadius: '3px',
                cursor: playing || exporting ? 'default' : 'pointer',
              }}
              title="Export animation as video"
            >
              {exporting ? `${Math.round(exportProgress * 100)}%...` : 'Video'}
            </button>
          )}
          {(frames || trajectory) && (
            <button
              onClick={() => { if (window.confirm('Clear simulation results?')) onClear(); }}
              disabled={exporting}
              style={{
                padding: '2px 8px',
                fontSize: '11px',
                background: 'transparent',
                color: '#565f89',
                border: '1px solid #2a2b3d',
                borderRadius: '3px',
                cursor: exporting ? 'default' : 'pointer',
              }}
              title="Clear simulation results"
            >
              Clear
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
