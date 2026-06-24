import { computeTrebuchetGeometry, type TrebuchetGeometry } from './geometry';
import { findSampleAtTime, type SimulationResult, type SimulationSample, type TrebuchetParams } from './physics';
import type { GeometryFunction } from './designs/types';

interface Viewport {
  scale: number;
  originX: number;
  originY: number;
}

export class TrebuchetRenderer {
  private readonly ctx: CanvasRenderingContext2D;
  private viewport: Viewport = { scale: 40, originX: 180, originY: 160 };
  private currentResult: SimulationResult | null = null;
  private geometryFn: GeometryFunction = computeTrebuchetGeometry;

  constructor(private readonly canvas: HTMLCanvasElement) {
    const context = canvas.getContext('2d');
    if (!context) {
      throw new Error('Unable to create canvas context.');
    }
    this.ctx = context;
  }

  setGeometryFunction(fn: GeometryFunction): void {
    this.geometryFn = fn;
  }

  setSimulation(result: SimulationResult | null): void {
    this.currentResult = result;
    if (result) {
      this.viewport = computeViewport(this.canvas, result, this.geometryFn);
    }
  }

  drawPreview(params: TrebuchetParams, sample: SimulationSample): void {
    this.currentResult = null;
    this.viewport = computeViewport(this.canvas, {
      params,
      samples: [sample],
      stats: {
        range: sample.projectileX,
        maxHeight: params.h - sample.projectileY,
        peakSpeed: 0,
        releaseSpeed: 0,
        releaseHeight: 0,
        releaseTime: 0,
        flightTime: 0,
        totalTime: 0,
        liftOffTime: 0,
      },
    }, this.geometryFn);
    this.drawScene(params, sample, [sample]);
  }

  render(result: SimulationResult, time: number): void {
    this.currentResult = result;
    const sample = findSampleAtTime(result.samples, time);
    this.drawScene(result.params, sample, result.samples);
  }

  private drawScene(params: TrebuchetParams, sample: SimulationSample, trailSource: SimulationSample[]): void {
    const { ctx, canvas } = this;
    const geometry = this.geometryFn(params, sample);
    const pivot = worldToScreen(this.viewport, 0, 0);
    const groundY = worldToScreen(this.viewport, 0, params.h).y;

    // Scale element sizes based on frame height in pixels (consistent across designs)
    const frameHeightPx = params.h * this.viewport.scale;
    const armWidth = Math.max(3, frameHeightPx * 0.03);
    const postWidth = Math.max(4, frameHeightPx * 0.04);
    const cwRodWidth = Math.max(2, frameHeightPx * 0.02);
    const cwSize = Math.max(8, frameHeightPx * 0.1);
    const projRadius = Math.max(4, frameHeightPx * 0.04);
    const pivotRadius = Math.max(3, frameHeightPx * 0.025);

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawBackdrop(ctx, canvas, groundY);
    drawGround(ctx, canvas, groundY);

    // Draw tracks if present (FAT vertical rails + horizontal guide)
    if (geometry.tracks) {
      ctx.save();
      ctx.strokeStyle = '#6b7280';
      ctx.lineWidth = Math.max(2, frameHeightPx * 0.015);
      ctx.setLineDash([Math.max(3, frameHeightPx * 0.02), Math.max(2, frameHeightPx * 0.015)]);

      if (geometry.tracks.vertical) {
        const vt = geometry.tracks.vertical;
        const vtTop = worldToScreen(this.viewport, vt.x, vt.yTop);
        const vtBot = worldToScreen(this.viewport, vt.x, vt.yBottom);
        // Draw two parallel rails
        const railGap = Math.max(3, frameHeightPx * 0.025);
        ctx.beginPath();
        ctx.moveTo(vtTop.x - railGap, vtTop.y);
        ctx.lineTo(vtBot.x - railGap, vtBot.y);
        ctx.moveTo(vtTop.x + railGap, vtTop.y);
        ctx.lineTo(vtBot.x + railGap, vtBot.y);
        ctx.stroke();
      }

      if (geometry.tracks.horizontal) {
        const ht = geometry.tracks.horizontal;
        const pinPos = worldToScreen(this.viewport, ht.x, ht.y);
        const slotEnd = worldToScreen(this.viewport, ht.x + ht.length, ht.y);
        // Draw horizontal guide slot
        const slotGap = Math.max(2, frameHeightPx * 0.015);
        ctx.beginPath();
        ctx.moveTo(pinPos.x, pinPos.y - slotGap);
        ctx.lineTo(slotEnd.x, slotEnd.y - slotGap);
        ctx.moveTo(pinPos.x, pinPos.y + slotGap);
        ctx.lineTo(slotEnd.x, slotEnd.y + slotGap);
        ctx.stroke();

        // Draw fixed pin as a small filled circle
        ctx.setLineDash([]);
        ctx.fillStyle = '#9ca3af';
        ctx.beginPath();
        ctx.arc(pinPos.x, pinPos.y, Math.max(3, frameHeightPx * 0.02), 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.restore();
    }

    ctx.save();
    // Vertical frame post (from ground to frame top)
    ctx.strokeStyle = '#a87c4f';
    ctx.lineWidth = postWidth;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(pivot.x, groundY);
    ctx.lineTo(pivot.x, pivot.y);
    ctx.stroke();
    ctx.restore();

    // Pivot position in renderer y-DOWN coords (0 for HCW/FCW, moves for FAT)
    const actualPivot = worldToScreen(this.viewport, 0, geometry.pivotY ?? 0);

    const armStart = worldToScreen(this.viewport, geometry.counterweightAttach.x, geometry.counterweightAttach.y);
    const armEnd = worldToScreen(this.viewport, geometry.slingAttach.x, geometry.slingAttach.y);
    const weight = worldToScreen(this.viewport, geometry.counterweight.x, geometry.counterweight.y);
    const projectile = worldToScreen(this.viewport, sample.projectileX, sample.projectileY);

    drawTrail(ctx, this.viewport, trailSource, sample.time);

    ctx.save();
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = armWidth;
    ctx.beginPath();
    ctx.moveTo(armStart.x, armStart.y);
    ctx.lineTo(armEnd.x, armEnd.y);
    ctx.stroke();

    // CW hanging rod (from attach point on arm to the counterweight)
    ctx.strokeStyle = '#cbd5e1';
    ctx.lineWidth = cwRodWidth;
    ctx.beginPath();
    ctx.moveTo(armStart.x, armStart.y);
    ctx.lineTo(weight.x, weight.y);
    ctx.stroke();

    const slingTip =
      sample.stage === 'flight'
        ? worldToScreen(
            this.viewport,
            geometry.slingAttach.x - params.LS * Math.sin(sample.Aq + sample.Sq),
            geometry.slingAttach.y - params.LS * Math.cos(sample.Aq + sample.Sq),
          )
        : projectile;

    ctx.strokeStyle = '#f8fafc';
    ctx.beginPath();
    ctx.moveTo(armEnd.x, armEnd.y);
    ctx.lineTo(slingTip.x, slingTip.y);
    ctx.stroke();
    ctx.restore();

    drawPivot(ctx, actualPivot, pivotRadius);
    drawCounterweight(ctx, weight, cwSize);
    drawProjectile(ctx, projectile, projRadius);
    drawHud(ctx, sample, params, canvas);
  }
}

function computeViewport(canvas: HTMLCanvasElement, result: SimulationResult, geometryFn: GeometryFunction): Viewport {
  // First pass: find the actual mechanism bounding box (excluding flight)
  let mechMinX = Infinity, mechMaxX = -Infinity;
  let mechMinY = Infinity, mechMaxY = -Infinity;

  for (let index = 0; index < result.samples.length; index += Math.max(1, Math.floor(result.samples.length / 300))) {
    const sample = result.samples[index];
    if (sample.stage === 'flight') continue;
    const geometry = geometryFn(result.params, sample);
    const points = [
      geometry.counterweight,
      geometry.counterweightAttach,
      geometry.slingAttach,
      geometry.projectile,
      { x: 0, y: 0 },  // frame top
      { x: 0, y: result.params.h }, // ground at frame base
    ];
    for (const p of points) {
      mechMinX = Math.min(mechMinX, p.x);
      mechMaxX = Math.max(mechMaxX, p.x);
      mechMinY = Math.min(mechMinY, p.y);
      mechMaxY = Math.max(mechMaxY, p.y);
    }
  }

  // If only 1 sample (preview), use it
  if (!isFinite(mechMinX)) {
    const geom = geometryFn(result.params, result.samples[0]);
    const points = [geom.counterweight, geom.counterweightAttach, geom.slingAttach, geom.projectile, { x: 0, y: 0 }, { x: 0, y: result.params.h }];
    for (const p of points) {
      mechMinX = Math.min(mechMinX, p.x);
      mechMaxX = Math.max(mechMaxX, p.x);
      mechMinY = Math.min(mechMinY, p.y);
      mechMaxY = Math.max(mechMaxY, p.y);
    }
  }

  const mechW = mechMaxX - mechMinX;
  const mechH = mechMaxY - mechMinY;

  // Target: mechanism should fill ~35% of canvas height
  // Calculate the scale needed for that, then derive viewport from scale
  const targetFraction = 0.35;
  const targetScale = (canvas.height * targetFraction) / mechH;

  // Derive viewport dimensions from canvas size and target scale
  const vpWidth = (canvas.width - 80) / targetScale;
  const vpHeight = (canvas.height - 80) / targetScale;

  // Center the mechanism horizontally, position ground at ~65% from top
  const mechCenterX = (mechMinX + mechMaxX) / 2;
  const groundY = result.params.h;
  // Ground should be at ~65% of viewport height from top
  const groundFraction = 0.65;
  const vpMinY = groundY - vpHeight * groundFraction;
  const vpMaxY = vpMinY + vpHeight;
  const vpMinX = mechCenterX - vpWidth / 2;
  const vpMaxX = mechCenterX + vpWidth / 2;

  // Include flight trajectory: expand right if needed, but cap expansion
  let finalMinX = vpMinX, finalMaxX = vpMaxX, finalMinY = vpMinY, finalMaxY = vpMaxY;
  for (let index = 0; index < result.samples.length; index += Math.max(1, Math.floor(result.samples.length / 300))) {
    const sample = result.samples[index];
    if (sample.stage === 'flight') {
      // Allow viewport to grow to include flight, but at most 3× wider
      finalMaxX = Math.min(vpMinX + vpWidth * 3, Math.max(finalMaxX, sample.projectileX + mechW * 0.2));
      finalMinY = Math.min(finalMinY, sample.projectileY - mechH * 0.1);
    }
  }

  const width = finalMaxX - finalMinX;
  const height = finalMaxY - finalMinY;
  const scale = Math.min((canvas.width - 80) / width, (canvas.height - 80) / height);

  return {
    scale,
    originX: 40 - finalMinX * scale,
    originY: 40 - finalMinY * scale,
  };
}

function worldToScreen(viewport: Viewport, x: number, y: number): { x: number; y: number } {
  return {
    x: viewport.originX + x * viewport.scale,
    y: viewport.originY + y * viewport.scale,
  };
}

function drawBackdrop(ctx: CanvasRenderingContext2D, canvas: HTMLCanvasElement, groundY: number): void {
  const sky = ctx.createLinearGradient(0, 0, 0, groundY);
  sky.addColorStop(0, '#0f172a');
  sky.addColorStop(1, '#2563eb');
  ctx.fillStyle = sky;
  ctx.fillRect(0, 0, canvas.width, groundY);

  const field = ctx.createLinearGradient(0, groundY, 0, canvas.height);
  field.addColorStop(0, '#166534');
  field.addColorStop(1, '#14532d');
  ctx.fillStyle = field;
  ctx.fillRect(0, groundY, canvas.width, canvas.height - groundY);
}

function drawGround(ctx: CanvasRenderingContext2D, canvas: HTMLCanvasElement, groundY: number): void {
  ctx.save();
  ctx.strokeStyle = '#bbf7d0';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(0, groundY);
  ctx.lineTo(canvas.width, groundY);
  ctx.stroke();
  ctx.restore();
}

function drawPivot(ctx: CanvasRenderingContext2D, pivot: { x: number; y: number }, radius: number): void {
  ctx.save();
  ctx.fillStyle = '#f8fafc';
  ctx.beginPath();
  ctx.arc(pivot.x, pivot.y, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawCounterweight(ctx: CanvasRenderingContext2D, point: { x: number; y: number }, size: number): void {
  ctx.save();
  ctx.fillStyle = '#f59e0b';
  ctx.fillRect(point.x - size / 2, point.y - size / 2, size, size);
  ctx.restore();
}

function drawProjectile(ctx: CanvasRenderingContext2D, point: { x: number; y: number }, radius: number): void {
  ctx.save();
  ctx.fillStyle = '#fb7185';
  ctx.beginPath();
  ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawTrail(ctx: CanvasRenderingContext2D, viewport: Viewport, samples: SimulationSample[], time: number): void {
  const relevant = samples.filter((sample) => sample.time <= time);
  if (relevant.length < 2) {
    return;
  }

  // Subsample to ~300 points for performance while showing the full trajectory
  const maxPoints = 300;
  const step = Math.max(1, Math.floor(relevant.length / maxPoints));
  const trail: SimulationSample[] = [];
  for (let i = 0; i < relevant.length; i += step) {
    trail.push(relevant[i]);
  }
  // Always include the last point
  if (trail[trail.length - 1] !== relevant[relevant.length - 1]) {
    trail.push(relevant[relevant.length - 1]);
  }

  ctx.save();
  ctx.strokeStyle = 'rgba(251, 113, 133, 0.45)';
  ctx.lineWidth = 2;
  ctx.beginPath();
  trail.forEach((sample, index) => {
    const point = worldToScreen(viewport, sample.projectileX, sample.projectileY);
    if (index === 0) {
      ctx.moveTo(point.x, point.y);
    } else {
      ctx.lineTo(point.x, point.y);
    }
  });
  ctx.stroke();
  ctx.restore();
}

function drawHud(
  ctx: CanvasRenderingContext2D,
  sample: SimulationSample,
  params: TrebuchetParams,
  canvas: HTMLCanvasElement,
): void {
  const stageLabel =
    sample.stage === 'ground'
      ? 'Ground constrained'
      : sample.stage === 'lifted'
        ? 'Projectile lifted'
        : 'Free flight';

  ctx.save();
  ctx.fillStyle = 'rgba(15, 23, 42, 0.78)';
  ctx.fillRect(canvas.width - 250, 18, 220, 110);
  ctx.fillStyle = '#f8fafc';
  ctx.font = '600 16px Inter, system-ui, sans-serif';
  ctx.fillText(stageLabel, canvas.width - 232, 42);
  ctx.font = '14px Inter, system-ui, sans-serif';
  ctx.fillStyle = '#cbd5e1';
  ctx.fillText(`t = ${sample.time.toFixed(3)} s`, canvas.width - 232, 66);
  ctx.fillText(`speed = ${sample.projectileSpeed.toFixed(2)} m/s`, canvas.width - 232, 88);
  ctx.fillText(`flight angle = ${sample.releaseAngleNow.toFixed(1)}°`, canvas.width - 232, 110);
  ctx.restore();

  void params;
}
