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
  // Fixed PiP bounds computed once from all pre-launch frames
  private pipBounds: { minX: number; maxX: number; minY: number; maxY: number } | null = null;

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
      this.pipBounds = this.computePipBounds(result);
    }
  }

  private computePipBounds(result: SimulationResult): { minX: number; maxX: number; minY: number; maxY: number } {
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    // Sample every few pre-launch frames to find the full mechanism extent
    const preLaunch = result.samples.filter(s => s.stage !== 'flight');
    const step = Math.max(1, Math.floor(preLaunch.length / 30));
    for (let i = 0; i < preLaunch.length; i += step) {
      const geom = this.geometryFn(result.params, preLaunch[i]);
      const points = [geom.counterweight, geom.counterweightAttach, geom.slingAttach, geom.projectile, { x: 0, y: 0 }, { x: 0, y: result.params.h }];
      for (const p of points) {
        minX = Math.min(minX, p.x);
        maxX = Math.max(maxX, p.x);
        minY = Math.min(minY, p.y);
        maxY = Math.max(maxY, p.y);
      }
    }
    // Always include last pre-launch frame
    if (preLaunch.length > 0) {
      const geom = this.geometryFn(result.params, preLaunch[preLaunch.length - 1]);
      const points = [geom.counterweight, geom.counterweightAttach, geom.slingAttach, geom.projectile, { x: 0, y: 0 }, { x: 0, y: result.params.h }];
      for (const p of points) {
        minX = Math.min(minX, p.x);
        maxX = Math.max(maxX, p.x);
        minY = Math.min(minY, p.y);
        maxY = Math.max(maxY, p.y);
      }
    }
    return { minX, maxX, minY, maxY };
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

    // Picture-in-Picture inset: show zoomed mechanism when main view is zoomed out
    this.drawPiP(params, sample, frameHeightPx);
  }

  private drawPiP(params: TrebuchetParams, sample: SimulationSample, mainFrameHeightPx: number): void {
    // Only show PiP when mechanism is too small in main view (< 15% of canvas height)
    const mechTooSmall = mainFrameHeightPx < this.canvas.height * 0.15;
    if (!mechTooSmall) return;

    const { ctx, canvas } = this;
    const geometry = this.geometryFn(params, sample);

    // PiP dimensions and position (bottom-right corner)
    const pipW = Math.round(canvas.width * 0.28);
    const pipH = Math.round(canvas.height * 0.35);
    const pipX = canvas.width - pipW - 12;
    const pipY = canvas.height - pipH - 12;

    // Use precomputed fixed bounds so the viewport doesn't shift frame-to-frame
    let minX: number, maxX: number, minY: number, maxY: number;
    if (this.pipBounds) {
      minX = this.pipBounds.minX;
      maxX = this.pipBounds.maxX;
      minY = this.pipBounds.minY;
      maxY = this.pipBounds.maxY;
    } else {
      const points = [
        geometry.counterweight,
        geometry.counterweightAttach,
        geometry.slingAttach,
        geometry.projectile,
        { x: 0, y: 0 },
        { x: 0, y: params.h },
      ];
      minX = Infinity; maxX = -Infinity; minY = Infinity; maxY = -Infinity;
      for (const p of points) {
        minX = Math.min(minX, p.x);
        maxX = Math.max(maxX, p.x);
        minY = Math.min(minY, p.y);
        maxY = Math.max(maxY, p.y);
      }
    }
    const mechW = maxX - minX;
    const mechH = maxY - minY;
    // Add padding (40% around mechanism)
    const pad = Math.max(mechW, mechH) * 0.4;
    minX -= pad; maxX += pad;
    minY -= pad; maxY += pad;
    // Ensure ground is visible
    maxY = Math.max(maxY, params.h + pad * 0.3);

    const vpW = maxX - minX;
    const vpH = maxY - minY;
    const pipScale = Math.min((pipW - 16) / vpW, (pipH - 16) / vpH);
    const pipViewport: Viewport = {
      scale: pipScale,
      originX: pipX + 8 - minX * pipScale,
      originY: pipY + 8 - minY * pipScale,
    };

    // Draw PiP background
    ctx.save();
    ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.5)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.roundRect(pipX, pipY, pipW, pipH, 6);
    ctx.fill();
    ctx.stroke();

    // Clip to PiP region
    ctx.beginPath();
    ctx.rect(pipX, pipY, pipW, pipH);
    ctx.clip();

    // Draw ground in PiP
    const pipGroundY = pipViewport.originY + params.h * pipViewport.scale;
    ctx.fillStyle = '#1a5c2a';
    ctx.fillRect(pipX, pipGroundY, pipW, pipH - (pipGroundY - pipY));
    ctx.strokeStyle = '#6ee7b7';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(pipX, pipGroundY);
    ctx.lineTo(pipX + pipW, pipGroundY);
    ctx.stroke();

    // Scale element sizes for PiP
    const pipFrameH = params.h * pipScale;
    const pArmW = Math.max(2, pipFrameH * 0.03);
    const pPostW = Math.max(2, pipFrameH * 0.04);
    const pCwSize = Math.max(5, pipFrameH * 0.1);
    const pProjR = Math.max(3, pipFrameH * 0.04);
    const pPivotR = Math.max(2, pipFrameH * 0.025);

    // Draw tracks in PiP
    if (geometry.tracks) {
      ctx.strokeStyle = '#6b7280';
      ctx.lineWidth = Math.max(1, pipFrameH * 0.012);
      ctx.setLineDash([Math.max(2, pipFrameH * 0.015), Math.max(1, pipFrameH * 0.012)]);
      if (geometry.tracks.vertical) {
        const vt = geometry.tracks.vertical;
        const vtTop = worldToScreen(pipViewport, vt.x, vt.yTop);
        const vtBot = worldToScreen(pipViewport, vt.x, vt.yBottom);
        const rg = Math.max(2, pipFrameH * 0.02);
        ctx.beginPath();
        ctx.moveTo(vtTop.x - rg, vtTop.y); ctx.lineTo(vtBot.x - rg, vtBot.y);
        ctx.moveTo(vtTop.x + rg, vtTop.y); ctx.lineTo(vtBot.x + rg, vtBot.y);
        ctx.stroke();
      }
      if (geometry.tracks.horizontal) {
        const ht = geometry.tracks.horizontal;
        const pinP = worldToScreen(pipViewport, ht.x, ht.y);
        const slotE = worldToScreen(pipViewport, ht.x + ht.length, ht.y);
        const sg = Math.max(1, pipFrameH * 0.012);
        ctx.beginPath();
        ctx.moveTo(pinP.x, pinP.y - sg); ctx.lineTo(slotE.x, slotE.y - sg);
        ctx.moveTo(pinP.x, pinP.y + sg); ctx.lineTo(slotE.x, slotE.y + sg);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = '#9ca3af';
        ctx.beginPath();
        ctx.arc(pinP.x, pinP.y, Math.max(2, pipFrameH * 0.015), 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.setLineDash([]);
    }

    // Frame post
    const pipPivot = worldToScreen(pipViewport, 0, 0);
    ctx.strokeStyle = '#a87c4f';
    ctx.lineWidth = pPostW;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(pipPivot.x, pipGroundY);
    ctx.lineTo(pipPivot.x, pipPivot.y);
    ctx.stroke();

    // Arm
    const pipArmStart = worldToScreen(pipViewport, geometry.counterweightAttach.x, geometry.counterweightAttach.y);
    const pipArmEnd = worldToScreen(pipViewport, geometry.slingAttach.x, geometry.slingAttach.y);
    const pipWeight = worldToScreen(pipViewport, geometry.counterweight.x, geometry.counterweight.y);
    const pipProj = worldToScreen(pipViewport, sample.projectileX, sample.projectileY);

    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = pArmW;
    ctx.beginPath();
    ctx.moveTo(pipArmStart.x, pipArmStart.y);
    ctx.lineTo(pipArmEnd.x, pipArmEnd.y);
    ctx.stroke();

    // CW rod
    ctx.strokeStyle = '#cbd5e1';
    ctx.lineWidth = Math.max(1, pipFrameH * 0.015);
    ctx.beginPath();
    ctx.moveTo(pipArmStart.x, pipArmStart.y);
    ctx.lineTo(pipWeight.x, pipWeight.y);
    ctx.stroke();

    // Sling
    const pipSlingTip = sample.stage === 'flight'
      ? worldToScreen(pipViewport,
          geometry.slingAttach.x - params.LS * Math.sin(sample.Aq + sample.Sq),
          geometry.slingAttach.y - params.LS * Math.cos(sample.Aq + sample.Sq))
      : pipProj;
    ctx.strokeStyle = '#f8fafc';
    ctx.beginPath();
    ctx.moveTo(pipArmEnd.x, pipArmEnd.y);
    ctx.lineTo(pipSlingTip.x, pipSlingTip.y);
    ctx.stroke();

    // Pivot, CW, Projectile
    const pipActualPivot = worldToScreen(pipViewport, 0, geometry.pivotY ?? 0);
    drawPivot(ctx, pipActualPivot, pPivotR);
    drawCounterweight(ctx, pipWeight, pCwSize);
    if (sample.stage !== 'flight') {
      drawProjectile(ctx, pipProj, pProjR);
    }

    ctx.restore();
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
