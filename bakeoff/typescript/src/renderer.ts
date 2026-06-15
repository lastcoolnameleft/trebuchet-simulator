import { computeTrebuchetGeometry } from './geometry';
import { findSampleAtTime, type SimulationResult, type SimulationSample, type TrebuchetParams } from './physics';

interface Viewport {
  scale: number;
  originX: number;
  originY: number;
}

export class TrebuchetRenderer {
  private readonly ctx: CanvasRenderingContext2D;
  private viewport: Viewport = { scale: 40, originX: 180, originY: 160 };
  private currentResult: SimulationResult | null = null;

  constructor(private readonly canvas: HTMLCanvasElement) {
    const context = canvas.getContext('2d');
    if (!context) {
      throw new Error('Unable to create canvas context.');
    }
    this.ctx = context;
  }

  setSimulation(result: SimulationResult | null): void {
    this.currentResult = result;
    if (result) {
      this.viewport = computeViewport(this.canvas, result);
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
    });
    this.drawScene(params, sample, [sample]);
  }

  render(result: SimulationResult, time: number): void {
    this.currentResult = result;
    const sample = findSampleAtTime(result.samples, time);
    this.drawScene(result.params, sample, result.samples);
  }

  private drawScene(params: TrebuchetParams, sample: SimulationSample, trailSource: SimulationSample[]): void {
    const { ctx, canvas } = this;
    const geometry = computeTrebuchetGeometry(params, sample);
    const pivot = worldToScreen(this.viewport, 0, 0);
    const groundY = worldToScreen(this.viewport, 0, params.h).y;

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    drawBackdrop(ctx, canvas, groundY);
    drawGround(ctx, canvas, groundY);

    ctx.save();
    ctx.strokeStyle = '#64748b';
    ctx.lineWidth = 10;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(pivot.x, groundY);
    ctx.lineTo(pivot.x, pivot.y);
    ctx.stroke();

    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(pivot.x - 24, groundY);
    ctx.lineTo(pivot.x + 24, groundY);
    ctx.stroke();
    ctx.restore();

    const armStart = worldToScreen(this.viewport, geometry.counterweightAttach.x, geometry.counterweightAttach.y);
    const armEnd = worldToScreen(this.viewport, geometry.slingAttach.x, geometry.slingAttach.y);
    const weight = worldToScreen(this.viewport, geometry.counterweight.x, geometry.counterweight.y);
    const projectile = worldToScreen(this.viewport, sample.projectileX, sample.projectileY);

    drawTrail(ctx, this.viewport, trailSource, sample.time);

    ctx.save();
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = Math.max(4, this.viewport.scale * 0.12);
    ctx.beginPath();
    ctx.moveTo(armStart.x, armStart.y);
    ctx.lineTo(armEnd.x, armEnd.y);
    ctx.stroke();

    ctx.strokeStyle = '#cbd5e1';
    ctx.lineWidth = Math.max(2, this.viewport.scale * 0.07);
    ctx.beginPath();
    ctx.moveTo(armStart.x, armStart.y);
    ctx.lineTo(weight.x, weight.y);
    ctx.stroke();

    const slingTip =
      sample.stage === 'flight'
        ? worldToScreen(
            this.viewport,
            geometry.slingAttach.x - params.LS * Math.sin(sample.Aq + sample.Sq),
            geometry.slingAttach.y + params.LS * Math.cos(sample.Aq + sample.Sq),
          )
        : projectile;

    ctx.strokeStyle = '#f8fafc';
    ctx.beginPath();
    ctx.moveTo(armEnd.x, armEnd.y);
    ctx.lineTo(slingTip.x, slingTip.y);
    ctx.stroke();
    ctx.restore();

    drawPivot(ctx, pivot);
    drawCounterweight(ctx, weight, this.viewport.scale);
    drawProjectile(ctx, projectile, this.viewport.scale);
    drawHud(ctx, sample, params, canvas);
  }
}

function computeViewport(canvas: HTMLCanvasElement, result: SimulationResult): Viewport {
  let minX = -result.params.LAl - result.params.LS - 2;
  let maxX = result.params.LAs + result.params.LW + 2;
  let minY = -4;
  let maxY = result.params.h + 2;

  for (let index = 0; index < result.samples.length; index += Math.max(1, Math.floor(result.samples.length / 300))) {
    const sample = result.samples[index];
    const geometry = computeTrebuchetGeometry(result.params, sample);
    const points = [
      geometry.counterweight,
      geometry.counterweightAttach,
      geometry.slingAttach,
      { x: sample.projectileX, y: sample.projectileY },
    ];

    for (const point of points) {
      minX = Math.min(minX, point.x);
      maxX = Math.max(maxX, point.x);
      minY = Math.min(minY, point.y);
      maxY = Math.max(maxY, point.y);
    }
  }

  maxY = Math.max(maxY, result.params.h + 1.5);
  const width = Math.max(8, maxX - minX);
  const height = Math.max(8, maxY - minY);
  const scale = Math.min((canvas.width - 120) / width, (canvas.height - 120) / height);

  return {
    scale,
    originX: 60 - minX * scale,
    originY: 60 - minY * scale,
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

function drawPivot(ctx: CanvasRenderingContext2D, pivot: { x: number; y: number }): void {
  ctx.save();
  ctx.fillStyle = '#f8fafc';
  ctx.beginPath();
  ctx.arc(pivot.x, pivot.y, 8, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawCounterweight(ctx: CanvasRenderingContext2D, point: { x: number; y: number }, scale: number): void {
  const size = Math.max(14, scale * 0.24);
  ctx.save();
  ctx.fillStyle = '#f59e0b';
  ctx.fillRect(point.x - size / 2, point.y - size / 2, size, size);
  ctx.restore();
}

function drawProjectile(ctx: CanvasRenderingContext2D, point: { x: number; y: number }, scale: number): void {
  ctx.save();
  ctx.fillStyle = '#fb7185';
  ctx.beginPath();
  ctx.arc(point.x, point.y, Math.max(6, scale * 0.09), 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawTrail(ctx: CanvasRenderingContext2D, viewport: Viewport, samples: SimulationSample[], time: number): void {
  const trail = samples.filter((sample) => sample.stage === 'flight' && sample.time <= time).slice(-160);
  if (trail.length < 2) {
    return;
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
