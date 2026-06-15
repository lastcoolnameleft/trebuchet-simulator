import { context } from 'esbuild';
import { createServer } from 'node:http';
import { mkdir, readFile } from 'node:fs/promises';
import path from 'node:path';
import process from 'node:process';

const root = process.cwd();
const port = 8080;

const contentTypes = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'application/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.jpeg': 'image/jpeg',
  '.ico': 'image/x-icon',
};

await mkdir(path.join(root, 'dist'), { recursive: true });

const ctx = await context({
  entryPoints: ['src/app.ts'],
  bundle: true,
  format: 'iife',
  platform: 'browser',
  target: 'es2020',
  outfile: 'dist/bundle.js',
  logLevel: 'info',
});

await ctx.rebuild();
await ctx.watch();

const server = createServer(async (req, res) => {
  try {
    const requested = new URL(req.url ?? '/', 'http://localhost').pathname;
    const relativePath = requested === '/' ? 'index.html' : requested.replace(/^\//, '');
    const filePath = path.resolve(root, relativePath);

    if (!filePath.startsWith(root)) {
      res.writeHead(403).end('Forbidden');
      return;
    }

    const body = await readFile(filePath);
    const ext = path.extname(filePath).toLowerCase();
    res.writeHead(200, { 'Content-Type': contentTypes[ext] ?? 'application/octet-stream' });
    res.end(body);
  } catch {
    res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
    res.end('Not found');
  }
});

server.listen(port, () => {
  console.log(`Trebuchet simulator available at http://localhost:${port}`);
});

const shutdown = async () => {
  server.close();
  await ctx.dispose();
  process.exit(0);
};

process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);
