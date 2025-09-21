// CommonJS build step: gzip /web -> /data
// Usage: node tools/build_web_assets.cjs
const fs = require('fs');
const fsp = fs.promises;
const path = require('path');
const zlib = require('zlib');
const crypto = require('crypto');

const PROJECT_DIR = process.env.PROJECT_DIR
  ? path.resolve(process.env.PROJECT_DIR)
  : process.cwd();

const WEB_DIR = path.join(PROJECT_DIR, 'web');
const DATA_DIR = path.join(PROJECT_DIR, 'data');
const ASSETS = ['index.html', 'style.css', 'app.js'];

async function ensureDir(p) {
  await fsp.mkdir(p, { recursive: true });
}

function sha256FileSync(p) {
  const h = crypto.createHash('sha256');
  const fd = fs.openSync(p, 'r');
  try {
    const buf = Buffer.allocUnsafe(1024 * 1024);
    let bytesRead = 0;
    while ((bytesRead = fs.readSync(fd, buf, 0, buf.length, null)) > 0) {
      h.update(buf.subarray(0, bytesRead));
    }
  } finally {
    fs.closeSync(fd);
  }
  return h.digest('hex');
}

async function writeGzip(src, dstGz) {
  const tmp = dstGz + '.tmp';
  await new Promise((resolve, reject) => {
    const inp = fs.createReadStream(src);
    const out = fs.createWriteStream(tmp);
    const gz = zlib.createGzip({ level: 9 });
    inp.on('error', reject);
    out.on('error', reject);
    out.on('finish', resolve);
    inp.pipe(gz).pipe(out);
  });

  // 바뀐 내용만 교체
  if (fs.existsSync(dstGz)) {
    const a = sha256FileSync(dstGz);
    const b = sha256FileSync(tmp);
    if (a === b) {
      fs.unlinkSync(tmp);
      console.log(`[web] unchanged ${path.basename(dstGz)}`);
      return;
    }
  }
  fs.renameSync(tmp, dstGz);
  const stat = fs.statSync(dstGz);
  console.log(`[web] wrote     ${path.basename(dstGz)} (${stat.size} bytes)`);
}

(async function main() {
  if (!fs.existsSync(WEB_DIR)) {
    console.log(`[web] skip (no ${WEB_DIR})`);
    return;
  }
  await ensureDir(DATA_DIR);

  for (const name of ASSETS) {
    const src = path.join(WEB_DIR, name);
    if (!fs.existsSync(src)) {
      console.log(`[web] warn  missing ${src}`);
      continue;
    }
    const dstGz = path.join(DATA_DIR, `${name}.gz`);
    await writeGzip(src, dstGz);
  }

  // 원본도 /data에 복사하고 싶으면 주석 해제
  // for (const name of ASSETS) {
  //   const src = path.join(WEB_DIR, name);
  //   if (fs.existsSync(src)) {
  //     await fsp.copyFile(src, path.join(DATA_DIR, name));
  //   }
  // }
})().catch(err => {
  console.error('[web] error', err);
  process.exit(1);
});
