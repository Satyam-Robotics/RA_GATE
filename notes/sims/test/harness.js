// Browser harness for ONE simulation.
// CLI:  node notes/sims/test/harness.js <id> --out <dir> [--widths 1100,390] [--file <path to the sim js>]
// Module: const { buildPage, openSim } = require('./harness.js');
'use strict';
const fs = require('fs');
const path = require('path');

const notesDir = path.join(__dirname, '..', '..');
const simsDir = path.join(__dirname, '..');

function scriptTag(code) {
  return '<script>\n' + code.replace(/<\/script/gi, '<\\/script') + '\n</script>';
}

// Writes <outDir>/<id>.html and returns its path.
function buildPage(id, outDir, file) {
  outDir = path.resolve(outDir || '.');
  fs.mkdirSync(outDir, { recursive: true });
  const simFile = file ? path.resolve(file) : path.join(simsDir, id + '.js');
  if (!fs.existsSync(simFile)) throw new Error('harness: simulation file not found: ' + simFile);
  const css = fs.readFileSync(path.join(notesDir, 'page.css'), 'utf8') + '\n' +
    fs.readFileSync(path.join(simsDir, 'core.css'), 'utf8');
  const html = '<!doctype html><html><head><meta charset="utf-8">' +
    '<meta name="viewport" content="width=device-width, initial-scale=1">' +
    '<title>Harness: ' + id + '</title>\n<style>' + css + '</style></head><body>\n' +
    '<h2>Harness: ' + id + '</h2>\n<div class="sim" data-sim="' + id + '"></div>\n' +
    scriptTag('window.RASIM_EAGER = true;') + '\n' +
    scriptTag(fs.readFileSync(path.join(simsDir, 'core.js'), 'utf8')) + '\n' +
    scriptTag(fs.readFileSync(simFile, 'utf8')) + '\n</body></html>';
  const htmlPath = path.join(outDir, id + '.html');
  fs.writeFileSync(htmlPath, html);
  return htmlPath;
}

// Opens the harness page in Chromium with the simulation mounted.
// Returns { browser, context, page, errors, warnings, htmlPath }; errors is a live array of console errors and page errors.
async function openSim(id, opts) {
  opts = opts || {};
  const width = opts.width || 1100, height = opts.height || 900;
  const outDir = opts.outDir || path.join(require('os').tmpdir(), 'rasim-harness');
  const htmlPath = buildPage(id, outDir, opts.file);
  const { chromium } = require('playwright');
  const browser = await chromium.launch();
  const context = await browser.newContext({
    viewport: { width, height },
    deviceScaleFactor: opts.deviceScaleFactor || 1,
    hasTouch: !!opts.touch,
    isMobile: !!opts.touch,
  });
  const page = await context.newPage();
  const errors = [], warnings = [];
  page.on('console', msg => {
    if (msg.type() === 'error') errors.push('console.error: ' + msg.text());
    else if (msg.type() === 'warning') warnings.push('console.warn: ' + msg.text());
  });
  page.on('pageerror', err => errors.push('page error: ' + (err && err.message || err)));
  await page.goto('file://' + htmlPath);
  try {
    await page.waitForFunction(sid => !!(window.RASim && window.RASim.instances[sid]), id, { timeout: 5000 });
  } catch (e) {
    errors.push('timeout: RASim.instances["' + id + '"] did not appear within 5 s');
  }
  return { browser, context, page, errors, warnings, htmlPath };
}

async function runWidth(id, width, outDir, file) {
  const s = await openSim(id, { width, height: 900, outDir, file });
  const { browser, page } = s;
  const errors = s.errors, warnings = s.warnings;
  let screenshot = null;
  try {
    await page.waitForTimeout(400);
    const layout = await page.evaluate(sid => {
      const out = [];
      const de = document.documentElement;
      if (de.scrollWidth > window.innerWidth + 1) out.push('horizontal overflow: document scrollWidth ' + de.scrollWidth + ' > innerWidth ' + window.innerWidth);
      const panel = document.querySelector('.sim[data-sim="' + sid + '"]');
      if (!panel) { out.push('panel not found'); return out; }
      const pr = panel.getBoundingClientRect();
      function sel(e) {
        let s = e.tagName.toLowerCase();
        if (e.id) s += '#' + e.id;
        if (e.classList.length) s += '.' + Array.from(e.classList).join('.');
        ['data-control', 'data-stat', 'data-canvas', 'data-matrix', 'data-tag'].forEach(a => { if (e.hasAttribute(a)) s += '[' + a + '="' + e.getAttribute(a) + '"]'; });
        return s;
      }
      let n = 0;
      panel.querySelectorAll('*').forEach(e => {
        const r = e.getBoundingClientRect();
        if (r.width === 0 && r.height === 0) return;
        if (r.right > pr.right + 1 && n < 20) {
          n++;
          out.push('overflow: ' + sel(e) + ' right edge ' + Math.round(r.right) + ' > panel ' + Math.round(pr.right));
        }
      });
      return out;
    }, id);
    layout.forEach(w => warnings.push(w));
    screenshot = path.join(path.resolve(outDir), id + '-' + width + '.png');
    await page.screenshot({ path: screenshot, fullPage: true });
  } catch (e) {
    errors.push('harness: ' + (e && e.message || e));
  } finally {
    await browser.close();
  }
  return { width, errors, warnings, screenshot };
}

async function main(argv) {
  const args = argv.slice(2);
  const id = args[0];
  const get = name => { const i = args.indexOf(name); return i >= 0 ? args[i + 1] : null; };
  if (!id || id.startsWith('--')) {
    console.error('usage: node notes/sims/test/harness.js <id> --out <dir> [--widths 1100,390] [--file <sim.js>]');
    process.exit(2);
  }
  const outDir = path.resolve(get('--out') || path.join(require('os').tmpdir(), 'rasim-harness'));
  const widths = (get('--widths') || '1100,390').split(',').map(Number).filter(Boolean);
  const file = get('--file');
  const runs = [];
  for (const w of widths) runs.push(await runWidth(id, w, outDir, file));
  const ok = runs.every(r => r.errors.length === 0);
  console.log(JSON.stringify({ id, ok, runs }, null, 2));
  process.exit(ok ? 0 : 1);
}

module.exports = { buildPage, openSim };

if (require.main === module) {
  main(process.argv).catch(e => { console.error(e); process.exit(1); });
}
