// Builds notes/RA_GATE2027_Notes.html and notes/RA_GATE2027_Notes.pdf from notes/src/*.html
// Usage: node notes/build.js [--html-only]
//   --html-only  write the HTML page only and skip the PDF step
// Styles: notes/page.css (notes layout, print + screen) followed by notes/sims/core.css (screen-only simulations).
// Scripts: notes/sims/core.js, then one <script> per notes/sims/*.js (excluding core.js and names starting with '_').
const fs = require('fs');
const path = require('path');

const dir = __dirname;
const srcDir = path.join(dir, 'src');
const simsDir = path.join(dir, 'sims');
const htmlOnly = process.argv.includes('--html-only');

const parts = fs.readdirSync(srcDir).filter(f => f.endsWith('.html')).sort();
const body = parts.map(f => `<section>${fs.readFileSync(path.join(srcDir, f), 'utf8')}</section>`).join('\n');

// Table of contents from h1/h2 headings
let n = 0;
const toc = [];
const withIds = body.replace(/<(h[12])( id="[^"]*")?>(.*?)<\/\1>/g, (m, tag, idAttr, text) => {
  const id = idAttr ? idAttr.match(/"(.*)"/)[1] : `s${++n}`;
  toc.push({ tag, id, text: text.replace(/<[^>]+>/g, '') });
  return `<${tag} id="${id}">${text}</${tag}>`;
});
const tocHtml = '<nav class="toc"><h1>Contents</h1><ul>' + toc.map(t =>
  `<li class="${t.tag}"><a href="#${t.id}">${t.text}</a></li>`).join('') + '</ul></nav>';
const simIndexHtml = '<nav class="sim-index" data-sim-index></nav>';

const css = fs.readFileSync(path.join(dir, 'page.css'), 'utf8') + '\n' +
  fs.readFileSync(path.join(simsDir, 'core.css'), 'utf8');

// Inline JS safely: a '</script' sequence inside the code must not terminate the element.
const inlineScript = (code, name) =>
  `<script data-src="${name}">\n${code.replace(/<\/script/gi, '<\\/script')}\n</script>`;
const simFiles = fs.readdirSync(simsDir)
  .filter(f => f.endsWith('.js') && f !== 'core.js' && !f.startsWith('_') &&
    fs.statSync(path.join(simsDir, f)).isFile())
  .sort();
const scripts = [inlineScript(fs.readFileSync(path.join(simsDir, 'core.js'), 'utf8'), 'sims/core.js')]
  .concat(simFiles.map(f => inlineScript(fs.readFileSync(path.join(simsDir, f), 'utf8'), 'sims/' + f)))
  .join('\n');

const cover = `<div class="cover"><h1>GATE 2027<br>Robotics and Automation (RA)</h1>
<p>Complete topic-wise study notes</p>
<p>Part A: Engineering Mathematics · Mechatronics · Robotics &amp; Automation<br>
Part B1: Electrical Engineering · Part B2: Mechanical Engineering</p>
<p style="font-size:10pt;margin-top:40mm">Organized exactly by the official GATE 2027 RA syllabus (IIT Madras).<br>
Concise exam notes: definitions, key results, formulas and standard cases.</p></div>`;

const html = `<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>GATE 2027 RA Notes</title>
<style>${css}</style></head><body>${cover}${tocHtml}${simIndexHtml}${withIds}
${scripts}
</body></html>`;

const htmlPath = path.join(dir, 'RA_GATE2027_Notes.html');
fs.writeFileSync(htmlPath, html);
console.log('Built', htmlPath, `(${simFiles.length} simulation file${simFiles.length === 1 ? '' : 's'}: ${simFiles.join(', ') || 'none'})`);

if (!htmlOnly) {
  const { chromium } = require('playwright');
  (async () => {
    const browser = await chromium.launch();
    const page = await browser.newPage();
    await page.goto('file://' + htmlPath);
    await page.pdf({
      path: path.join(dir, 'RA_GATE2027_Notes.pdf'), format: 'A4', printBackground: true,
      displayHeaderFooter: true,
      headerTemplate: '<div style="font-size:7pt;width:100%;text-align:right;padding-right:14mm;color:#888">GATE 2027 RA — Study Notes</div>',
      footerTemplate: '<div style="font-size:8pt;width:100%;text-align:center;color:#666"><span class="pageNumber"></span> / <span class="totalPages"></span></div>',
      margin: { top: '16mm', bottom: '16mm', left: '14mm', right: '14mm' },
    });
    await browser.close();
    console.log('Built PDF', path.join(dir, 'RA_GATE2027_Notes.pdf'));
  })();
}
