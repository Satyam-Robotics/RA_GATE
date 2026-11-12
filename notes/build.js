// Builds notes/RA_GATE2027_Notes.html and notes/RA_GATE2027_Notes.pdf from notes/src/*.html
// Usage: node notes/build.js
const fs = require('fs');
const path = require('path');
const { chromium } = require('playwright');

const dir = __dirname;
const srcDir = path.join(dir, 'src');
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

const css = `
@page { size: A4; margin: 16mm 14mm 16mm 14mm; }
body { font-family: "DejaVu Serif", Georgia, serif; font-size: 10.2pt; line-height: 1.42; color: #1a1a1a; }
h1 { font-family: "DejaVu Sans", Arial, sans-serif; color: #6b1d1d; border-bottom: 2px solid #6b1d1d; padding-bottom: 4px; page-break-before: always; font-size: 19pt; }
h2 { font-family: "DejaVu Sans", Arial, sans-serif; color: #1f4e79; font-size: 14pt; margin-top: 18px; border-left: 4px solid #1f4e79; padding-left: 7px; page-break-after: avoid; }
h3 { font-family: "DejaVu Sans", Arial, sans-serif; color: #2e6b30; font-size: 11pt; margin: 12px 0 4px; page-break-after: avoid; }
ul { margin: 3px 0 6px 0; padding-left: 20px; }
li { margin: 2px 0; }
table { border-collapse: collapse; width: 100%; margin: 6px 0 10px; font-size: 9.4pt; page-break-inside: avoid; }
th, td { border: 1px solid #999; padding: 3px 6px; vertical-align: top; text-align: left; }
th { background: #f3ead2; }
b { color: #222; }
.cover { text-align: center; padding-top: 70mm; }
.cover h1 { page-break-before: avoid; border: none; font-size: 30pt; }
.cover p { font-size: 13pt; color: #444; }
.toc h1 { page-break-before: always; }
.toc ul { list-style: none; padding-left: 0; columns: 2; column-gap: 24px; }
.toc li.h1 { font-weight: bold; margin-top: 8px; color: #6b1d1d; }
.toc li.h2 { padding-left: 12px; font-size: 9.5pt; }
.toc a { color: inherit; text-decoration: none; }
`;

const cover = `<div class="cover"><h1>GATE 2027<br>Robotics and Automation (RA)</h1>
<p>Complete topic-wise study notes</p>
<p>Part A: Engineering Mathematics · Mechatronics · Robotics &amp; Automation<br>
Part B1: Electrical Engineering · Part B2: Mechanical Engineering</p>
<p style="font-size:10pt;margin-top:40mm">Organized exactly by the official GATE 2027 RA syllabus (IIT Madras).<br>
Concise exam notes: definitions, key results, formulas and standard cases.</p></div>`;

const html = `<!doctype html><html><head><meta charset="utf-8"><title>GATE 2027 RA Notes</title>
<style>${css}</style></head><body>${cover}${tocHtml}${withIds}</body></html>`;

const htmlPath = path.join(dir, 'RA_GATE2027_Notes.html');
fs.writeFileSync(htmlPath, html);

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
  console.log('Built', htmlPath, 'and PDF');
})();
