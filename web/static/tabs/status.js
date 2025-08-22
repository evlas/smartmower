// Tab: Stato & Mappa (quadrato 10.000 m²)
import { App } from '/static/app.js';

function metersToPixels(m, scale){ return m * scale; }

function drawField(ctx, sizePx){
  // sfondo
  ctx.fillStyle = '#0b132b';
  ctx.fillRect(0, 0, sizePx, sizePx);
  // bordo esterno
  ctx.strokeStyle = '#3a506b';
  ctx.lineWidth = 2;
  ctx.strokeRect(1, 1, sizePx-2, sizePx-2);
  // griglia ogni 10 metri
  ctx.strokeStyle = '#1f2f4f';
  ctx.lineWidth = 1;
  const step = 10; // m
  const scale = sizePx / 100; // 100m lato
  for(let m = step; m < 100; m += step){
    const p = Math.round(metersToPixels(m, scale)) + 0.5;
    // verticale
    ctx.beginPath(); ctx.moveTo(p, 0); ctx.lineTo(p, sizePx); ctx.stroke();
    // orizzontale
    ctx.beginPath(); ctx.moveTo(0, p); ctx.lineTo(sizePx, p); ctx.stroke();
  }
  // etichetta
  ctx.fillStyle = '#2a9d8f';
  ctx.font = '14px system-ui, sans-serif';
  ctx.fillText('Area 100m x 100m (10.000 m²) - scala 1m ≈ ' + (scale.toFixed(2)) + ' px', 12, 20);
}

async function updatePanels(){
  const st = await App.getState();
  if(st.ok && st.json){
    const data = st.json;
    const text = data.current || data.state || JSON.stringify(data);
    const el = document.getElementById('robot-state'); if(el) el.textContent = text;
  } else {
    App.setStatus('Errore stato: ' + (st.text||'N/A'), true);
  }
  const bt = await App.getBattery();
  if(bt.ok && bt.json){
    const p = Number(bt.json.percentage ?? bt.json.level ?? 0);
    const pct = isNaN(p)? '—%' : (p + '%');
    const ep = document.getElementById('battery-pct'); if(ep) ep.textContent = pct;
    const es = document.getElementById('battery-status'); if(es) es.textContent = bt.json.status || '';
  }
}

function setupControls(){
  const bind = (id, action) => {
    const b = document.getElementById(id); if(!b) return;
    b.addEventListener('click', () => App.sendCmd(action));
  };
  bind('btn-start', 'START');
  bind('btn-pause', 'PAUSE');
  bind('btn-stop', 'STOP');
  bind('btn-docking', 'DOCKING');
}

export function init(){
  // canvas 640x640 rappresenta 100m x 100m
  const c = document.getElementById('map');
  if(c){
    const sizePx = Math.min(c.width, c.height);
    const ctx = c.getContext('2d');
    drawField(ctx, sizePx);
  }
  setupControls();
  updatePanels();
  // polling leggero
  const t1 = setInterval(updatePanels, 3000);
  // opzionale: salvataggio handler per cleanup futuro
  c?.addEventListener('tab:dispose', () => { clearInterval(t1); });
}
