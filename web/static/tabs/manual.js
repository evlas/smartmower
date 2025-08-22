// Tab: Controllo Manuale
import { App } from '/static/app.js';

function setMsg(msg, isErr=false){
  const el = document.getElementById('manual-status'); if(!el) return;
  el.textContent = msg; el.className = isErr? 'error' : 'muted';
}

async function apiJson(url, body){
  const r = await App.fetchJson(url, { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(body||{}) });
  if(!r.ok){ setMsg('Errore: ' + (r.text||''), true); return false; }
  return true;
}

// rate limit 10Hz
let lastVelTs = 0;
async function sendVel(lin, ang){
  const now = performance.now();
  if(now - lastVelTs < 100) return true; // drop to 10Hz
  lastVelTs = now;
  return apiJson('/api/manual/cmd_vel', { linear: lin, angular: ang });
}
function sendBlade(on){ return apiJson('/api/manual/blade', { on }); }

// hold-to-drive support
let driveTimer = null;
function startDrive(lin, ang){
  stopDrive();
  // send immediately then at 10Hz
  sendVel(lin, ang);
  driveTimer = setInterval(()=> sendVel(lin, ang), 100);
}
function stopDrive(){ if(driveTimer){ clearInterval(driveTimer); driveTimer = null; } }

export function init(){
  const btnEnter = document.getElementById('btn-enter-manual');
  const btnExit  = document.getElementById('btn-exit-manual');
  btnEnter?.addEventListener('click', async ()=>{ if(await apiJson('/api/manual/enter', {})) setMsg('Modalità manuale richiesta'); });
  btnExit?.addEventListener('click', async ()=>{ if(await apiJson('/api/manual/exit', {})) setMsg('Uscita da manuale richiesta'); });

  const rngLin = document.getElementById('lin');
  const rngAng = document.getElementById('ang');
  const prev = document.getElementById('vel-preview');
  const updatePrev = ()=>{ prev && (prev.textContent = `v=${Number(rngLin?.value||0).toFixed(2)} m/s, w=${Number(rngAng?.value||0).toFixed(2)} rad/s`); };
  rngLin?.addEventListener('input', updatePrev);
  rngAng?.addEventListener('input', updatePrev);
  updatePrev();

  const btnSend = document.getElementById('btn-send-vel');
  btnSend?.addEventListener('click', async ()=>{
    const lin = Number(rngLin?.value||0); const ang = Number(rngAng?.value||0);
    if(await sendVel(lin, ang)) setMsg(`Velocità inviata: v=${lin.toFixed(2)}, w=${ang.toFixed(2)}`);
  });

  // hold-to-drive (mouse/touch)
  const bindHold = (id, lin, ang)=>{
    const el = document.getElementById(id); if(!el) return;
    const down = ()=> startDrive(lin, ang);
    const up = ()=> stopDrive();
    el.addEventListener('mousedown', down);
    el.addEventListener('touchstart', down);
    ['mouseup','mouseleave','touchend','touchcancel'].forEach(ev=> el.addEventListener(ev, up));
  };
  bindHold('mv-forward',  0.4,  0.0);
  bindHold('mv-backward', -0.3, 0.0);
  bindHold('mv-left',     0.0,  0.8);
  bindHold('mv-right',    0.0, -0.8);
  document.getElementById('mv-stop')?.addEventListener('click', ()=> { stopDrive(); sendVel(0,0); });

  // lama
  document.getElementById('blade-on')?.addEventListener('click', async ()=> { if(await sendBlade(true)) setMsg('Lama ON'); });
  document.getElementById('blade-off')?.addEventListener('click', async ()=> { if(await sendBlade(false)) setMsg('Lama OFF'); });

  // preset velocità
  document.getElementById('preset-slow')?.addEventListener('click', ()=> { rngLin.value = '0.15'; rngAng.value = '0.3'; updatePrev(); });
  document.getElementById('preset-medium')?.addEventListener('click', ()=> { rngLin.value = '0.30'; rngAng.value = '0.6'; updatePrev(); });
  document.getElementById('preset-fast')?.addEventListener('click', ()=> { rngLin.value = '0.50'; rngAng.value = '0.8'; updatePrev(); });

  // scorciatoie tastiera: W/A/S/D, SPazio per stop
  const keyState = {};
  const applyKeyDrive = ()=>{
    if(keyState[' '] || keyState['Space']) { stopDrive(); sendVel(0,0); return; }
    let lin = 0, ang = 0;
    if(keyState['w'] || keyState['ArrowUp']) lin += 0.4;
    if(keyState['s'] || keyState['ArrowDown']) lin -= 0.3;
    if(keyState['a'] || keyState['ArrowLeft']) ang += 0.8;
    if(keyState['d'] || keyState['ArrowRight']) ang -= 0.8;
    if(lin !== 0 || ang !== 0) startDrive(lin, ang); else stopDrive();
  };
  window.addEventListener('keydown', (e)=>{
    if(['INPUT','TEXTAREA'].includes((e.target?.tagName||''))) return;
    keyState[e.key] = true; applyKeyDrive();
  });
  window.addEventListener('keyup', (e)=>{
    keyState[e.key] = false; applyKeyDrive();
  });
}
