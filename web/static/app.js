// App Utente - SmartMower (orchestratore tabs)
export const App = {
  async fetchJson(url, opts){
    const res = await fetch(url, opts||{});
    const txt = await res.text();
    try{ return { ok: res.ok, json: JSON.parse(txt), text: txt }; }catch{ return { ok: res.ok, json: null, text: txt }; }
  },
  setStatus(msg, isErr=false){
    const el = document.getElementById('status'); if(!el) return;
    el.textContent = msg; el.className = isErr? 'error' : 'muted';
  },
  async sendCmd(action){
    const r = await this.fetchJson('/api/state/cmd', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ action }) });
    if(!r.ok) this.setStatus('Comando fallito: ' + (r.text||''), true);
  },
  async getState(){ return this.fetchJson('/api/state'); },
  async getBattery(){ return this.fetchJson('/api/battery'); },
};

async function loadTab(name) {
  const main = document.getElementById('tab-content');
  if (!main) return;
  const res = await fetch(`/static/tabs/${name}.html`);
  const html = await res.text();
  main.innerHTML = html;
  const panel = main.querySelector('.tab-panel');
  if (panel) panel.classList.add('active');
  const mod = await import(`/static/tabs/${name}.js`);
  if (mod && typeof mod.init === 'function') mod.init(App);
}

function switchTab(name) {
  document.querySelectorAll('.tab').forEach(btn => {
    const active = btn.dataset.tab === name; btn.classList.toggle('active', active); btn.setAttribute('aria-selected', active ? 'true' : 'false');
  });
  return loadTab(name);
}

window.addEventListener('DOMContentLoaded', async () => {
  const al = document.querySelector('.admin-link');
  if(al){
    al.addEventListener('click', async (e)=>{
      e.preventDefault();
      const pin = window.prompt('Inserisci PIN amministratore (6 cifre):','');
      if(!pin) return;
      try{
        const res = await fetch('/api/admin/login', { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({pin}) });
        if(res.ok){ window.location.href = '/admin'; }
        else { const t = await res.text(); App.setStatus('Login fallito: ' + t, true); }
      }catch(err){ App.setStatus('Errore login: ' + err, true); }
    });
  }
  // setup tabs (per ora solo 'status')
  document.querySelectorAll('.tab').forEach(btn => {
    btn.addEventListener('click', () => switchTab(btn.dataset.tab));
  });
  await switchTab('status');
});
