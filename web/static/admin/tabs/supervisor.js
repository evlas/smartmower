export function init(App) {
  const SERVICES = [
    { name: 'costmap_node', label: 'Navigation: Costmap' },
    { name: 'state_machine_node', label: 'State Machine' },
    { name: 'gps_bridge', label: 'Bridge: GPS' },
    { name: 'pico_bridge', label: 'Bridge: Pico' },
    { name: 'vision_obstacle', label: 'Vision: Obstacle' },
    { name: 'fusion_node', label: 'Fusion' },
    { name: 'safety_supervisor', label: 'Safety Supervisor' }
  ];

  function setSupervisorStatus(msg, isErr=false) {
    const el = document.getElementById('supervisor-status'); if (!el) return;
    el.textContent = msg; el.className = isErr ? 'error' : 'muted';
  }

  async function supervisorCmd(action, service) {
    try {
      setSupervisorStatus(`Invio comando: ${action} → ${service}...`);
      const res = await fetch('/api/supervisor/cmd', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ action, service }) });
      const txt = await res.text();
      if (!res.ok) throw new Error(txt);
      setSupervisorStatus(`OK: ${service} → ${action}`);
    } catch (e) { setSupervisorStatus('Errore: ' + e.message, true); }
  }

  function ensureConfigPaths() {
    if (!App.currentConfig.system) App.currentConfig.system = {};
    if (!App.currentConfig.system.supervisor) App.currentConfig.system.supervisor = {};
    if (!App.currentConfig.system.supervisor.autostart) App.currentConfig.system.supervisor.autostart = {};
  }

  function renderRows() {
    ensureConfigPaths();
    const rows = document.getElementById('svc-rows'); if (!rows) return;
    rows.innerHTML = '';
    const autostart = App.currentConfig.system.supervisor.autostart || {};
    SERVICES.forEach(svc => {
      const row = document.createElement('div');
      row.style.display = 'grid';
      row.style.gridTemplateColumns = '1fr auto auto';
      row.style.gap = '8px';
      row.style.padding = '8px 0';
      row.style.borderBottom = '1px solid #2a3355';

      // nome
      const nameCell = document.createElement('div');
      nameCell.textContent = `${svc.label} (${svc.name})`;
      row.appendChild(nameCell);

      // azioni
      const actions = document.createElement('div');
      actions.className = 'controls';
      const bStart = document.createElement('button'); bStart.textContent = 'Start'; bStart.addEventListener('click', ()=>supervisorCmd('start', svc.name));
      const bStop = document.createElement('button'); bStop.textContent = 'Stop'; bStop.addEventListener('click', ()=>supervisorCmd('stop', svc.name));
      const bRestart = document.createElement('button'); bRestart.textContent = 'Restart'; bRestart.addEventListener('click', ()=>supervisorCmd('restart', svc.name));
      actions.appendChild(bStart); actions.appendChild(bStop); actions.appendChild(bRestart);
      row.appendChild(actions);

      // autostart toggle
      const autoCell = document.createElement('div');
      const cb = document.createElement('input'); cb.type = 'checkbox'; cb.checked = !!autostart[svc.name];
      cb.addEventListener('change', async ()=>{
        ensureConfigPaths();
        App.currentConfig.system.supervisor.autostart[svc.name] = cb.checked;
        try{ await App.saveConfig(); setSupervisorStatus('Autostart aggiornato'); }catch(e){ setSupervisorStatus('Errore salvataggio: ' + e.message, true); }
      });
      autoCell.appendChild(cb);
      row.appendChild(autoCell);

      rows.appendChild(row);
    });
  }

  // assicurati di avere la config
  (async ()=>{
    try {
      if (!App.currentConfig || Object.keys(App.currentConfig).length === 0) {
        await App.loadConfig();
      }
      renderRows();
    } catch(e) {
      setSupervisorStatus('Impossibile caricare configurazione: ' + e.message, true);
    }
  })();
}
