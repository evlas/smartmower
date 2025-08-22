// app.js - orchestratore tabs
import { App } from './js/core.js';

const TAB_ORDER = ['mqtt','system','hardware','pico','gps','slam','navigation','vision','supervisor'];

async function loadTab(name) {
  const main = document.getElementById('tab-content');
  if (!main) return;
  // carica html
  const res = await fetch(`/static/admin/tabs/${name}.html`);
  const html = await res.text();
  main.innerHTML = html;
  // assicura visibilità del pannello
  const panel = main.querySelector('.tab-panel');
  if (panel) panel.classList.add('active');
  // importa modulo
  const mod = await import(`/static/admin/tabs/${name}.js`);
  if (mod && typeof mod.init === 'function') mod.init(App);
}

function switchTab(name) {
  document.querySelectorAll('.tab').forEach(btn => {
    const active = btn.dataset.tab === name; btn.classList.toggle('active', active); btn.setAttribute('aria-selected', active ? 'true' : 'false');
  });
  return loadTab(name);
}

window.addEventListener('DOMContentLoaded', async () => {
  // setup nav
  document.querySelectorAll('.tab').forEach(btn => {
    btn.addEventListener('click', () => switchTab(btn.dataset.tab));
  });
  // default: prova a caricare config ma non bloccare UI se fallisce
  try {
    await App.loadConfig();
  } catch (e) {
    // status già impostato da App.loadConfig; prosegui comunque
  } finally {
    await switchTab('mqtt');
  }
});
