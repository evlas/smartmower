import { App } from '../js/core.js';

export function init() {
  App.renderFormBySchema('mqtt-form', { fields: [
    { path: 'broker', label: 'Broker', type: 'string', placeholder: 'localhost' },
    { path: 'port', label: 'Porta', type: 'integer', min: 1, max: 65535, step: 1 },
    { path: 'username', label: 'Username', type: 'string' },
    { path: 'password', label: 'Password', type: 'password' },
    { path: 'root_topic', label: 'Root topic', type: 'string' }
  ]}, 'mqtt');
  const btnLoad = document.getElementById('btn-mqtt-load');
  const btnSave = document.getElementById('btn-mqtt-save');
  if (btnLoad) btnLoad.addEventListener('click', () => App.loadConfig().then(()=>init()));
  if (btnSave) btnSave.addEventListener('click', () => App.collectAndSave('mqtt', 'mqtt-form'));
}
