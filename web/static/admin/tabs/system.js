import { App } from '../js/core.js';

export function init() {
  const schema = { fields: [
    { path: 'safety.emergency_stop_enabled', label: 'Emergency Stop Abilitato', type: 'boolean' },
    { path: 'safety.lift_sensor_enabled', label: 'Lift Sensor Abilitato', type: 'boolean' },
    { path: 'safety.theft_alarm_enabled', label: 'Allarme Furto Abilitato', type: 'boolean' },
    { path: 'safety.geofence_enabled', label: 'Geofence Abilitato', type: 'boolean' }
  ]};
  App.renderFormBySchema('system-form', schema, 'system');
  const btnLoad = document.getElementById('btn-system-load');
  const btnSave = document.getElementById('btn-system-save');
  if (btnLoad) btnLoad.addEventListener('click', () => App.loadConfig().then(()=>init()));
  if (btnSave) btnSave.addEventListener('click', () => App.collectAndSave('system', 'system-form'));
}
