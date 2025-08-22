import { App } from '../js/core.js';

export function init() {
  const f = document.getElementById('gps-form'); if (!f) return; App.clear(f);
  const gc = (App.currentConfig.gps_config ||= {});
  const gl = (App.currentConfig.gps_logging ||= {});
  const h1 = document.createElement('h3'); h1.textContent = 'gps_config'; h1.style.marginTop = '12px'; f.appendChild(h1);
  f.appendChild(App.formRow('gps_config.uart_device', App.inputText('gps_config.uart_device', gc.uart_device || '/dev/ttyAMA2')));
  f.appendChild(App.formRow('gps_config.baudrate', App.inputNumber('gps_config.baudrate', Number(gc.baudrate ?? 115200), 1, undefined, 1)));
  f.appendChild(App.formRow('gps_config.uart_timeout_ms', App.inputNumber('gps_config.uart_timeout_ms', Number(gc.uart_timeout_ms ?? 1000), 0, undefined, 1)));
  f.appendChild(App.formRow('gps_config.protocol', App.inputText('gps_config.protocol', gc.protocol || 'nmea')));
  f.appendChild(App.formRow('gps_config.max_satellites', App.inputNumber('gps_config.max_satellites', Number(gc.max_satellites ?? 24), 0, undefined, 1)));
  const h2 = document.createElement('h3'); h2.textContent = 'gps_logging'; h2.style.marginTop = '12px'; f.appendChild(h2);
  f.appendChild(App.formRow('gps_logging.level', App.inputText('gps_logging.level', gl.level || 'info')));
  f.appendChild(App.formRow('gps_logging.file', App.inputText('gps_logging.file', gl.file || '/opt/smartmower/log/gps_bridge.log')));
  if (gl.data_dir !== undefined) f.appendChild(App.formRow('gps_logging.data_dir', App.inputText('gps_logging.data_dir', gl.data_dir || '')));
  const btnLoad = document.getElementById('btn-gps-load');
  const btnSave = document.getElementById('btn-gps-save');
  if (btnLoad) btnLoad.addEventListener('click', () => App.loadConfig().then(()=>init()));
  if (btnSave) btnSave.addEventListener('click', () => App.collectAndSave('gps', 'gps-form'));
}
