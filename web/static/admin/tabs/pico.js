import { App } from '../js/core.js';

export function init() {
  const f = document.getElementById('pico-form'); if (!f) return; App.clear(f);
  const pc = (App.currentConfig.pico_config ||= {});
  const pl = (App.currentConfig.pico_logging ||= {});
  const h1 = document.createElement('h3'); h1.textContent = 'pico_config'; h1.style.marginTop = '12px'; f.appendChild(h1);
  f.appendChild(App.formRow('pico_config.uart_device', App.inputText('pico_config.uart_device', pc.uart_device || '/dev/ttyAMA1')));
  f.appendChild(App.formRow('pico_config.baudrate', App.inputNumber('pico_config.baudrate', Number(pc.baudrate ?? 115200), 1, undefined, 1)));
  f.appendChild(App.formRow('pico_config.uart_timeout_ms', App.inputNumber('pico_config.uart_timeout_ms', Number(pc.uart_timeout_ms ?? 1000), 0, undefined, 1)));
  const h2 = document.createElement('h3'); h2.textContent = 'pico_logging'; h2.style.marginTop = '12px'; f.appendChild(h2);
  f.appendChild(App.formRow('pico_logging.level', App.inputText('pico_logging.level', pl.level || 'info')));
  f.appendChild(App.formRow('pico_logging.file', App.inputText('pico_logging.file', pl.file || '/opt/smartmower/log/pico_bridge.log')));
  if (pl.data_dir !== undefined) f.appendChild(App.formRow('pico_logging.data_dir', App.inputText('pico_logging.data_dir', pl.data_dir || '')));
  if (pl.max_size_mb !== undefined) f.appendChild(App.formRow('pico_logging.max_size_mb', App.inputNumber('pico_logging.max_size_mb', Number(pl.max_size_mb ?? 10), 0, undefined, 1)));
  if (pl.max_files !== undefined) f.appendChild(App.formRow('pico_logging.max_files', App.inputNumber('pico_logging.max_files', Number(pl.max_files ?? 5), 0, undefined, 1)));
  const btnLoad = document.getElementById('btn-pico-load');
  const btnSave = document.getElementById('btn-pico-save');
  if (btnLoad) btnLoad.addEventListener('click', () => App.loadConfig().then(()=>init()));
  if (btnSave) btnSave.addEventListener('click', () => App.collectAndSave('pico', 'pico-form'));
}
