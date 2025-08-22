import { App } from '../js/core.js';

export function init() {
  const schema = {
    groups: [
      { title: 'Batteria', fields: [ { path: 'battery_type', label: 'Profilo batteria attivo', type: 'enum', options: (cfg)=> Object.keys((cfg.battery_profiles)||{}) } ] },
      { title: 'Dimensioni', fields: [
        { path: 'dimensions.robot_length', label: 'Lunghezza robot (m)', type: 'number', min: 0 },
        { path: 'dimensions.robot_width', label: 'Larghezza robot (m)', type: 'number', min: 0 },
        { path: 'dimensions.robot_height', label: 'Altezza robot (m)', type: 'number', min: 0 },
        { path: 'dimensions.wheel_base', label: 'Interasse ruote (m)', type: 'number', min: 0 },
        { path: 'dimensions.wheel_diameter', label: 'Diametro ruota (m)', type: 'number', min: 0 },
        { path: 'dimensions.wheel_track', label: 'Carreggiata (m)', type: 'number', min: 0 },
        { path: 'dimensions.ground_clearance', label: 'Luce a terra (m)', type: 'number', min: 0 },
        { path: 'dimensions.weight', label: 'Peso (kg)', type: 'number', min: 0 }
      ]},
      { title: 'Lama (Blade)', fields: [
        { path: 'blade.diameter', label: 'Diametro lama (m)', type: 'number', min: 0 },
        { path: 'blade.offset_x', label: 'Offset X lama (m)', type: 'number' },
        { path: 'blade.offset_y', label: 'Offset Y lama (m)', type: 'number' },
        { path: 'blade.cutting_deck_width', label: 'Larghezza piatto (m)', type: 'number', min: 0 }
      ]},
      { title: 'Motori', fields: [
        { path: 'motors.motors_max_rpm', label: 'Ruote: Max RPM', type: 'number', min: 0, step: 1 },
        { path: 'motors.motors_gear_ratio', label: 'Ruote: Rapporto di riduzione', type: 'number', min: 0 },
        { path: 'motors.motors_encoder_pulses_per_rev', label: 'Ruote: Impulsi encoder per giro', type: 'number', min: 0, step: 1 },
        { path: 'motors.blade_motor_max_rpm', label: 'Lama: Max RPM', type: 'number', min: 0, step: 1 },
        { path: 'motors.blade_motor_gear_ratio', label: 'Lama: Rapporto di riduzione', type: 'number', min: 0 },
        { path: 'motors.blade_motor_encoder_pulses_per_rev', label: 'Lama: Impulsi encoder per giro', type: 'number', min: 0, step: 1 }
      ]}
    ]
  };
  App.renderFormBySchema('hardware-form', schema, 'hardware');
  const btnLoad = document.getElementById('btn-hardware-load');
  const btnSave = document.getElementById('btn-hardware-save');
  if (btnLoad) btnLoad.addEventListener('click', () => App.loadConfig().then(()=>init()));
  if (btnSave) btnSave.addEventListener('click', () => App.collectAndSave('hardware', 'hardware-form').then(()=>App.renderBatteryProfileDetails()));
  const hwForm = document.getElementById('hardware-form');
  if (hwForm) {
    hwForm.addEventListener('change', (e) => { const t = e.target; if (t && t.name === 'battery_type') App.renderBatteryProfileDetails(t.value); });
    App.renderBatteryProfileDetails();
  }
}
