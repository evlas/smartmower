import { App } from '../js/core.js';

export function init() {
  const schema = {
    groups: [
      { title: 'SLAM Core', fields: [
        { path: 'slam.update_rate_hz', label: 'Update rate (Hz)', type: 'number', min: 1 },
        { path: 'slam.publish_rate_hz', label: 'Publish rate (Hz)', type: 'number', min: 1 },
        { path: 'slam.map_update_rate_hz', label: 'Map update rate (Hz)', type: 'number', min: 0 },
        { path: 'slam.save_map_interval_s', label: 'Save map interval (s)', type: 'integer', min: 0 }
      ]},
      { title: 'Ostacoli', fields: [
        { path: 'obstacle_detection.min_distance', label: 'Min distanza (m)', type: 'number', min: 0 },
        { path: 'obstacle_detection.max_distance', label: 'Max distanza (m)', type: 'number', min: 0 },
        { path: 'obstacle_detection.filter_window_size', label: 'Finestra filtro', type: 'integer', min: 1 },
        { path: 'obstacle_detection.obstacle_timeout', label: 'Timeout ostacolo (s)', type: 'number', min: 0 }
      ]},
      { title: 'Sensor Fusion (abilitazioni)', fields: [
        { path: 'sensor_fusion.use_gps', label: 'Usa GPS', type: 'boolean' },
        { path: 'sensor_fusion.use_odometry', label: 'Usa Odometry', type: 'boolean' },
        { path: 'sensor_fusion.use_visual_odometry', label: 'Usa Visual Odometry', type: 'boolean' },
        { path: 'sensor_fusion.use_ultrasonic', label: 'Usa Ultrasonic', type: 'boolean' },
        { path: 'sensor_fusion.use_imu', label: 'Usa IMU', type: 'boolean' },
        { path: 'sensor_fusion.use_magnetometer', label: 'Usa Magnetometer', type: 'boolean' },
        { path: 'sensor_fusion.use_bumper', label: 'Usa Bumper', type: 'boolean' }
      ]}
    ]
  };
  App.renderFormBySchema('slam-form', schema, 'slam_config');
  const btnLoad = document.getElementById('btn-slam-load');
  const btnSave = document.getElementById('btn-slam-save');
  if (btnLoad) btnLoad.addEventListener('click', () => App.loadConfig().then(()=>init()));
  if (btnSave) btnSave.addEventListener('click', () => App.collectAndSave('slam', 'slam-form'));
}
