import { App } from '../js/core.js';

export function init() {
  const schema = {
    groups: [
      { title: 'Camera', fields: [
        { path: 'camera.width', label: 'Larghezza', type: 'integer', min: 1 },
        { path: 'camera.height', label: 'Altezza', type: 'integer', min: 1 },
        { path: 'camera.fps', label: 'FPS', type: 'integer', min: 1 },
        { path: 'camera.rotation', label: 'Rotazione (deg)', type: 'integer', min: 0 },
        { path: 'camera.flip', label: 'Flip', type: 'enum', options: ['none','horizontal','vertical','both'] }
      ]},
      { title: 'VO Features', fields: [
        { path: 'odometry.features.max_corners', label: 'Max corners', type: 'integer', min: 1 },
        { path: 'odometry.features.quality_level', label: 'Quality level', type: 'number', min: 0, max: 1, step: 0.01 },
        { path: 'odometry.features.min_distance', label: 'Min distance', type: 'integer', min: 1 },
        { path: 'odometry.features.block_size', label: 'Block size', type: 'integer', min: 1 }
      ]},
      { title: 'Obstacle detection', fields: [
        { path: 'detection.obstacle_detection.parameters.min_area_ratio', label: 'Min area ratio', type: 'number', min: 0, max: 1, step: 0.01 }
      ]}
    ]
  };
  App.renderFormBySchema('vision-form', schema, 'vision_config');
  const btnLoad = document.getElementById('btn-vision-load');
  const btnSave = document.getElementById('btn-vision-save');
  if (btnLoad) btnLoad.addEventListener('click', () => App.loadConfig().then(()=>init()));
  if (btnSave) btnSave.addEventListener('click', () => App.collectAndSave('vision', 'vision-form'));
}
