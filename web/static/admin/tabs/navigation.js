import { App } from '../js/core.js';

export function init() {
  const schema = {
    groups: [
      { title: 'Navigazione', fields: [
        { path: 'navigation_config.max_speed_mps', label: 'Velocità max (m/s)', type: 'number', min: 0 },
        { path: 'navigation_config.min_speed_mps', label: 'Velocità min (m/s)', type: 'number', min: 0 },
        { path: 'navigation_config.turn_radius_m', label: 'Raggio di sterzata (m)', type: 'number', min: 0 },
        { path: 'navigation_config.waypoint_spacing_m', label: 'Spaziatura waypoint (m)', type: 'number', min: 0 },
        { path: 'navigation_config.max_waypoints_per_plan', label: 'Max waypoint per plan', type: 'integer', min: 1 }
      ]},
      { title: 'Sicurezza', fields: [
        { path: 'safety_config.emergency_stop_distance_m', label: 'Distanza E-Stop (m)', type: 'number', min: 0 },
        { path: 'safety_config.collision_avoidance_distance_m', label: 'Distanza anti-collisione (m)', type: 'number', min: 0 },
        { path: 'safety_config.perimeter_safety_distance_m', label: 'Distanza sicurezza perimetro (m)', type: 'number', min: 0 },
        { path: 'safety_config.max_slope_degrees', label: 'Pendenza max (°)', type: 'number', min: 0 }
      ]},
      { title: 'Prestazioni', fields: [
        { path: 'performance_config.target_coverage_percentage', label: 'Copertura target (%)', type: 'integer', min: 0, max: 100 },
        { path: 'performance_config.max_planning_time_s', label: 'Tempo max pianificazione (s)', type: 'integer', min: 0 }
      ]}
    ]
  };
  App.renderFormBySchema('navigation-form', schema, 'path_planning_config');
  const btnLoad = document.getElementById('btn-navigation-load');
  const btnSave = document.getElementById('btn-navigation-save');
  if (btnLoad) btnLoad.addEventListener('click', () => App.loadConfig().then(()=>init()));
  if (btnSave) btnSave.addEventListener('click', () => App.collectAndSave('navigation', 'navigation-form'));
}
