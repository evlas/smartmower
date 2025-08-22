// core.js - utilità condivise per Web Admin
export const App = {
  currentConfig: {},
  // --- Helpers path ---
  getByPath(obj, path) {
    return path.split('.').reduce((acc, k) => (acc && k in acc ? acc[k] : undefined), obj);
  },
  setByPath(obj, path, value) {
    const parts = path.split('.');
    let cur = obj;
    for (let i = 0; i < parts.length - 1; i++) {
      const k = parts[i];
      if (!(k in cur) || typeof cur[k] !== 'object') cur[k] = {};
      cur = cur[k];
    }
    cur[parts[parts.length - 1]] = value;
  },
  // --- DOM helpers ---
  clear(el) { while (el.firstChild) el.removeChild(el.firstChild); },
  formRow(labelText, inputEl) {
    const row = document.createElement('div'); row.className = 'form-row';
    const label = document.createElement('label'); label.textContent = labelText;
    row.appendChild(label); row.appendChild(inputEl); return row;
  },
  inputText(name, value = '', placeholder = '') {
    const i = document.createElement('input'); i.type = 'text'; i.name = name; i.value = value ?? ''; i.placeholder = placeholder; return i;
  },
  inputNumber(name, value = 0, min, max, step) {
    const i = document.createElement('input'); i.type = 'number'; i.name = name; i.value = value ?? 0; if(min!=null)i.min=min; if(max!=null)i.max=max; if(step!=null)i.step=step; return i;
  },
  inputPassword(name, value = '', placeholder = '') {
    const i = document.createElement('input'); i.type = 'password'; i.name = name; i.value = value ?? ''; i.placeholder = placeholder; return i;
  },
  inputBoolean(name, value = false) {
    const sel = document.createElement('select'); sel.name = name;
    ['false','true'].forEach(val=>{const o=document.createElement('option');o.value=val;o.textContent=val; if(String(value)===val)o.selected=true; sel.appendChild(o);});
    return sel;
  },
  setStatus(msg, isErr=false) {
    const el = document.getElementById('status'); if (!el) return;
    el.textContent = msg; el.className = isErr ? 'error' : '';
  },
  // --- Schema renderer ---
  renderFormBySchema(containerId, schema, baseKey) {
    const f = document.getElementById(containerId); if (!f) return; this.clear(f);
    const makeField = (field) => {
      const fullPath = field.path;
      const currentVal = this.getByPath(this.currentConfig[baseKey] || {}, fullPath);
      let input;
      if (field.type === 'integer' || field.type === 'number') input = this.inputNumber(fullPath, currentVal, field.min, field.max, field.step);
      else if (field.type === 'password') input = this.inputPassword(fullPath, currentVal || '', field.placeholder || '');
      else if (field.type === 'enum') {
        input = document.createElement('select'); input.name = fullPath;
        const opts = (typeof field.options === 'function') ? field.options(this.currentConfig) : (field.options || []);
        opts.forEach(opt => { const o=document.createElement('option'); o.value=String(opt); o.textContent=String(opt); if(String(currentVal)===String(opt)) o.selected=true; input.appendChild(o); });
      } else if (field.type === 'boolean') input = this.inputBoolean(fullPath, !!currentVal);
      else input = this.inputText(fullPath, currentVal || '', field.placeholder || '');
      f.appendChild(this.formRow(field.label || fullPath, input));
    };
    if (schema.fields) schema.fields.forEach(makeField);
    if (schema.groups) schema.groups.forEach(g => {
      if (g.title) { const h = document.createElement('h3'); h.textContent = g.title; h.style.marginTop = '12px'; f.appendChild(h); }
      (g.fields || []).forEach(makeField);
    });
  },
  // --- Config ops ---
  async loadConfig() {
    this.setStatus('Caricamento...');
    try {
      const res = await fetch('/api/config');
      const txt = await res.text();
      if (!res.ok) throw new Error(txt);
      const payload = JSON.parse(txt);
      this.currentConfig = payload.data !== undefined ? payload.data : payload;
      this.setStatus('Config caricata');
      return this.currentConfig;
    } catch (e) {
      this.setStatus('Errore caricamento: ' + e.message, true);
      throw e;
    }
  },
  async saveConfig() {
    this.setStatus('Salvataggio...');
    try {
      const res = await fetch('/api/config', {
        method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ value: this.currentConfig })
      });
      const txt = await res.text();
      if (!res.ok) throw new Error(txt);
      this.setStatus('Config salvata: ' + txt);
    } catch (e) {
      this.setStatus('Errore salvataggio: ' + e.message, true);
      throw e;
    }
  },
  collectAndSave(sectionName, formId) {
    const formEl = document.getElementById(formId); if (!formEl) return;
    const keyMap = { mqtt:'mqtt', system:'system', hardware:'hardware', slam:'slam_config', navigation:'path_planning_config', vision:'vision_config', pico:'pico', gps:'gps' };
    const baseKey = keyMap[sectionName] || sectionName;
    const inputs = formEl.querySelectorAll('input,select,textarea');
    inputs.forEach(el => {
      const localPath = el.name; if (!localPath) return;
      let val = el.type === 'number' ? (el.value === '' ? null : Number(el.value)) : el.value;
      if (el.tagName === 'SELECT' && (el.value === 'true' || el.value === 'false')) val = (el.value === 'true');
      if (!(baseKey in this.currentConfig) || typeof this.currentConfig[baseKey] !== 'object') this.currentConfig[baseKey] = {};
      this.setByPath(this.currentConfig[baseKey], localPath, val);
    });
    const ta = document.getElementById('config-json'); if (ta) ta.value = JSON.stringify(this.currentConfig, null, 2);
    return this.saveConfig();
  },
  // Battery profile details (used by Hardware tab)
  renderBatteryProfileDetails(selectedType) {
    const hwForm = document.getElementById('hardware-form'); if (!hwForm) return;
    const select = hwForm.querySelector('select[name="battery_type"]'); if (!select) return;
    let row = select.closest('.form-row') || select.parentElement; let box = row ? row.nextElementSibling : null;
    if (!box || !box.classList || !box.classList.contains('battery-profile-details')) {
      box = document.createElement('div'); box.className = 'battery-profile-details readonly'; box.style.margin = '8px 0 12px 0';
      if (row && row.parentNode) row.parentNode.insertBefore(box, row.nextSibling); else hwForm.appendChild(box);
    } else { box.innerHTML = ''; }
    const profiles = this.currentConfig.battery_profiles || {}; const hw = this.currentConfig.hardware || {}; const type = selectedType || (hw.battery_type || hw.battery || '');
    const prof = profiles[type]; const title = document.createElement('div'); title.className = 'muted'; title.textContent = type ? `Profilo: ${type}` : 'Nessun profilo selezionato'; box.appendChild(title);
    if (!prof) { const none = document.createElement('div'); none.textContent = '—'; none.style.marginTop = '6px'; box.appendChild(none); return; }
    const editable = document.createElement('div'); editable.className = 'battery-editable'; editable.style.marginTop = '6px';
    const edTitle = document.createElement('div'); edTitle.textContent = 'Modificabili (salvati nel profilo batteria)'; edTitle.style.fontWeight = '600'; edTitle.style.margin = '6px 0'; editable.appendChild(edTitle);
    const capInput = this.inputNumber('capacity_ah', Number(prof.capacity_ah ?? 0)); capInput.dataset.batteryEdit = '1'; const capRow = this.formRow('capacity_ah', capInput); editable.appendChild(capRow);
    const chargingKeys = [
      ['charging.full_voltage_per_cell', Number(prof?.charging?.full_voltage_per_cell ?? 0)],
      ['charging.trickle_current_ma', Number(prof?.charging?.trickle_current_ma ?? 0)],
      ['charging.stable_time_seconds', Number(prof?.charging?.stable_time_seconds ?? 0)],
      ['charging.current_threshold_ma', Number(prof?.charging?.current_threshold_ma ?? 0)]
    ];
    chargingKeys.forEach(([name, val]) => { const inp = this.inputNumber(name, val); inp.dataset.batteryEdit = '1'; const row = this.formRow(name, inp); editable.appendChild(row); });
    box.appendChild(editable);
  },
};
