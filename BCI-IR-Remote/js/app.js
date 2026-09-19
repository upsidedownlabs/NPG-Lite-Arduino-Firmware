/* UI layer. Owns the DOM and the local mirror of the board state. */

(() => {

  const $ = id => document.getElementById(id);

  const el = {
    conn: $('btn-conn'), connTxt: $('conn-txt'),
    theme: $('btn-theme'), info: $('btn-info'), settings: $('btn-settings'), wipe: $('btn-wipe'),

    lock: $('btn-lock'), lockIco: document.querySelector('#btn-lock .ico'),
    notch: $('notch'), controls: $('controls'),

    bars: $('bars'), cols: document.querySelector('.cols'),
    profiles: $('profiles'), list: $('list'),
    cmdTtl: $('cmd-ttl'), cmdEmpty: $('cmd-empty'),
    ftr: $('ftr'), activeNm: $('active-nm'), fire: $('btn-fire'), toast: $('toast'),

    listenBar: $('listen-bar'), listenSec: $('listen-time'), listenX: $('listen-cancel'),
    capInput: $('cap-input'), capCount: $('cap-count'), capTtl: $('cap-ttl'),
    capDup: $('cap-dup'), capMeta: $('cap-meta'), capActions: $('cap-actions'),
    renTtl: $('ren-ttl'), renInput: $('ren-input'), renCount: $('ren-count'), renSave: $('ren-save'),
    cfmTtl: $('cfm-ttl'), cfmBody: $('cfm-body'), cfmOk: $('cfm-ok'), cfmNo: $('cfm-no'),
  };

  const HOME = BLE.SCREEN.HOME;
  const MAX_CHANNELS = 6;

  // These mirror MAX_PROFILES and MAX_COMMANDS in the firmware. The board
  // just refuses when it is full, so the app checks first and says why.
  const MAX_PROFILES = 5;
  const MAX_COMMANDS = 10;

  let screen      = HOME;        // what the board says it is showing
  let profiles    = new Map();   // id -> { name, count }
  let commands    = new Map();   // id -> { name }
  let openProfile = -1;
  let homeCursor  = -1;          // centre profile, the board moves it too
  let activeId    = -1;          // centre command inside the open profile
  let locked      = true;        // edits are off until you unlock
  let loadingCommands = false;   // a profile's commands are still arriving

  let capture = null, renameKind = 'command', renameId = -1;
  let onConfirm = null, onDecline = null;
  let loading = false, listTimer = null, toastTimer = null, listenTimer = null;

  /* theme ---------------------------------------------------- */

  const savedTheme = localStorage.getItem('npg-theme');
  if (savedTheme) document.documentElement.dataset.theme = savedTheme;

  el.theme.addEventListener('click', () => {
    const next = themeIsDark() ? 'light' : 'dark';
    document.documentElement.dataset.theme = next;
    localStorage.setItem('npg-theme', next);
  });

  function themeIsDark() {
    const set = document.documentElement.dataset.theme;
    if (set) return set === 'dark';
    return matchMedia('(prefers-color-scheme: dark)').matches;
  }

  /* settings -------------------------------------------------- */

  // Channels, thresholds and the mapping live in this browser and are pushed
  // to the board on connect. The board boots on its own defaults, so it still
  // works standalone.
  const TUNE_KEY = 'npg-tuning';
  const MAP_KEY  = 'npg-mapping';
  const CHAN_KEY = 'npg-channels';

  const DEFAULT_TUNING = {
    muscleThreshold: 75, muscleRelease: 60, focusThreshold: 10, blinkThreshold: 50,
  };
  const DEFAULT_MAPPING = {
    [BLE.ACTION.SCROLL_DOWN]: { channel: 0, gesture: BLE.GESTURE.CLENCH_HOLD },
    [BLE.ACTION.SCROLL_UP]:   { channel: 0, gesture: BLE.GESTURE.CLENCH },
    [BLE.ACTION.FIRE]:        { channel: 0, gesture: BLE.GESTURE.FOCUS },
    [BLE.ACTION.HOME]:        { channel: 0, gesture: BLE.GESTURE.TRIPLE_BLINK },
  };

  // Earlier builds stored a single set of thresholds and a flat mapping, so
  // anything that does not match the current shape is dropped rather than
  // half read. Losing a tuning is cheap; throwing on load is not.
  const storedTuning   = load(TUNE_KEY,  v => Array.isArray(v) && v.length === MAX_CHANNELS
                                              && v.every(t => t && 'muscleThreshold' in t));
  const storedMapping  = load(MAP_KEY,   v => v && Object.values(v).every(b => b && 'channel' in b));
  const storedChannels = load(CHAN_KEY,  v => v && Array.isArray(v.filters) && v.filters.length === MAX_CHANNELS);

  // one set of thresholds per channel
  let tuning = storedTuning ||
    Array.from({ length: MAX_CHANNELS }, () => ({ ...DEFAULT_TUNING }));
  let mapping = storedMapping || structuredClone(DEFAULT_MAPPING);
  let channels = storedChannels || {
    notchHz: 50,
    filters: [BLE.FILTER.EEG, 0, 0, 0, 0, 0],
  };

  // Whether this browser has settings of its own worth pushing. It starts from
  // what was stored and becomes true the moment you change something, so a
  // board that reboots gets your setup back rather than overwriting it.
  let own = {
    channels: !!storedChannels,
    mapping:  !!storedMapping,
    tuning:   !!storedTuning,
  };

  let available = 3;          // how many channels this playmate has
  let settingsPushed = false; // this browser's settings have reached the board
  let configSettled  = false; // the opening config exchange is done

  function load(key, isValid) {
    try {
      const value = JSON.parse(localStorage.getItem(key));
      if (value && isValid(value)) return value;
    } catch {}
    localStorage.removeItem(key);
    return null;
  }
  function store(key, value) {
    try { localStorage.setItem(key, JSON.stringify(value)); } catch {}
  }

  // Focus is a percentage, the others are raw envelope levels, so their bars
  // need different full-scale values. 500 puts a resting level and a
  // deliberate gesture in the readable part of the track.
  const SIGNAL_SCALE = { muscle: 500, focus: 100, blink: 500 };
  const SIGNAL_STEP  = { muscle: 5, focus: 0.5, blink: 5 };

  /* helpers -------------------------------------------------- */

  function toast(message, isError = false) {
    el.toast.textContent = message;
    el.toast.classList.toggle('err', isError);
    el.toast.classList.add('show');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => el.toast.classList.remove('show'), 2600);
  }

  // No label any more, so what would have been the status line becomes the
  // connect button's tooltip.
  const setStatus = text => { el.conn.title = text; };
  const openModal = id => $(id).classList.add('show');
  const closeModal = id => $(id).classList.remove('show');

  async function guard(fn, errorText) {
    try { await fn(); }
    catch (err) { toast(errorText || err.message, true); }
  }

  /* rendering ------------------------------------------------ */

  // A slot is only a remote once it has been named, and only then does it
  // have commands to hold.
  const usable = id => profiles.get(id) ? profiles.get(id).named === true : false;
  const validProfile = () => usable(openProfile);

  // The commands panel only ever goes empty for lack of any remote to show.
  // Whichever one sorts first by id is "the" remote until you pick another,
  // not necessarily the one literally named "Remote 1".
  function openFirstIfNone() {
    if (validProfile()) return;
    const first = [...profiles.keys()].sort((a, b) => a - b).find(usable);
    if (first !== undefined) guard(() => BLE.openProfile(first));
  }

  function render() {
    if (loading) return;

    const connected = BLE.isConnected();
    el.lock.classList.toggle('on', !locked);
    el.lock.title = locked ? 'Unlock to configure' : 'Lock';
    el.lockIco.replaceChildren(Icons.svg(locked ? 'lock' : 'lock-open'));
    el.cols.classList.toggle('locked', locked);
    // the notch restarts sampling, so it only moves while unlocked
    [...el.notch.children].forEach(b => { b.disabled = locked || !connected; });

    renderControls();
    renderProfiles();
    renderCommands();
    explainLock();
  }

  const LOCK_HINT = 'Click the unlock button to edit';

  // Anything switched off because the page is locked says so on hover, rather
  // than looking broken.
  function explainLock() {
    document.querySelectorAll('.ctl-row select, .switch button, .add-btn, .row-actions button')
      .forEach(node => {
        if (locked && (node.disabled || node.classList.contains('locked-out'))) node.title = LOCK_HINT;
        else if (node.title === LOCK_HINT) node.title = '';
      });
  }

  // A click on an edit, delete or add button while locked says why nothing
  // happened and points at the way out, rather than just sitting there dim.
  function lockedToast() {
    toast('Unlock edits by clicking on the unlock button at the top', true);
    el.lock.classList.remove('hint');
    void el.lock.offsetWidth;
    el.lock.classList.add('hint');
    setTimeout(() => el.lock.classList.remove('hint'), 600);
  }

  /* controls --------------------------------------------------
     One row per control: its channel, that channel's filter, the gesture
     that drives it, and the level that gesture is measured against. */

  const CONTROL_NAMES = [
    [BLE.ACTION.SCROLL_DOWN, 'Forward',       'move-right'],
    [BLE.ACTION.SCROLL_UP,   'Backward',      'move-left'],
    [BLE.ACTION.FIRE,        'Shoot IR',      'send-horizontal'],
    [BLE.ACTION.HOME,        'Switch Remote', 'house'],
  ];

  const FILTER_NAMES = [
    [BLE.FILTER.EMG, 'EMG'],
    [BLE.FILTER.EEG, 'EEG'],
    [BLE.FILTER.EOG, 'EOG'],
  ];

  const controlName = action => CONTROL_NAMES.find(c => c[0] === action)[1];

  let barCells = [];
  let controlRows = new Map();   // action -> its row, for the fired highlight

  // A channel's filter belongs to whichever control claimed it first. Any
  // other control on the same channel shows it and cannot change it.
  function filterOwner(channel) {
    const hit = CONTROL_NAMES.find(([action]) => mapping[action].channel === channel);
    return hit ? hit[0] : null;
  }

  function renderControls() {
    const config = channels;
    renderNotch(config.notchHz);
    el.controls.replaceChildren();
    barCells = [];
    controlRows = new Map();

    CONTROL_NAMES.forEach(([action, label, icon]) => {
      const bind = mapping[action];
      const channel = bind.channel < MAX_CHANNELS ? bind.channel : -1;
      const filter = channel < 0 ? BLE.FILTER.OFF : config.filters[channel];

      const row = document.createElement('div');
      row.className = 'ctl-row';

      const name = document.createElement('span');
      name.className = 'ctl-nm';
      const text = document.createElement('span');
      text.textContent = label;
      name.append(text, Icons.svg(icon));

      row.append(name,
        channelSelect(action, channel),
        filterSelect(action, channel, filter),
        triggerSelect(action, channel, filter),
        levelFor(action, bind, channel));
      controlRows.set(action, row);
      el.controls.appendChild(row);
    });
  }

  function channelSelect(action, channel) {
    const select = document.createElement('select');
    select.className = 'select sel-ch';
    select.disabled = locked;
    addPlaceholder(select, roomy() ? 'Channel' : 'Ch');
    for (let ch = 0; ch < available; ch++) addOption(select, ch, 'Ch' + (ch + 1));
    select.value = channel < 0 ? '' : String(channel);

    select.addEventListener('change', () => {
      const next = select.value === '' ? -1 : Number(select.value);
      if (next < 0) { clearControl(action); return; }
      // a channel nothing was using needs a filter to start from
      if (channels.filters[next] === BLE.FILTER.OFF) {
        commitChannels(next, freeFilter(channels));
      }
      mapping[action] = { channel: next, gesture: BLE.GESTURE.NONE };
      commitMapping();
      render();
    });
    return select;
  }

  function filterSelect(action, channel, filter) {
    const select = document.createElement('select');
    select.className = 'select sel-exg';
    addPlaceholder(select, 'EXG');
    FILTER_NAMES.forEach(([value, text]) => addOption(select, value, text));
    select.value = channel < 0 ? '' : String(filter);

    const owner = channel < 0 ? null : filterOwner(channel);
    const shared = owner !== null && owner !== action;
    select.disabled = locked || channel < 0 || shared;
    if (shared) select.title = 'Set on ' + controlName(owner);

    select.addEventListener('change', () => {
      commitChannels(channel, Number(select.value));
      // the gesture it was watching may not exist under the new filter
      mapping[action] = { channel, gesture: BLE.GESTURE.NONE };
      commitMapping();
      render();
    });
    return select;
  }

  function triggerSelect(action, channel, filter) {
    const select = document.createElement('select');
    select.className = 'select sel-trig';
    select.disabled = locked || channel < 0 || filter === BLE.FILTER.OFF;
    addOption(select, '', 'None');
    (BLE.GESTURES_FOR[filter] || []).forEach(([gesture, label]) =>
      addOption(select, gesture, label));

    select.value = mapping[action].gesture ? String(mapping[action].gesture) : '';

    select.addEventListener('change', () => {
      const gesture = select.value === '' ? BLE.GESTURE.NONE : Number(select.value);
      if (gesture === BLE.GESTURE.NONE) {
        mapping[action] = { channel, gesture };
        commitMapping();
        render();
        return;
      }

      // one gesture drives one control, so say so before taking it away
      const holder = CONTROL_NAMES.find(([other]) => other !== action &&
        mapping[other].channel === channel && mapping[other].gesture === gesture);

      if (!holder) {
        mapping[action] = { channel, gesture };
        commitMapping();
        render();
        return;
      }

      confirm('Already assigned',
        'That gesture drives ' + holder[1] + '. Reassign it to ' + controlName(action) + '?',
        'Reassign',
        () => {
          mapping[holder[0]] = { channel: mapping[holder[0]].channel, gesture: BLE.GESTURE.NONE };
          mapping[action] = { channel, gesture };
          commitMapping();
          render();
        },
        () => render());
    });
    return select;
  }

  // The level this control watches, with its threshold and what it last
  // reported. A control with nothing assigned keeps an empty slot so the
  // rows still line up.
  function levelFor(action, bind, channel) {
    const signal = channel < 0 ? null : BLE.signalFor(bind.gesture);
    const wrap = document.createElement('div');
    wrap.className = 'ctl-level';

    const scale = signal ? SIGNAL_SCALE[signal] : 100;
    const value = signal ? thresholdFor(channel, signal) : 0;

    const readout = document.createElement('span');
    readout.className = 'bar-val';
    readout.textContent = signal ? format(signal, value) : '0';

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.className = 'level';
    slider.min = 0;
    slider.max = scale;
    slider.step = signal ? SIGNAL_STEP[signal] : 1;
    slider.value = value;
    slider.disabled = !signal;
    if (signal) {
      slider.addEventListener('input', () => {
        setThreshold(channel, signal, Number(slider.value));
        syncThresholds(channel, signal);
      });
    }

    wrap.append(readout, slider);
    const cell = { channel, signal, action, slider, readout, scale, shown: 0, target: 0 };
    barCells.push(cell);
    paint(cell);
    return wrap;
  }

  function addOption(select, value, text) {
    const option = document.createElement('option');
    option.value = value;
    option.textContent = text;
    select.appendChild(option);
    return option;
  }

  // A placeholder names the picker, it is not something you can choose.
  function addPlaceholder(select, text) {
    const option = addOption(select, '', text);
    option.disabled = true;
    option.hidden = true;
  }

  // The full word when the row has space for it, the short one when it does
  // not. Rendered text, so it has to be decided here rather than in CSS.
  const roomy = () => window.innerWidth >= 1100;

  function clearControl(action) {
    mapping[action] = { channel: BLE.NO_CHANNEL, gesture: BLE.GESTURE.NONE };
    commitMapping();
    render();
  }

  function commitMapping() {
    own.mapping = true;
    store(MAP_KEY, mapping);
    if (BLE.isConnected()) guard(() => BLE.setMapping(mapping));
  }

  // EEG and EOG are each one channel's job. EMG can be on as many as you like.
  function freeFilter(config) {
    return config.filters.includes(BLE.FILTER.EEG) ? BLE.FILTER.EMG : BLE.FILTER.EEG;
  }

  // Every change lands straight away. Restarting the ADC costs a few
  // milliseconds and the filters settle before triggers resume, so there is
  // nothing to batch behind an Apply button.
  function commitChannels(channel, filter) {
    if (channel !== null) channels.filters[channel] = filter;
    own.channels = true;
    store(CHAN_KEY, channels);
    if (BLE.isConnected()) guard(() => BLE.setChannels(channels.notchHz, channels.filters));
  }

  function renderNotch(hz) {
    [...el.notch.children].forEach(b => b.classList.toggle('on', Number(b.dataset.hz) === hz));
  }

  /* profiles and commands ------------------------------------- */

  function renderProfiles() {
    const ids = [...profiles.keys()].sort((a, b) => a - b);
    el.profiles.replaceChildren();

    ids.forEach(id => {
      const profile = profiles.get(id);
      const named = profile.named;
      // The board leaves an unnamed slot's name empty, so a placeholder
      // stands in for it here rather than in firmware.
      const displayName = profile.name || ('Remote ' + (id + 1));

      const li = document.createElement('li');
      // Switching remotes opens the new one immediately, so the full
      // highlight on the open remote is the only selection state there is.
      li.className = 'row row-sm' + (id === openProfile ? ' active' : '') +
        (named ? '' : ' blank');
      li.dataset.id = id;
      li.tabIndex = 0;
      li.title = named ? displayName
        : 'Click on add button in edit mode to assign a remote profile';

      const name = document.createElement('span');
      name.className = 'row-nm';
      name.textContent = displayName;
      li.appendChild(name);

      if (named) {
        const badge = document.createElement('span');
        badge.className = 'badge';
        badge.textContent = profile.count + '/' + MAX_COMMANDS;
        li.appendChild(badge);
      }

      const add = document.createElement('button');
      add.className = 'icon-btn icon-btn-sm add-btn';
      add.disabled = named;
      add.classList.toggle('locked-out', locked && !named);
      add.title = 'Set up this remote';
      add.setAttribute('aria-label', 'Set up ' + displayName);
      add.appendChild(Icons.svg('plus'));
      add.addEventListener('click', e => {
        e.stopPropagation();
        if (locked) { lockedToast(); return; }
        openRename('profile', id);
      });
      if (named) add.hidden = true;
      li.appendChild(add);

      li.appendChild(rowActions(displayName,
        () => openRename('profile', id),
        () => confirmDeleteProfile(id), !named, !named));

      const open = () => {
        if (!named) { toast('Set this remote up first', true); return; }
        if (id !== openProfile) guard(() => BLE.openProfile(id));
      };
      li.addEventListener('click', open);
      li.addEventListener('keydown', e => {
        if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); open(); }
      });
      el.profiles.appendChild(li);
    });
  }

  // Ten fixed slots, two across. An empty slot offers to record into itself;
  // a filled one can be sent, renamed or cleared.
  function renderCommands() {
    el.list.replaceChildren();
    if (!validProfile()) {
      el.cmdEmpty.hidden = false;
      el.cmdEmpty.textContent = !BLE.isConnected() ? 'Connect your NPG-Lite device.'
        : [...profiles.values()].some(r => r.named)
          ? 'Pick a remote on the left.'
          : 'Set up a remote on the left to add IR commands.';
      return;
    }

    // The map was just cleared for the remote that is opening, so every slot
    // would briefly guess "Command N" - even ones about to turn out to be
    // "Power" or "Mute" - until the real list streams in. Better to show
    // nothing for that one beat than a name that is about to change.
    if (loadingCommands) {
      el.cmdEmpty.hidden = false;
      el.cmdEmpty.textContent = 'Loading commands…';
      return;
    }
    el.cmdEmpty.hidden = true;

    for (let id = 0; id < MAX_COMMANDS; id++) {
      // The board only fills in "Command N" for a slot the moment its remote
      // is created. An unrecorded slot's name goes back to empty on every
      // reboot after that, so the placeholder has to be filled in here too,
      // not just when the slot is missing from the map entirely.
      const entry = commands.get(id);
      const cmd = entry && entry.name ? entry
        : { name: 'Command ' + (id + 1), recorded: entry ? entry.recorded : false };
      const li = document.createElement('li');
      li.className = 'row' + (id === activeId ? ' active' : '') +
        (cmd.recorded ? '' : ' blank');
      li.dataset.id = id;
      li.tabIndex = 0;

      const radio = document.createElement('span');
      radio.className = 'radio';

      const name = document.createElement('span');
      name.className = 'row-nm';
      name.textContent = cmd.name;
      li.append(radio, name);
      li.title = cmd.recorded ? cmd.name
        : 'Click on add button in edit mode to assign an IR command';

      li.appendChild(rowActions(cmd.name,
        () => openRename('command', id),
        () => confirmDeleteCommand(id), !cmd.recorded, !cmd.recorded));

      // fire (or the add/reassign button that records into this slot) sits
      // rightmost, the one thing on the row you reach for over and over
      if (cmd.recorded && locked) {
        const fire = document.createElement('button');
        fire.className = 'icon-btn icon-btn-sm fire-btn';
        fire.title = 'Send ' + cmd.name;
        fire.setAttribute('aria-label', 'Send ' + cmd.name);
        fire.appendChild(Icons.svg('send-horizontal'));
        fire.addEventListener('click', e => {
          e.stopPropagation();
          setActive(id);
          // the board's own ack is a full round trip behind the IR pulse it
          // already sent, so the flash plays on the click, not the reply
          showFire(id);
          guard(() => BLE.fire(id));
        });
        li.appendChild(fire);
      } else if (cmd.recorded) {
        // unlocked is for setting up, not firing, so a recorded slot offers
        // to be re-recorded straight over rather than needing a delete first
        const reassign = document.createElement('button');
        reassign.className = 'icon-btn icon-btn-sm';
        reassign.title = 'Re-record ' + cmd.name;
        reassign.setAttribute('aria-label', 'Re-record ' + cmd.name);
        reassign.appendChild(Icons.svg('refresh-cw'));
        reassign.addEventListener('click', e => { e.stopPropagation(); recordInto(id); });
        li.appendChild(reassign);
      } else {
        const add = document.createElement('button');
        add.className = 'icon-btn icon-btn-sm add-btn';
        add.classList.toggle('locked-out', locked);
        add.title = 'Record into ' + cmd.name;
        add.setAttribute('aria-label', 'Record into ' + cmd.name);
        add.appendChild(Icons.svg('plus'));
        add.addEventListener('click', e => {
          e.stopPropagation();
          if (locked) { lockedToast(); return; }
          recordInto(id);
        });
        li.appendChild(add);
      }

      // an empty slot is not a thing you can point at yet
      const act = () => {
        if (!cmd.recorded) { toast('No IR code added. Click on add button to record', true); return; }
        setActive(id);
      };
      li.addEventListener('click', act);
      li.addEventListener('keydown', e => {
        if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); act(); }
      });
      el.list.appendChild(li);
    }
  }

  // Which slot a recording is destined for, so the capture goes where it was
  // asked for rather than the first free place.
  let recordTarget = -1;

  function recordInto(id) {
    recordTarget = id;
    guard(() => BLE.startRec());
  }

  // Always laid out, so locking never changes the shape of a row. Locked
  // simply puts them out of reach.
  function rowActions(name, onRename, onDelete, noDelete = false, noRename = false) {
    const actions = document.createElement('span');
    actions.className = 'row-actions';

    // noRename/noDelete are structural (nothing there to act on) and stay a
    // real disabled button. Locked is temporary, so it stays clickable and
    // says why, rather than going silent.
    const edit = document.createElement('button');
    edit.className = 'icon-btn icon-btn-sm';
    edit.title = 'Rename';
    edit.setAttribute('aria-label', 'Rename ' + name);
    edit.appendChild(Icons.svg('pencil'));
    edit.disabled = noRename;
    edit.classList.toggle('locked-out', locked && !noRename);
    edit.addEventListener('click', e => {
      e.stopPropagation();
      if (locked) { lockedToast(); return; }
      onRename();
    });

    const del = document.createElement('button');
    del.className = 'icon-btn icon-btn-sm danger';
    del.title = 'Delete';
    del.setAttribute('aria-label', 'Delete ' + name);
    del.appendChild(Icons.svg('trash'));
    del.disabled = noDelete;
    del.classList.toggle('locked-out', locked && !noDelete);
    del.addEventListener('click', e => {
      e.stopPropagation();
      if (locked) { lockedToast(); return; }
      onDelete();
    });

    actions.append(edit, del);
    return actions;
  }

  /* levels ---------------------------------------------------- */

  // The board says which control ran, so that row lights up. A held gesture
  // repeats, and each repeat keeps the highlight alive.
  let firedTimers = new Map();

  function showFired(action) {
    const row = controlRows.get(action);
    if (!row) return;
    row.classList.add('fired');
    clearTimeout(firedTimers.get(action));
    firedTimers.set(action, setTimeout(() => row.classList.remove('fired'), 500));
  }

  // Two controls can watch one signal, so moving either threshold moves both.
  function syncThresholds(channel, signal) {
    const value = thresholdFor(channel, signal);
    barCells.forEach(cell => {
      if (cell.channel !== channel || cell.signal !== signal) return;
      cell.slider.value = value;
      cell.readout.textContent = format(signal, value);
      paint(cell);
    });
  }

  const format = (signal, value) =>
    signal === 'focus' ? value.toFixed(1) + '%' : String(Math.round(value));

  function paint(cell) {
    const pct = Math.min(cell.shown / cell.scale, 1) * 100;
    const lit = cell.shown >= Number(cell.slider.value) ? 'var(--accent)' : 'var(--text-faint)';
    cell.slider.style.background =
      'linear-gradient(to right, ' + lit + ' ' + pct + '%, var(--surface-hi) ' + pct + '%)';
  }

  // Levels arrive twenty times a second, which steps visibly. Easing towards
  // the latest value each frame smooths that out, and rides over the short
  // gap while an IR send has the loop blocked.
  function animate() {
    barCells.forEach(cell => {
      const gap = cell.target - cell.shown;
      if (Math.abs(gap) > cell.scale / 2000) {
        cell.shown += gap * 0.25;
        paint(cell);
      } else if (cell.shown !== cell.target) {
        cell.shown = cell.target;
        paint(cell);
      }
    });
    requestAnimationFrame(animate);
  }

  function drawLevels(levels) {
    barCells.forEach(cell => {
      if (!cell.signal) return;
      const raw = levels[cell.action] ?? 0;
      cell.target = cell.signal === 'focus' ? raw / 10 : raw;
    });
  }

  function thresholdFor(channel, signal) {
    const t = tuning[channel];
    if (signal === 'muscle') return t.muscleThreshold;
    if (signal === 'focus')  return t.focusThreshold;
    return t.blinkThreshold;
  }

  let pushTimer = null;

  function setThreshold(channel, signal, value) {
    const t = tuning[channel];
    if (signal === 'muscle') {
      t.muscleThreshold = value;
      // hysteresis is not worth its own slider, the muscle just has to relax
      // below this before the next clench counts
      t.muscleRelease = Math.round(value * 0.8);
    } else if (signal === 'focus') {
      t.focusThreshold = value;
    } else {
      t.blinkThreshold = value;
    }
    own.tuning = true;
    store(TUNE_KEY, tuning);
    clearTimeout(pushTimer);
    pushTimer = setTimeout(() => {
      if (BLE.isConnected()) guard(() => BLE.setTuning(channel, t));
    }, 150);
  }

  /* trigger feedback ----------------------------------------- */

  function flashRow(id, className, ms) {
    const row = el.list.querySelector(`[data-id="${id}"]`);
    if (!row) return;
    row.classList.remove(className);
    void row.offsetWidth;
    row.classList.add(className);
    setTimeout(() => row.classList.remove(className), ms);
  }

  function showFire(id) {
    flashRow(id, 'fired', 480);
  }

  function setConnectedUI(connected, deviceName) {
    el.conn.classList.toggle('live', connected);
    el.connTxt.textContent = connected ? 'Disconnect' : 'Connect';
    [el.lock, el.wipe].forEach(b => { b.disabled = !connected; });
    if (connected) setStatus(deviceName);
  }

  /* actions -------------------------------------------------- */

  function setActive(id) {
    if (id === activeId) return;
    const previous = activeId;
    activeId = id;
    render();
    guard(async () => {
      try { await BLE.setActive(id); }
      catch (err) { activeId = previous; render(); throw err; }
    });
  }

  // Unlocking only makes things editable. It does not move anything, and it
  // suspends the board's triggers so a stray gesture cannot fight you while
  // you reassign one.
  function setLocked(on) {
    locked = on;
    render();
    guard(() => BLE.setEdit(!on));
  }

  // Only a destructive action gets the red button. Declining is always
  // Cancel unless the caller has a better word for it.
  const DESTRUCTIVE = ['Delete', 'Erase all'];

  function confirm(title, body, okLabel, ok, decline, declineLabel = 'Cancel') {
    const harmful = DESTRUCTIVE.includes(okLabel);
    el.cfmTtl.textContent = title;
    el.cfmBody.textContent = body;
    el.cfmOk.textContent = okLabel;
    el.cfmOk.classList.toggle('btn-danger', harmful);
    el.cfmOk.classList.toggle('btn-primary', !harmful);
    el.cfmNo.textContent = declineLabel;
    onConfirm = ok;
    onDecline = decline;
    openModal('ov-confirm');
  }

  function confirmDeleteCommand(id) {
    const cmd = commands.get(id);
    if (!cmd) return;
    confirm('Delete command',
      `"${cmd.name}" will be removed from the board. This cannot be undone.`,
      'Delete', () => guard(() => BLE.remove(id)));
  }

  function confirmDeleteProfile(id) {
    const profile = profiles.get(id);
    if (!profile) return;
    confirm('Delete remote',
      `"${profile.name}" and its ${profile.count} command${profile.count === 1 ? '' : 's'} ` +
      'will be removed from the board. This cannot be undone.',
      'Delete', () => guard(() => BLE.deleteProfile(id)));
  }

  function confirmWipe() {
    confirm('Erase everything',
      `All ${profiles.size} remote${profiles.size === 1 ? '' : 's'}, every command ` +
      'inside them and their IR signals will be erased from the board. This cannot be undone.',
      'Erase all', () => guard(() => BLE.wipe()));
  }

  function openRename(kind, id) {
    renameKind = kind;
    renameId = id;
    const fresh = kind === 'profile' && !usable(id);
    el.renTtl.textContent = fresh ? 'Name this remote'
      : kind === 'profile' ? 'Rename remote' : 'Rename command';
    el.renInput.value = fresh ? ''
      : (kind === 'profile' ? profiles.get(id) : commands.get(id))?.name || '';
    el.renCount.textContent = el.renInput.value.length;
    el.renInput.classList.remove('invalid');
    openModal('ov-rename');
    setTimeout(() => el.renInput.focus(), 50);
  }

  function submitRename() {
    const name = el.renInput.value.trim();
    if (!name) { el.renInput.classList.add('invalid'); el.renInput.focus(); return; }
    closeModal('ov-rename');
    guard(() => renameKind === 'profile'
      ? BLE.renameProfile(renameId, name)
      : BLE.rename(renameId, name));
  }

  /* listening ------------------------------------------------ */

  function showListening(seconds) {
    // the overlay must be visible first, a hidden element has no layout
    // to reflow and the bar would jump straight to zero
    openModal('ov-listening');
    const bar = el.listenBar;
    bar.style.transition = 'none';
    bar.style.width = '100%';
    void bar.offsetWidth;
    bar.style.transition = `width ${seconds}s linear`;
    bar.style.width = '0%';

    let left = seconds;
    el.listenSec.textContent = `${left}s`;
    clearInterval(listenTimer);
    listenTimer = setInterval(() => {
      left -= 1;
      el.listenSec.textContent = `${Math.max(left, 0)}s`;
      if (left <= 0) clearInterval(listenTimer);
    }, 1000);
  }

  function hideListening() {
    clearInterval(listenTimer);
    listenTimer = null;
    closeModal('ov-listening');
  }

  function cancelListening() {
    hideListening();
    guard(() => BLE.cancelRec());
  }

  // the board holds the capture in RAM, so every way of dismissing this
  // dialog has to tell it to let go or the button stays stuck
  function discardCapture() {
    closeModal('ov-capture');
    capture = null;
    guard(() => BLE.discard());
  }

  /* capture flow --------------------------------------------- */

  function showCapture(data) {
    capture = data;
    const dup = data.isDuplicate ? commands.get(data.duplicateId) : null;
    const dupName = dup ? dup.name : `slot ${data.duplicateId}`;

    el.capTtl.textContent = data.isDuplicate ? 'Signal already saved' : 'IR signal captured';

    el.capDup.replaceChildren();
    if (data.isDuplicate) {
      const notice = document.createElement('div');
      notice.className = 'notice';
      notice.appendChild(Icons.svg('triangle-alert'));
      const text = document.createElement('span');
      text.textContent = `This matches "${dupName}" in this remote. Save it as a new command or replace the existing one.`;
      notice.appendChild(text);
      el.capDup.appendChild(notice);
    }

    el.capMeta.replaceChildren();
    addMeta('Protocol', BLE.protocolName(data.protocol));
    addMeta('Bits', String(data.bits));
    if (data.protocol !== 0) addMeta('Value', `0x${data.value}`);

    // recordTarget only ever points at a blank slot, so its "name" is just
    // the placeholder ("Command 5") - not worth prefilling.
    el.capInput.value = '';
    el.capCount.textContent = String(el.capInput.value.length);
    el.capInput.classList.remove('invalid');

    el.capActions.replaceChildren();
    addCaptureAction('Discard', 'btn', discardCapture);
    if (data.isDuplicate) {
      addCaptureAction('Replace', 'btn btn-danger', () => {
        const name = el.capInput.value.trim() || dupName;
        closeModal('ov-capture');
        guard(() => BLE.saveOver(data.duplicateId, name));
      });
    }
    addCaptureAction('Save', 'btn btn-primary', () => {
      const name = el.capInput.value.trim();
      if (!name) { el.capInput.classList.add('invalid'); el.capInput.focus(); return; }
      closeModal('ov-capture');
      // it goes to the slot whose button started this, not the next free one
      const slot = recordTarget >= 0 ? recordTarget : data.duplicateId;
      recordTarget = -1;
      if (slot >= 0) guard(() => BLE.saveOver(slot, name));
      else guard(() => BLE.saveNew(name));
    });

    openModal('ov-capture');
    setTimeout(() => el.capInput.focus(), 50);
  }

  function addMeta(term, value) {
    const dt = document.createElement('dt');
    dt.textContent = term;
    const dd = document.createElement('dd');
    dd.textContent = value;
    el.capMeta.append(dt, dd);
  }

  function addCaptureAction(label, className, handler) {
    const btn = document.createElement('button');
    btn.className = className;
    btn.textContent = label;
    btn.addEventListener('click', handler);
    el.capActions.appendChild(btn);
  }

  /* board events --------------------------------------------- */

  function requestProfiles() {
    loading = true;
    setStatus('Loading remotes');
    guard(() => BLE.getProfiles());
    clearTimeout(listTimer);
    listTimer = setTimeout(finishProfiles, 4000);
  }

  BLE.on('connect', name => {
    profiles.clear();
    commands.clear();
    activeId = openProfile = -1;
    screen = HOME;
    locked = true;
    settingsPushed = false;
    configSettled = false;
    setConnectedUI(true, name);
    render();
    // the board streams its profiles on its own, we only need its settings
    loading = true;
    setStatus('Loading settings');
    guard(() => BLE.getConfig());
    // if the config never lands, still show whatever the board has
    clearTimeout(listTimer);
    listTimer = setTimeout(requestProfiles, 4000);
  });

  BLE.on('disconnect', () => {
    profiles.clear();
    commands.clear();
    activeId = openProfile = -1;
    screen = HOME;
    locked = true;
    loading = false;
    clearTimeout(listTimer);
    hideListening();
    // the board drops its pending capture on disconnect, so close the
    // dialog rather than leaving a Save button that cannot work
    closeModal('ov-capture');
    capture = null;
    setConnectedUI(false);
    setStatus('Not connected');
    render();
  });

  // The board is the source of truth for which screen is showing, because a
  // blink can move it without the app being involved.
  // Home only changes what a gesture scrolls. The open profile and its
  // commands stay put, so the list never empties underneath you.
  BLE.on('screen', state => {
    const movedProfile = state.profile !== openProfile;
    screen = state.screen;
    openProfile = state.profile;
    homeCursor = state.home;
    activeId = state.cursor;
    if (movedProfile) {
      // its list is on the way, and holding the last profile's would merge
      // the two into one
      commands.clear();
      loadingCommands = true;
    }
    loading = false;
    render();
  });

  BLE.on('profile', entry => {
    profiles.set(entry.id, { name: entry.name, count: entry.count, named: entry.named });
    loading = true;
    clearTimeout(listTimer);
    listTimer = setTimeout(finishProfiles, 900);
  });

  BLE.on('profileEnd', () => {
    clearTimeout(listTimer);
    openFirstIfNone();
    finishProfiles();
  });

  BLE.on('profileSaved', entry => {
    const existing = profiles.get(entry.id);
    const fresh = !existing || !existing.named;
    profiles.set(entry.id, {
      name: entry.name, count: existing ? existing.count : 0, named: true,
    });
    if (homeCursor < 0) homeCursor = entry.id;
    // a newly set-up remote is the one you meant to look at
    if (fresh) guard(() => BLE.openProfile(entry.id));
    loading = false;
    render();
    toast(`Saved "${entry.name}"`);
  });

  BLE.on('profileDeleted', id => {
    profiles.delete(id);
    if (homeCursor === id) homeCursor = -1;
    // the board does not pick a replacement on its own when the one open
    // was the one just deleted, so this side has to
    openFirstIfNone();
    loading = false;
    render();
    toast('Remote deleted');
  });

  BLE.on('listEntry', entry => {
    commands.set(entry.id, { name: entry.name, recorded: entry.recorded });
    if (entry.active) activeId = entry.id;
    loading = true;
    // the board may never send list-end if a packet is dropped
    clearTimeout(listTimer);
    listTimer = setTimeout(finishList, 900);
  });

  BLE.on('listEnd', () => { clearTimeout(listTimer); finishList(); });

  BLE.on('saved', cmd => {
    // The board only sends this for a command that now has a recording,
    // whether this is a fresh save or a rename of one that already did.
    commands.set(cmd.id, { name: cmd.name, recorded: true });
    if (activeId < 0) activeId = cmd.id;
    bumpCount();
    loading = false;
    render();
    toast(`Saved "${cmd.name}"`);
  });

  BLE.on('deleted', id => {
    commands.delete(id);
    if (activeId === id) activeId = -1;
    bumpCount();
    loading = false;
    render();
    toast('Command deleted');
  });

  // also arrives when a gesture steps the selection on the board
  BLE.on('active', id => {
    activeId = id;
    loading = false;
    render();
  });

  BLE.on('wiped', () => {
    profiles.clear();
    commands.clear();
    activeId = openProfile = homeCursor = -1;
    screen = HOME;
    locked = true;
    loading = false;
    render();
    toast('Everything erased');
  });

  BLE.on('fired', () => {
    const cmd = commands.get(activeId);
    showFire(activeId);
  });

  BLE.on('failed', () => toast('The board could not complete that', true));
  BLE.on('listening', showListening);

  BLE.on('listenEnd', reason => {
    hideListening();
    if (reason === 0) toast('No signal received', true);
    else if (reason === 1) toast('Recording cancelled');
  });

  BLE.on('capture', data => { hideListening(); showCapture(data); });
  BLE.on('stream', drawLevels);
  BLE.on('trigger', action => {
    showFired(action);
    // Switch Remote steps to the next one in order and opens it - opening
    // is what puts it on screen and selects its first command.
    if (action === BLE.ACTION.HOME) switchToNextRemote();
  });

  // Cycles through the remotes that are actually set up, wrapping around.
  // With one or none there is nowhere to go, so it does nothing.
  function switchToNextRemote() {
    const ids = [...profiles.keys()].sort((a, b) => a - b).filter(usable);
    if (ids.length < 2) return;
    const at = ids.indexOf(openProfile);
    const next = ids[(at + 1) % ids.length];
    guard(() => BLE.openProfile(next));
  }


  // The board reports what it has. Its channel count is hardware, so that
  // always wins; what this browser tuned wins over the board's own defaults.
  // Once the opening exchange is done, whatever the board reports is what it
  // is actually sampling, so it wins. A filter the board refused, or a write
  // that never landed, corrects itself here instead of leaving a dead bar.
  BLE.on('channels', cfg => {
    available = cfg.available;
    if (!own.channels || configSettled) {
      channels = { notchHz: cfg.notchHz, filters: cfg.filters.slice() };
      store(CHAN_KEY, channels);
    }
    // never keep a channel this board does not have
    for (let ch = available; ch < MAX_CHANNELS; ch++) channels.filters[ch] = BLE.FILTER.OFF;
    render();
  });

  BLE.on('tuning', t => {
    if (own.tuning) return;                   // ours already wins
    tuning[t.channel] = {
      muscleThreshold: t.muscleThreshold,
      muscleRelease: t.muscleRelease,
      focusThreshold: t.focusThreshold,
      blinkThreshold: t.blinkThreshold,
    };
    store(TUNE_KEY, tuning);
  });

  BLE.on('mapping', m => {
    if (!own.mapping || configSettled) { mapping = m; store(MAP_KEY, mapping); }
    render();
  });

  // The board has told us everything, so now push whatever this browser had.
  //
  // Once per connection only. Applying a channel change makes the board
  // restart sampling and report its config again, and pushing ours back every
  // time that happened would never stop.
  BLE.on('configEnd', () => {
    if (!settingsPushed) {
      settingsPushed = true;
      if (own.channels) guard(() => BLE.setChannels(channels.notchHz, channels.filters));
      if (own.mapping)  guard(() => BLE.setMapping(mapping));
      if (own.tuning)   tuning.forEach((t, ch) => guard(() => BLE.setTuning(ch, t)));
    }
    configSettled = true;
    render();
    // Ask only now. The board paces its bursts one packet per connection
    // interval, and two bursts sharing that wire cost each other packets,
    // which is what left the list empty until a manual refresh.
    requestProfiles();
  });

  // Every slot is in the commands map, recorded or not, because all ten are
  // always listed. The badge only cares about the ones that actually hold
  // an IR code.
  function bumpCount() {
    const profile = profiles.get(openProfile);
    if (profile) profile.count = [...commands.values()].filter(c => c.recorded).length;
  }

  function finishProfiles() {
    loading = false;
    render();
    const n = profiles.size;
    setStatus(`${BLE.DEVICE_NAME} · ${n} remote${n === 1 ? '' : 's'}`);
  }

  function finishList() {
    loading = false;
    loadingCommands = false;
    // Something is always selected once a remote has a command to select.
    // This does not depend on what the board itself landed on when it was
    // opened, so it holds regardless of which firmware is on it.
    if (!commands.get(activeId)?.recorded) {
      const first = [...commands.keys()].sort((a, b) => a - b)
        .find(id => commands.get(id).recorded);
      if (first !== undefined) setActive(first);
    }
    bumpCount();
    render();
    const n = [...commands.values()].filter(c => c.recorded).length;
    setStatus(`${BLE.DEVICE_NAME} · ${n} command${n === 1 ? '' : 's'}`);
  }

  /* wiring --------------------------------------------------- */

  el.conn.addEventListener('click', async () => {
    if (el.conn.classList.contains('unsupported')) {
      toast("This browser can't connect over Bluetooth. Open the page in Chrome, Brave or Edge.", true);
      return;
    }
    if (BLE.isConnected()) { BLE.disconnect(); return; }
    setStatus('Scanning');
    try { await BLE.connect(); }
    catch (err) {
      setStatus('Not connected');
      if (err.name !== 'NotFoundError') toast(err.message, true);
    }
  });

  el.lock.addEventListener('click', () => setLocked(!locked));

  el.notch.addEventListener('click', e => {
    const hz = Number(e.target.closest('button')?.dataset.hz);
    if (!hz || locked) return;
    channels.notchHz = hz;
    commitChannels(null, null);
    render();
  });


  el.info.addEventListener('click', () => openModal('ov-info'));
  el.wipe.addEventListener('click', () => { closeModal('ov-info'); confirmWipe(); });

  el.renSave.addEventListener('click', submitRename);

  el.cfmOk.addEventListener('click', () => {
    closeModal('ov-confirm');
    const fn = onConfirm;
    onConfirm = onDecline = null;
    fn?.();
  });

  el.cfmNo.addEventListener('click', () => {
    closeModal('ov-confirm');
    const fn = onDecline;
    onConfirm = onDecline = null;
    fn?.();
  });

  bindCounter(el.capInput, el.capCount, () => el.capActions.lastElementChild?.click());
  bindCounter(el.renInput, el.renCount, submitRename);

  function bindCounter(input, output, onEnter) {
    input.addEventListener('input', () => {
      output.textContent = input.value.length;
      input.classList.remove('invalid');
    });
    input.addEventListener('keydown', e => {
      if (e.key !== 'Enter') return;
      e.preventDefault();
      onEnter();
    });
  }

  // two overlays mirror state the board is holding, so closing them has to
  // tell the board. every dismissal route goes through here.
  function dismiss(id) {
    if (id === 'ov-listening') cancelListening();
    else if (id === 'ov-capture') discardCapture();
    else closeModal(id);
  }

  document.querySelectorAll('[data-close]').forEach(btn => {
    btn.addEventListener('click', () => dismiss(btn.dataset.close));
  });

  el.listenX.addEventListener('click', cancelListening);

  document.querySelectorAll('.overlay').forEach(overlay => {
    overlay.addEventListener('mousedown', e => {
      if (e.target === overlay) dismiss(overlay.id);
    });
  });

  document.addEventListener('keydown', e => {
    if (e.key !== 'Escape') return;
    const open = document.querySelector('.overlay.show');
    if (open) dismiss(open.id);
  });

  /* start ---------------------------------------------------- */

  Icons.hydrate(document);
  requestAnimationFrame(animate);

  // the channel label is picked from the available width, so follow it
  let resizeTimer = null;
  addEventListener('resize', () => {
    clearTimeout(resizeTimer);
    resizeTimer = setTimeout(render, 150);
  });

  if (!BLE.isSupported()) {
    el.conn.classList.add('unsupported');
    setStatus('Web Bluetooth not supported');
    openModal('ov-browser-warning');
  }

  render();

})();
