/* Bluetooth transport and wire protocol.
   Knows nothing about the DOM. Emits decoded events to subscribers. */

const BLE = (() => {

  const SVC_UUID    = '12345678-1234-1234-1234-1234567890ab';
  const NOTIFY_UUID = '12345678-1234-1234-1234-1234567890ac';
  const WRITE_UUID  = '12345678-1234-1234-1234-1234567890ad';

  const DEVICE_NAME = 'NPG-IR';
  const MAX_NAME    = 16;

  // board -> app
  const EV = {
    PROFILE:       0x10,
    PROFILE_END:   0x11,
    LIST_ENTRY:    0x12,
    LIST_END:      0x14,
    SCREEN:        0x15,
    STREAM:        0x16,
    BLINK:         0x17,
    CHANNELS:      0x18,
    TRIGGER:       0x19,
    PROFILE_SAVED: 0x1A,
    PROFILE_DEL:   0x1B,
    TUNING:        0x1C,
    MAPPING:       0x1D,
    CONFIG_END:    0x1E,
    CAPTURE:       0x20,
    SAVED:         0x21,
    DELETED:       0x22,
    ACTIVE:        0x23,
    OK:            0x24,
    FAIL:          0x25,
    WIPED:         0x26,
    LISTENING:     0x27,
    LISTEN_END:    0x28,
  };

  // app -> board
  const CMD = {
    SET_ACTIVE:     0x01,
    DELETE:         0x02,
    RENAME:         0x03,
    SAVE_NEW:       0x04,
    SAVE_OVER:      0x05,
    GET_LIST:       0x06,
    FIRE:           0x07,
    WIPE:           0x08,
    CANCEL_REC:     0x09,
    DISCARD:        0x0A,
    GET_PROFILES:   0x0B,
    OPEN_PROFILE:   0x0C,
    GO_HOME:        0x0D,
    ADD_PROFILE:    0x0E,
    RENAME_PROFILE: 0x0F,
    DEL_PROFILE:    0x10,
    SET_EDIT:       0x11,
    SET_TUNING:     0x12,
    SET_MAPPING:    0x13,
    GET_CONFIG:     0x14,
    START_REC:      0x15,
    SET_CHANNELS:   0x16,
  };

  // Wire values, so they must match the firmware.
  const ACTION = { SCROLL_DOWN: 1, SCROLL_UP: 2, FIRE: 3, HOME: 4 };

  // What a channel is filtered for. OFF also means the channel is not sampled.
  const FILTER = { OFF: 0, EMG: 1, EEG: 2, EOG: 3 };

  // What a channel can produce. Which of these exist depends on its filter.
  const GESTURE = {
    NONE: 0, CLENCH: 1, CLENCH_HOLD: 2, FOCUS: 3, DOUBLE_BLINK: 4, TRIPLE_BLINK: 5,
  };

  // A control names a channel and a gesture. This channel means unassigned.
  const NO_CHANNEL = 0xFF;

  const SCREEN = { HOME: 0, PROFILE: 1 };

  // Which gestures each filter makes available, and what to call them. A
  // clench is the same detector either way, but on an EMG channel the whole
  // channel is the muscle, so it is not named after the jaw.
  const GESTURES_FOR = {
    [FILTER.EMG]: [
      [GESTURE.CLENCH,       'EMG'],
      [GESTURE.CLENCH_HOLD,  'EMG (Held)'],
    ],
    [FILTER.EEG]: [
      [GESTURE.CLENCH,       'Jaw Clench'],
      [GESTURE.CLENCH_HOLD,  'Jaw Clench (Held)'],
      [GESTURE.FOCUS,        'Focus'],
      [GESTURE.DOUBLE_BLINK, 'Double Blink'],
      [GESTURE.TRIPLE_BLINK, 'Triple Blink'],
    ],
    [FILTER.EOG]: [
      [GESTURE.DOUBLE_BLINK, 'Double Blink'],
      [GESTURE.TRIPLE_BLINK, 'Triple Blink'],
    ],
  };

  // Which threshold a gesture is compared against, so the app knows which
  // slider belongs to which bar.
  function signalFor(gesture) {
    switch (gesture) {
      case GESTURE.CLENCH:
      case GESTURE.CLENCH_HOLD:  return 'muscle';
      case GESTURE.FOCUS:        return 'focus';
      case GESTURE.DOUBLE_BLINK:
      case GESTURE.TRIPLE_BLINK: return 'blink';
      default: return null;
    }
  }

  const PROTOCOLS = {
    0: 'Unknown', 1: 'RC5', 2: 'RC6', 3: 'NEC', 4: 'Sony', 5: 'Panasonic',
    6: 'JVC', 7: 'Samsung', 8: 'Whynter', 9: 'Sanyo', 10: 'LG',
    12: 'Mitsubishi', 15: 'Coolix', 16: 'Daikin', 18: 'Kelvinator',
    20: 'Mitsubishi AC', 24: 'Gree', 29: 'Fujitsu AC', 32: 'Hitachi AC',
    45: 'Toshiba AC', 48: 'Trotec',
  };

  const decoder = new TextDecoder();
  const encoder = new TextEncoder();

  let device = null;
  let writeChar = null;
  const handlers = {};

  function on(event, fn) {
    (handlers[event] ||= []).push(fn);
  }

  function emit(event, payload) {
    (handlers[event] || []).forEach(fn => fn(payload));
  }

  function protocolName(code) {
    return PROTOCOLS[code] || `Protocol ${code}`;
  }

  /* Names are capped at 16 bytes so every packet fits the 20 byte
     payload guaranteed by the default BLE MTU. Slice by encoded
     bytes rather than characters so multi-byte input cannot overrun. */
  function encodeName(text) {
    let bytes = encoder.encode(text);
    if (bytes.length > MAX_NAME) {
      bytes = bytes.slice(0, MAX_NAME);
      // the cut can land inside a character, so drop the partial one.
      // continuation bytes are 10xxxxxx, a lead byte is anything > 0x7F.
      while (bytes.length && (bytes[bytes.length - 1] & 0xC0) === 0x80) {
        bytes = bytes.slice(0, -1);
      }
      if (bytes.length && bytes[bytes.length - 1] > 0x7F) {
        bytes = bytes.slice(0, -1);
      }
    }
    return Array.from(bytes);
  }

  function isSupported() {
    return typeof navigator !== 'undefined' && !!navigator.bluetooth;
  }

  function isConnected() {
    return !!(device && device.gatt.connected && writeChar);
  }

  async function connect() {
    if (!isSupported()) {
      throw new Error('Web Bluetooth is not available in this browser');
    }

    device = await navigator.bluetooth.requestDevice({
      filters: [{ name: DEVICE_NAME }],
      optionalServices: [SVC_UUID],
    });

    device.addEventListener('gattserverdisconnected', () => {
      writeChar = null;
      writeQueue = Promise.resolve();   // drop anything still queued
      emit('disconnect');
    });

    const server  = await device.gatt.connect();
    const service = await server.getPrimaryService(SVC_UUID);

    const notifyChar = await service.getCharacteristic(NOTIFY_UUID);
    await notifyChar.startNotifications();
    notifyChar.addEventListener('characteristicvaluechanged', onNotify);

    writeChar = await service.getCharacteristic(WRITE_UUID);
    emit('connect', device.name || DEVICE_NAME);
  }

  function disconnect() {
    if (device && device.gatt.connected) device.gatt.disconnect();
  }

  /* Web Bluetooth runs one GATT operation at a time. Overlapping writes are
     rejected with "GATT operation already in progress", and pushing a whole
     settings set at once is exactly that, so every write goes through one
     queue. A failed write must not stall the ones behind it. */
  let writeQueue = Promise.resolve();

  function send(bytes) {
    if (!writeChar) return Promise.reject(new Error('Not connected'));
    const run = writeQueue.then(() => {
      if (!writeChar) throw new Error('Not connected');
      return writeChar.writeValueWithResponse(new Uint8Array(bytes));
    });
    writeQueue = run.catch(() => {});
    return run;
  }

  function onNotify(event) {
    const d = new Uint8Array(event.target.value.buffer);
    if (d.length === 0) return;

    switch (d[0]) {

      // A slot is only a remote once it has been named. The top bit of the
      // count carries that, because the packet already fills the payload the
      // default connection guarantees.
      case EV.PROFILE: {
        const nameLen = d[3];
        emit('profile', {
          id: d[1],
          count: d[2] & 0x7F,
          named: (d[2] & 0x80) !== 0,
          name: decoder.decode(d.slice(4, 4 + nameLen)),
        });
        break;
      }

      case EV.PROFILE_END:
        emit('profileEnd');
        break;

      case EV.PROFILE_SAVED: {
        const nameLen = d[2];
        emit('profileSaved', {
          id: d[1],
          name: decoder.decode(d.slice(3, 3 + nameLen)),
        });
        break;
      }

      case EV.PROFILE_DEL:
        emit('profileDeleted', d[1]);
        break;

      // Both cursors travel together: which profile the gestures are on, and
      // which command inside the open profile.
      case EV.SCREEN:
        emit('screen', {
          screen:  d[1],
          profile: d[2] === 0xFF ? -1 : d[2],
          home:    d[3] === 0xFF ? -1 : d[3],
          cursor:  d[4] === 0xFF ? -1 : d[4],
        });
        break;

      // One level per control, about 20 per second. Focus arrives in tenths
      // of a percent, everything else is a raw envelope level.
      case EV.STREAM: {
        const raw = a => d[1 + a * 2] | (d[2 + a * 2] << 8);
        emit('stream', {
          [ACTION.SCROLL_DOWN]: raw(0),
          [ACTION.SCROLL_UP]:   raw(1),
          [ACTION.FIRE]:        raw(2),
          [ACTION.HOME]:        raw(3),
        });
        break;
      }

      case EV.BLINK:
        emit('blink', { channel: d[1], count: d[2] });
        break;

      case EV.CHANNELS:
        emit('channels', {
          available: d[1],
          notchHz: d[2],
          filters: Array.from(d.slice(3, 3 + 6)),
        });
        break;

      case EV.TUNING:
        emit('tuning', {
          channel: d[1],
          muscleThreshold: d[2] | (d[3] << 8),
          muscleRelease:   d[4] | (d[5] << 8),
          focusThreshold: (d[6] | (d[7] << 8)) / 10,
          blinkThreshold:  d[8] | (d[9] << 8),
        });
        break;

      case EV.MAPPING: {
        const bind = i => ({ channel: d[1 + i * 2], gesture: d[2 + i * 2] });
        emit('mapping', {
          [ACTION.SCROLL_DOWN]: bind(0),
          [ACTION.SCROLL_UP]:   bind(1),
          [ACTION.FIRE]:        bind(2),
          [ACTION.HOME]:        bind(3),
        });
        break;
      }

      case EV.CONFIG_END:
        emit('configEnd');
        break;

      case EV.TRIGGER:
        emit('trigger', d[1]);
        break;

      // Every slot arrives, empty ones included. The flags say which one the
      // board is pointing at and which actually hold a waveform.
      case EV.LIST_ENTRY: {
        const nameLen = d[3];
        emit('listEntry', {
          id: d[1],
          active: (d[2] & 1) !== 0,
          recorded: (d[2] & 2) !== 0,
          name: decoder.decode(d.slice(4, 4 + nameLen)),
        });
        break;
      }

      case EV.LIST_END:
        emit('listEnd');
        break;

      case EV.CAPTURE: {
        let value = 0n;
        for (let i = 0; i < 8; i++) value |= BigInt(d[7 + i]) << BigInt(8 * i);
        emit('capture', {
          isDuplicate: d[1] === 1,
          duplicateId: d[2] === 0xFF ? -1 : d[2],
          protocol: d[3] | (d[4] << 8),
          bits: d[5] | (d[6] << 8),
          value: value.toString(16).toUpperCase(),
        });
        break;
      }

      case EV.SAVED: {
        const nameLen = d[2];
        emit('saved', {
          id: d[1],
          name: decoder.decode(d.slice(3, 3 + nameLen)),
        });
        break;
      }

      case EV.DELETED:    emit('deleted', d[1]);         break;
      case EV.ACTIVE:     emit('active',  d[1]);         break;
      case EV.OK:         emit('fired');                 break;
      case EV.FAIL:       emit('failed');                break;
      case EV.WIPED:      emit('wiped');                 break;
      case EV.LISTENING:  emit('listening', d[1] || 15); break;
      case EV.LISTEN_END: emit('listenEnd', d[1] ?? 2);  break;
    }
  }

  /* commands ------------------------------------------------- */

  const u16 = v => [v & 0xFF, (v >> 8) & 0xFF];

  const setActive = id         => send([CMD.SET_ACTIVE, id]);
  const remove    = id         => send([CMD.DELETE, id]);
  const rename    = (id, name) => send([CMD.RENAME, id, ...encodeName(name)]);
  const saveNew   = name       => send([CMD.SAVE_NEW, ...encodeName(name)]);
  const saveOver  = (id, name) => send([CMD.SAVE_OVER, id, ...encodeName(name)]);
  const getList   = ()         => send([CMD.GET_LIST]);
  const fire      = id         => send([CMD.FIRE, id]);
  const wipe      = ()         => send([CMD.WIPE]);
  const cancelRec = ()         => send([CMD.CANCEL_REC]);
  const discard   = ()         => send([CMD.DISCARD]);

  const getProfiles   = ()         => send([CMD.GET_PROFILES]);
  const openProfile   = id         => send([CMD.OPEN_PROFILE, id]);
  const goHome        = ()         => send([CMD.GO_HOME]);
  const addProfile    = name       => send([CMD.ADD_PROFILE, ...encodeName(name)]);
  const renameProfile = (id, name) => send([CMD.RENAME_PROFILE, id, ...encodeName(name)]);
  const deleteProfile = id         => send([CMD.DEL_PROFILE, id]);
  const setEdit       = on         => send([CMD.SET_EDIT, on ? 1 : 0]);
  const getConfig     = ()         => send([CMD.GET_CONFIG]);
  const startRec      = ()         => send([CMD.START_REC]);

  // One channel's thresholds. Focus is a percentage with one decimal so it
  // goes over the wire as tenths; the rest are whole envelope levels.
  const setTuning = (channel, t) => send([
    CMD.SET_TUNING, channel,
    ...u16(t.muscleThreshold), ...u16(t.muscleRelease),
    ...u16(Math.round(t.focusThreshold * 10)), ...u16(t.blinkThreshold),
  ]);

  const setMapping = m => send([
    CMD.SET_MAPPING,
    m[ACTION.SCROLL_DOWN].channel, m[ACTION.SCROLL_DOWN].gesture,
    m[ACTION.SCROLL_UP].channel,   m[ACTION.SCROLL_UP].gesture,
    m[ACTION.FIRE].channel,        m[ACTION.FIRE].gesture,
    m[ACTION.HOME].channel,        m[ACTION.HOME].gesture,
  ]);

  // Changes what the hardware samples, so the board restarts its ADC and
  // answers with the config it actually ended up with.
  const setChannels = (notchHz, filters) => send([
    CMD.SET_CHANNELS, notchHz, ...filters.slice(0, 6),
  ]);

  return {
    MAX_NAME, DEVICE_NAME, ACTION, FILTER, GESTURE, SCREEN, NO_CHANNEL,
    GESTURES_FOR, signalFor,
    on, connect, disconnect, isConnected, isSupported, protocolName,
    setActive, remove, rename, saveNew, saveOver, getList, fire, wipe,
    cancelRec, discard,
    getProfiles, openProfile, goHome, addProfile, renameProfile, deleteProfile,
    setEdit, getConfig, startRec, setTuning, setMapping, setChannels,
  };

})();
