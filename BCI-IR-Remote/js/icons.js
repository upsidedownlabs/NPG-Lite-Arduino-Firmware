/* Lucide icons, inlined.
 *
 * The SVG files in assets/ are the originals from lucide.dev. Their inner
 * markup is copied here verbatim so the icons can inherit colour from CSS,
 * which an <img> cannot do. Keys match the Lucide names, so to add one,
 * download it from lucide.dev and paste its inner markup below. */

const Icons = (() => {
  'use strict';

  const NS = 'http://www.w3.org/2000/svg';

  const SHAPES = {
    'sun':
      '<circle cx="12" cy="12" r="4"/><path d="M12 2v2"/><path d="M12 20v2"/>' +
      '<path d="m4.93 4.93 1.41 1.41"/><path d="m17.66 17.66 1.41 1.41"/>' +
      '<path d="M2 12h2"/><path d="M20 12h2"/><path d="m6.34 17.66-1.41 1.41"/>' +
      '<path d="m19.07 4.93-1.41 1.41"/>',

    'moon':
      '<path d="M20.985 12.486a9 9 0 1 1-9.473-9.472c.405-.022.617.46.402.803a6 6 0 0 0 8.268 8.268c.344-.215.825-.004.803.401"/>',

    'info':
      '<circle cx="12" cy="12" r="10"/><path d="M12 16v-4"/><path d="M12 8h.01"/>',

    'bluetooth':
      '<path d="m7 7 10 10-5 5V2l5 5L7 17"/>',

    'power':
      '<path d="M12 2v10"/><path d="M18.4 6.6a9 9 0 1 1-12.77.04"/>',

    'pencil':
      '<path d="M21.174 6.812a1 1 0 0 0-3.986-3.987L3.842 16.174a2 2 0 0 0-.5.83l-1.321 4.352a.5.5 0 0 0 .623.622l4.353-1.32a2 2 0 0 0 .83-.497z"/>' +
      '<path d="m15 5 4 4"/>',

    'trash':
      '<path d="M10 11v6"/><path d="M14 11v6"/>' +
      '<path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6"/><path d="M3 6h18"/>' +
      '<path d="M8 6V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/>',

    'x':
      '<path d="M18 6 6 18"/><path d="m6 6 12 12"/>',

    'refresh-cw':
      '<path d="M3 12a9 9 0 0 1 9-9 9.75 9.75 0 0 1 6.74 2.74L21 8"/>' +
      '<path d="M21 3v5h-5"/>' +
      '<path d="M21 12a9 9 0 0 1-9 9 9.75 9.75 0 0 1-6.74-2.74L3 16"/>' +
      '<path d="M8 16H3v5"/>',

    'triangle-alert':
      '<path d="m21.73 18-8-14a2 2 0 0 0-3.48 0l-8 14A2 2 0 0 0 4 21h16a2 2 0 0 0 1.73-3"/>' +
      '<path d="M12 9v4"/><path d="M12 17h.01"/>',

    'radio-tower':
      '<path d="M4.9 16.1C1 12.2 1 5.8 4.9 1.9"/>' +
      '<path d="M7.8 4.7a6.14 6.14 0 0 0-.8 7.5"/><circle cx="12" cy="9" r="2"/>' +
      '<path d="M16.2 4.8c2 2 2.26 5.11.8 7.47"/>' +
      '<path d="M19.1 1.9a9.96 9.96 0 0 1 0 14.1"/><path d="M9.5 18h5"/>' +
      '<path d="m8 22 4-11 4 11"/>',

    'sliders-horizontal':
      '<path d="M10 5H3"/><path d="M12 19H3"/><path d="M14 3v4"/><path d="M16 17v4"/>' +
      '<path d="M21 12h-9"/><path d="M21 19h-5"/><path d="M21 5h-7"/>' +
      '<path d="M8 10v4"/><path d="M8 12H3"/>',

    'arrow-left':
      '<path d="m12 19-7-7 7-7"/><path d="M19 12H5"/>',

    'plus':
      '<path d="M5 12h14"/><path d="M12 5v14"/>',

    'house':
      '<path d="M15 21v-8a1 1 0 0 0-1-1h-4a1 1 0 0 0-1 1v8"/>' +
      '<path d="M3 10a2 2 0 0 1 .709-1.528l7-6a2 2 0 0 1 2.582 0l7 6A2 2 0 0 1 21 10v9a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/>',

    'eye':
      '<path d="M2.062 12.348a1 1 0 0 1 0-.696 10.75 10.75 0 0 1 19.876 0 1 1 0 0 1 0 .696 10.75 10.75 0 0 1-19.876 0"/>' +
      '<circle cx="12" cy="12" r="3"/>',

    'brain':
      '<path d="M12 18V5"/><path d="M15 13a4.17 4.17 0 0 1-3-4 4.17 4.17 0 0 1-3 4"/>' +
      '<path d="M17.598 6.5A3 3 0 1 0 12 5a3 3 0 1 0-5.598 1.5"/>' +
      '<path d="M17.997 5.125a4 4 0 0 1 2.526 5.77"/><path d="M18 18a4 4 0 0 0 2-7.464"/>' +
      '<path d="M19.967 17.483A4 4 0 1 1 12 18a4 4 0 1 1-7.967-.517"/>' +
      '<path d="M6 18a4 4 0 0 1-2-7.464"/><path d="M6.003 5.125a4 4 0 0 0-2.526 5.77"/>',

    'send-horizontal':
      '<path d="M3.714 3.048a.498.498 0 0 0-.683.627l2.843 7.627a2 2 0 0 1 0 1.396l-2.842 7.627a.498.498 0 0 0 .682.627l18-8.5a.5.5 0 0 0 0-.904z"/>' +
      '<path d="M6 12h16"/>',

    'move-right':
      '<path d="M18 8L22 12L18 16"/><path d="M2 12H22"/>',

    'move-left':
      '<path d="M6 8L2 12L6 16"/><path d="M2 12H22"/>',

    'lock':
      '<rect width="18" height="11" x="3" y="11" rx="2" ry="2"/>' +
      '<path d="M7 11V7a5 5 0 0 1 10 0v4"/>',

    'lock-open':
      '<rect width="18" height="11" x="3" y="11" rx="2" ry="2"/>' +
      '<path d="M7 11V7a5 5 0 0 1 9.9-1"/>',

    'settings':
      '<path d="M9.671 4.136a2.34 2.34 0 0 1 4.659 0 2.34 2.34 0 0 0 3.319 1.915 2.34 2.34 0 0 1 2.33 4.033 2.34 2.34 0 0 0 0 3.831 2.34 2.34 0 0 1-2.33 4.033 2.34 2.34 0 0 0-3.319 1.915 2.34 2.34 0 0 1-4.659 0 2.34 2.34 0 0 0-3.32-1.915 2.34 2.34 0 0 1-2.33-4.033 2.34 2.34 0 0 0 0-3.831A2.34 2.34 0 0 1 6.35 6.051a2.34 2.34 0 0 0 3.319-1.915"/>' +
      '<circle cx="12" cy="12" r="3"/>',

    'chevron-right':
      '<path d="m9 18 6-6-6-6"/>',

    'activity':
      '<path d="M22 12h-2.48a2 2 0 0 0-1.93 1.46l-2.35 8.36a.25.25 0 0 1-.48 0L9.24 2.18a.25.25 0 0 0-.48 0l-2.35 8.36A2 2 0 0 1 4.49 12H2"/>'
  };

  // Stroke width, caps and colour come from the stylesheet, so the icons
  // match the text around them at every size.
  function svg(name) {
    const node = document.createElementNS(NS, 'svg');
    node.setAttribute('viewBox', '0 0 24 24');
    node.setAttribute('aria-hidden', 'true');
    node.innerHTML = SHAPES[name] || '';
    return node;
  }

  // Fills in every <span class="ico" data-icon="name"> under root.
  function hydrate(root) {
    (root || document).querySelectorAll('[data-icon]').forEach(slot => {
      slot.replaceChildren(svg(slot.dataset.icon));
    });
  }

  return { svg, hydrate };
})();
