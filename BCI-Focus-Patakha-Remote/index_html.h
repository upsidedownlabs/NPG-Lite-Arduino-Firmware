// Web page for Focus Patakha, served by the NPG Lite.
// Kept in its own file so the Arduino IDE's prototype generator (ctags)
// never scans the JavaScript inside it.
#pragma once
#include <Arduino.h>

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>Focus Patakha</title>
<style>
:root{--bg:#0b0d13;--card:#141824;--line:#252b3b;--text:#e8ecf7;--muted:#8b93a8;--accent:#8b5cf6;--on:#fbbf24;--ok:#22c55e;--warn:#f59e0b;--bad:#ef4444;--gap:clamp(8px,1.6vmin,16px);--pad:clamp(10px,2vmin,20px)}
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;overflow:hidden}
body{background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;height:100vh;height:100dvh;padding:var(--gap);display:flex;flex-direction:column;gap:var(--gap)}
button{font-family:inherit}
header{flex:0 0 auto;display:flex;justify-content:space-between;align-items:center;gap:10px;max-width:1400px;width:100%;margin:0 auto}
h1{font-size:clamp(1rem,2.6vmin,1.5rem);font-weight:700;letter-spacing:-.02em;white-space:nowrap}
h1 span{color:var(--on)}
.brand{display:flex;align-items:baseline;gap:12px;min-width:0}
.by{position:relative;padding-left:12px;font-size:clamp(.66rem,1.6vmin,.85rem);color:var(--muted);text-decoration:none;white-space:nowrap;letter-spacing:.02em}
.by::before{content:"";position:absolute;left:0;top:15%;bottom:15%;width:1px;background:var(--line)}
.by b{font-weight:700;background:linear-gradient(90deg,#fbbf24,#fb7185 55%,#a78bfa);-webkit-background-clip:text;background-clip:text;color:transparent}
.by:hover b{filter:brightness(1.2)}
.chips{display:flex;gap:6px;min-width:0}
.chip{display:flex;align-items:center;gap:6px;padding:4px 10px;border-radius:999px;background:var(--card);border:1px solid var(--line);font-size:clamp(.68rem,1.6vmin,.85rem);color:var(--muted);white-space:nowrap}
.chip em{font-style:normal;font-variant-numeric:tabular-nums;color:#6b7390}
.dot{width:8px;height:8px;border-radius:50%;background:var(--bad);display:inline-block;flex:0 0 auto}
.dot.ok{background:var(--ok);box-shadow:0 0 8px var(--ok)}
.dot.warn{background:var(--warn);box-shadow:0 0 8px var(--warn)}
.wrap{flex:1 1 auto;min-height:0;max-width:1400px;width:100%;margin:0 auto;display:grid;gap:var(--gap);
 grid-template-columns:minmax(0,1.4fr) minmax(0,1fr);grid-template-rows:minmax(0,1.6fr) minmax(0,1fr);grid-template-areas:"hero ctrl" "chart chart"}
.card{background:var(--card);border:1px solid var(--line);border-radius:clamp(12px,2vmin,18px);padding:var(--pad);min-height:0;min-width:0;display:flex;flex-direction:column}
.card h2{font-size:clamp(.6rem,1.4vmin,.75rem);text-transform:uppercase;letter-spacing:.09em;color:var(--muted);font-weight:600;flex:0 0 auto;display:flex;justify-content:space-between;align-items:center;gap:8px;white-space:nowrap}
.h2r{display:flex;align-items:center;gap:10px;text-transform:none;letter-spacing:0;font-weight:500}
.hero-card{grid-area:hero}.ctrl{grid-area:ctrl}.chart-card{grid-area:chart}

/* ---------- hero ---------- */
.hero{flex:1 1 auto;min-height:0;container-type:size;display:flex;align-items:center;justify-content:space-evenly;gap:2cqw}
.gauge{position:relative;height:min(94cqh,52cqw);aspect-ratio:1;container-type:size;flex:0 0 auto}
.gauge svg{display:block;width:100%;height:100%;overflow:hidden}
.g-track{fill:none;stroke:#222839;stroke-width:16;stroke-linecap:round;stroke-dasharray:471.24 999}
.g-val{fill:none;stroke:var(--accent);stroke-width:16;stroke-linecap:round;stroke-dasharray:0 999;transition:stroke-dasharray .25s ease,stroke .3s;filter:drop-shadow(0 0 6px rgba(139,92,246,.6))}
.g-val.hot{stroke:var(--on);filter:drop-shadow(0 0 10px rgba(251,191,36,.8))}
.g-th{fill:none;stroke:#fff;stroke-width:24;stroke-dasharray:3 999;transition:stroke-dashoffset .2s}
.g-center{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center}
.g-num{font-size:22cqw;font-weight:750;font-variant-numeric:tabular-nums;line-height:1}
.g-unit{color:var(--muted);font-size:5.5cqw;margin-top:1cqw}
.g-state{margin-top:4cqw;font-size:5cqw;font-weight:600;padding:1.6cqw 5cqw;border-radius:999px;background:#1d2232;color:var(--muted);white-space:nowrap}
.g-state.hot{background:rgba(251,191,36,.15);color:var(--on)}
.cracker{height:min(100cqh,62cqw);aspect-ratio:.7;flex:0 0 auto;display:flex;flex-direction:column;align-items:center;container-type:inline-size}
.art{position:relative;flex:1 1 auto;min-height:0;width:100%}
.art::before{display:none;content:"";position:absolute;left:5%;right:5%;bottom:0;height:55%;border-radius:50%;background:radial-gradient(closest-side,rgba(251,146,60,.35),rgba(251,146,60,0));opacity:0;transition:opacity .4s}
.art.lit::before{opacity:1}
#cardCv{position:absolute;inset:0;width:100%;height:100%;display:block}
.c-label{margin-top:2cqw;font-weight:750;letter-spacing:.06em;font-size:max(.7rem,8cqw);color:var(--muted);white-space:nowrap;flex:0 0 auto}
.c-label.on{color:var(--on);text-shadow:0 0 12px rgba(251,191,36,.6)}
.c-sub{font-size:max(.62rem,6cqw);color:var(--muted);margin-top:1cqw;height:1.3em;width:100%;text-align:center;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;flex:0 0 auto}

/* ---------- controls ---------- */
.ctrl{justify-content:space-around;gap:clamp(6px,1.4vmin,14px)}
.th-row{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-top:clamp(4px,1vmin,10px)}
.th-val{font-size:clamp(1.5rem,5vmin,2.6rem);font-weight:750;font-variant-numeric:tabular-nums;color:var(--on);line-height:1.1}
.th-val small{font-size:.4em;color:var(--muted);margin-left:2px}
.step{width:clamp(32px,6vmin,46px);height:clamp(32px,6vmin,46px);border-radius:12px;border:1px solid var(--line);background:#1b2030;color:var(--text);font-size:clamp(1rem,2.6vmin,1.4rem);cursor:pointer;flex:0 0 auto}
.step:hover{background:#232a3d}
input[type=range]{width:100%;-webkit-appearance:none;appearance:none;height:8px;border-radius:99px;background:#232a3b;outline:none;margin:clamp(6px,1.2vmin,10px) 0 2px}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;border-radius:50%;background:var(--on);border:3px solid #fff3c4;cursor:pointer;box-shadow:0 0 10px rgba(251,191,36,.6)}
input[type=range]::-moz-range-thumb{width:18px;height:18px;border-radius:50%;background:var(--on);border:3px solid #fff3c4;cursor:pointer}
.save{font-size:clamp(.6rem,1.4vmin,.78rem);color:var(--muted)}
.seg,.picks{display:grid;background:#0f121b;border:1px solid var(--line);border-radius:12px;padding:3px;gap:3px;margin-top:clamp(4px,1vmin,10px)}
.seg{grid-template-columns:repeat(3,1fr)}
.picks{grid-template-columns:repeat(3,1fr)}
.seg button,.picks button{border:0;background:transparent;color:var(--muted);padding:clamp(5px,1.3vmin,10px) 2px;border-radius:9px;font-weight:600;cursor:pointer;font-size:clamp(.64rem,1.55vmin,.85rem);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.seg button.active{background:#262d42;color:var(--text)}
.picks button.active{background:rgba(251,191,36,.16);color:var(--on);box-shadow:inset 0 0 0 1px rgba(251,191,36,.45)}
.link{border:0;background:transparent;color:var(--on);cursor:pointer;font-size:inherit;font-weight:600}
.link:hover{text-decoration:underline}
.sw{display:flex;align-items:center;gap:6px;cursor:pointer;color:var(--muted);user-select:none}
.sw input{display:none}
.sw i{width:26px;height:15px;border-radius:99px;background:#2a3042;position:relative;transition:background .2s;flex:0 0 auto}
.sw i::after{content:"";position:absolute;top:2px;left:2px;width:11px;height:11px;border-radius:50%;background:#8b93a8;transition:transform .2s,background .2s}
.sw input:checked+i{background:rgba(251,191,36,.35)}
.sw input:checked+i::after{transform:translateX(11px);background:var(--on)}

/* ---------- chart ---------- */
.chart-card h2{margin-bottom:clamp(4px,1vmin,10px)}
.hstats{text-transform:none;letter-spacing:0;font-weight:500;display:flex;gap:10px;align-items:center}
.hstats b{color:var(--text);font-variant-numeric:tabular-nums;font-weight:650}
canvas#chart{flex:1 1 auto;min-height:0;width:100%;display:block}
.legend{display:flex;gap:14px;font-size:clamp(.6rem,1.4vmin,.75rem);color:var(--muted);margin-top:clamp(3px,.8vmin,8px);flex:0 0 auto}
.legend i{display:inline-block;width:14px;height:3px;border-radius:2px;margin-right:6px;vertical-align:middle}

/* ---------- sparks layer over the whole page ---------- */
#fx{position:fixed;inset:0;width:100vw;height:100vh;height:100dvh;pointer-events:none;z-index:40;display:block}
.card{transition:border-color .5s,box-shadow .5s}
body.lightA .card{animation:cardlight .8s ease-out}
@keyframes cardlight{0%{border-color:rgba(251,191,36,.6);box-shadow:0 0 34px rgba(251,146,60,.35),inset 0 0 70px rgba(251,191,36,.12)}100%{}}
body.shake .wrap{animation:shk .5s linear}
@keyframes shk{10%,50%,90%{transform:translate(-5px,2px)}30%,70%{transform:translate(5px,-3px)}100%{transform:none}}

/* ---------- portrait / phones ---------- */
@media (max-aspect-ratio:1/1){
 .wrap{grid-template-columns:minmax(0,1fr);grid-template-rows:minmax(0,1.5fr) auto minmax(0,1fr);grid-template-areas:"hero" "ctrl" "chart"}
}
@media (max-height:520px) and (min-aspect-ratio:1/1){
 .ctrl{gap:4px}.th-row{margin-top:2px}.step{width:28px;height:28px}.th-val{font-size:1.3rem}
 .seg button,.picks button{padding:4px 2px}.seg,.picks{margin-top:3px}input[type=range]{margin:5px 0 0}
}
@media (max-width:520px){.brand{flex-direction:column;gap:0}.by{padding-left:0}.by::before{display:none}.chip em{display:none}.legend{gap:10px}.hstats{gap:6px}}
@media (max-width:380px){#resetStats{display:none}}
</style></head><body>
 <header>
  <div class="brand"><h1>Focus <span>Patakha</span></h1><a class="by" href="https://github.com/upsidedownlabs" target="_blank" rel="noopener">by <b>Upside Down Labs</b></a></div>
  <div class="chips">
   <div class="chip" title="Web page link to the NPG Lite: round-trip time and EEG sample rate"><i class="dot" id="npgDot"></i>NPG<em id="npgMeta"></em></div>
   <div class="chip" title="XIAO replies per second (should be ~10)"><i class="dot" id="xiaoDot"></i>XIAO<em id="xiaoMeta"> offline</em></div>
  </div>
 </header>
 <main class="wrap">
  <section class="card hero-card">
   <h2>Live focus &middot; beta power</h2>
   <div class="hero">
    <div class="gauge">
     <svg viewBox="0 0 240 240"><g transform="rotate(135 120 120)">
      <circle cx="120" cy="120" r="100" class="g-track"/>
      <circle cx="120" cy="120" r="100" class="g-val" id="gv"/>
      <circle cx="120" cy="120" r="100" class="g-th" id="gt"/>
     </g></svg>
     <div class="g-center">
      <div class="g-num" id="focusNum">0.0</div>
      <div class="g-unit">% beta</div>
      <div class="g-state" id="focusState">Waiting for signal</div>
     </div>
    </div>
    <div class="cracker">
     <div class="art" id="art"></div>
     <div class="c-label" id="cLabel">NOT LIT</div>
     <div class="c-sub" id="cSub"></div>
    </div>
   </div>
  </section>
  <section class="card ctrl">
   <div>
    <h2>Focus threshold <span class="save" id="saveTxt"></span></h2>
    <div class="th-row">
     <button class="step" id="thMinus">&minus;</button>
     <div class="th-val"><span id="thNum">8.0</span><small>%</small></div>
     <button class="step" id="thPlus">+</button>
    </div>
    <input type="range" id="thSlider" min="5" max="40" step="0.5" value="8">
   </div>
   <div>
    <h2>Cracker
     
    </h2>
    <div class="picks" id="picks">
     <button data-c="anaar">Anaar</button><button data-c="sheeta">Special Patakha</button><button data-c="chakri">Chakri</button><button data-c="rocket">Rocket</button><button data-c="sparkler">Phuljhadi</button><button data-c="skyshot">Sky shot</button>
    </div>
   </div>
   <div>
    <h2>Mode</h2>
    <div class="seg" id="seg"><button data-m="0">Auto</button><button data-m="1">Always lit</button><button data-m="2">Off</button></div>
   </div>
  </section>
  <section class="card chart-card">
   <h2>Last 30 s
    <span class="hstats"><span><span class="lbl">Avg </span><b id="stAvg">&ndash;</b></span><span><span class="lbl">Peak </span><b id="stPeak">&ndash;</b></span><span><span class="lbl">Lit </span><b id="stAbove">&ndash;</b></span><button class="link" id="resetStats">Reset</button></span>
   </h2>
   <canvas id="chart"></canvas>
   <div class="legend"><span><i style="background:#a78bfa"></i>Focus</span><span><i style="background:#fbbf24"></i>Cracker lit</span><span><i style="background:repeating-linear-gradient(90deg,#fbbf24 0 4px,transparent 4px 7px)"></i>Threshold</span></div>
  </section>
 </main>
 <canvas id="fx"></canvas>
<script>
const $=id=>document.getElementById(id);
const ARC=471.24, MAXG=40, WIN=30000;
let th=8, editing=false, saveT=null, hist=[], base='', rtt=0, fails=0, lastD=null;
let st={sum:0,n:0,peak:0,above:0};
const store={get(k,d){try{const v=localStorage.getItem(k);return v===null?d:v;}catch(e){return d;}},set(k,v){try{localStorage.setItem(k,v);}catch(e){}}};
const api=p=>fetch(base+p,{cache:'no-store'});

/* ================= gauge + threshold ================= */
function setGauge(v,hot){const f=Math.max(0,Math.min(1,v/MAXG));const g=$('gv');g.style.strokeDasharray=(ARC*f)+' 999';g.classList.toggle('hot',hot);}
function setThMark(t){const f=Math.max(0,Math.min(1,t/MAXG));$('gt').style.strokeDashoffset=-(ARC*f-1.5);}
function setTh(v,send){
  th=Math.round(Math.max(5,Math.min(40,v))*2)/2;
  $('thNum').textContent=th.toFixed(1); $('thSlider').value=th; setThMark(th);
  if(!send) return;
  editing=true; $('saveTxt').textContent='Saving…';
  clearTimeout(saveT);
  saveT=setTimeout(()=>{
    api('/set?th='+th).then(r=>{$('saveTxt').textContent=r.ok?'✓ Saved':'Save failed';})
      .catch(()=>{$('saveTxt').textContent='Save failed';})
      .finally(()=>{editing=false;});
  },300);
}
$('thSlider').addEventListener('input',e=>setTh(parseFloat(e.target.value),true));
$('thMinus').onclick=()=>setTh(th-0.5,true);
$('thPlus').onclick=()=>setTh(th+0.5,true);
function setModeUI(m){document.querySelectorAll('#seg button').forEach(b=>b.classList.toggle('active',+b.dataset.m===m));}
document.querySelectorAll('#seg button').forEach(b=>b.onclick=()=>{const m=+b.dataset.m;setModeUI(m);api('/mode?m='+m).catch(()=>{});});
$('resetStats').onclick=()=>{st={sum:0,n:0,peak:0,above:0};};

/* ================= particle engine ================= */
/* One transparent canvas covers the whole page (pointer-events:none).
   The cracker is drawn exactly where its card slot is; sparks fly over
   every card, land on card edges, and bursts light the cards up.        */
const TAU=Math.PI*2, R=Math.random, rr=(a,b)=>a+R()*(b-a);
const HUES=[0,28,48,120,190,280,320];
function glow(c,x,y,r,col){if(r<=0)return;const g=c.createRadialGradient(x,y,0,x,y,r);g.addColorStop(0,col);g.addColorStop(1,'rgba(0,0,0,0)');c.fillStyle=g;c.beginPath();c.arc(x,y,r,0,TAU);c.fill();}
let pfT=null;
function pageFlash(big){const b=document.body;b.classList.remove('lightA','shake');void b.offsetWidth;b.classList.add('lightA');if(big)b.classList.add('shake');
  clearTimeout(pfT);pfT=setTimeout(()=>b.classList.remove('lightA','shake'),850);}
class Stage{
  constructor(cv,cap){this.cv=cv;this.c=cv.getContext('2d');this.p=[];this.fl=[];this.rings=[];this.W=0;this.H=0;this.cap=cap;this.surf=null;this.fs=false;}
  resize(){const d=Math.min(2,window.devicePixelRatio||1);this.W=window.innerWidth;this.H=window.innerHeight;
    this.cv.width=Math.max(1,Math.round(this.W*d));this.cv.height=Math.max(1,Math.round(this.H*d));this.c.setTransform(d,0,0,d,0,0);}
  spark(x,y,vx,vy,o){if(this.p.length>=this.cap)return;const p={x,y,vx,vy,g:0,dr:1,life:40,max:40,h:40,l:65,w:1.4,bn:this.fs};Object.assign(p,o);this.p.push(p);}
  burst(x,y,rad,n){const h1=HUES[R()*HUES.length|0],h2=R()<.4?HUES[R()*HUES.length|0]:h1;
    for(let i=0;i<n;i++){const a=R()*TAU,v=rad*rr(.45,1)/46;
      this.spark(x,y,Math.cos(a)*v,Math.sin(a)*v,{g:rad*.00022,dr:.985,life:rr(50,85),max:85,h:(i&1?h1:h2)+rr(-10,10),l:rr(55,72),w:rr(1.2,2.3)});}
    this.fl.push({x,y,r:rad*(this.fs?1.4:.7),a:1});
    if(this.fs)pageFlash(false);}
  boom(x,y,rad){                       // big bang: debris + flash + shock ring
    for(let i=0;i<240;i++){const a=R()*TAU,v=rad*rr(.25,1)/26,hot=R()<.25;
      this.spark(x,y,Math.cos(a)*v,Math.sin(a)*v,{g:rad*.00035,dr:.965,life:rr(40,95),max:95,h:hot?50:rr(0,45),l:hot?95:rr(50,70),w:rr(1.2,3)});}
    for(let i=0;i<24;i++){const a=-Math.PI*rr(.1,.9),v=rad*rr(.4,1)/30;   // paper pieces
      this.spark(x,y,Math.cos(a)*v,Math.sin(a)*v,{g:rad*.0005,dr:.97,life:rr(60,110),max:110,h:i&1?0:125,l:42,w:rr(2.5,4)});}
    this.fl.push({x,y,r:rad*1.5,a:1.3});
    if(this.fs)pageFlash(true);}
  run(k){const c=this.c,P=this.p,S=this.surf;c.globalCompositeOperation='lighter';c.lineCap='round';
    for(let i=P.length-1;i>=0;i--){const p=P[i],py=p.y;
      if(p.dr!==1){const d=Math.pow(p.dr,k);p.vx*=d;p.vy*=d;}
      p.vy+=p.g*k;p.x+=p.vx*k;p.y+=p.vy*k;p.life-=k;
      if(S&&p.bn&&p.vy>0){for(const r of S){if(p.x>r.left&&p.x<r.right&&py<r.top&&p.y>=r.top){   // lands on a card edge
            p.y=r.top;p.vy*=-rr(.2,.4);p.vx*=.6;p.bn=false;p.life=Math.min(p.life,rr(10,26));break;}}}
      if(p.life<=0||p.y>this.H+20||p.x<-50||p.x>this.W+50){P[i]=P[P.length-1];P.pop();continue;}
      const a=Math.min(1,p.life/p.max*1.5);
      c.strokeStyle='hsla('+p.h+',100%,'+p.l+'%,'+a.toFixed(2)+')';c.lineWidth=p.w;
      c.beginPath();c.moveTo(p.x-p.vx*1.6,p.y-p.vy*1.6);c.lineTo(p.x+.01,p.y);c.stroke();}
    for(let i=this.fl.length-1;i>=0;i--){const f=this.fl[i];glow(c,f.x,f.y,f.r,'rgba(255,236,200,'+(Math.min(1,f.a)*.45).toFixed(2)+')');f.a-=.06*k;if(f.a<=0)this.fl.splice(i,1);}
    c.globalCompositeOperation='source-over';}
}
const lighter=(c,fn)=>{c.globalCompositeOperation='lighter';fn();c.globalCompositeOperation='source-over';};
function fuse(c,x,y,s,dir){c.strokeStyle='#9ca3af';c.lineWidth=Math.max(1,.012*s);c.lineCap='round';c.beginPath();c.moveTo(x,y);
  c.bezierCurveTo(x,y-.08*s,x+dir*.09*s,y-.07*s,x+dir*.07*s,y-.16*s);c.stroke();return {x:x+dir*.07*s,y:y-.16*s};}
const amt=(k,base,s)=>Math.ceil(base*k*Math.min(2,Math.max(.6,s/90)));
/* fountain shared by Anaar and Sheeta Geeta: aims at a height, works in card or full page */
function fountain(S,sl,k,x,top,rate,hue,boost){const {s,fs}=sl;
  const reach=fs?Math.max(top-10,2*s):2*s, g=fs?Math.max(s*.00097,reach*.0005):s*.00097;
  for(let i=amt(k,(fs?rate*1.5:rate),s);i--;){const a=-Math.PI/2+rr(-1,1)*(fs?.42:.37),hh=reach*rr(fs?.3:.45,1)*(boost||1),v=Math.sqrt(2*g*hh),w=R()<.18;
    S.spark(x+rr(-.02,.02)*s,top,Math.cos(a)*v,Math.sin(a)*v,{g,life:Math.min(240,2.1*v/g)*rr(.8,1.05),max:120,h:w?50:hue+rr(-12,12),l:w?92:rr(60,75),w:rr(1,2.3)});}
  lighter(S.c,()=>glow(S.c,x,top,s*.25,'rgba(255,240,200,.9)'));}

/* ================= crackers ================= */
const CR={
 anaar:{name:'Anaar',frame(S,sl,k,lit){const c=S.c,{x,y,s,fs}=sl,q=sl.st,FT=280;
   if(!q.ph){q.ph='idle';q.t=0;}
   if(q.t>0)q.t-=k;
   if(q.ph==='idle'&&lit&&q.t<=0){q.ph='fount';q.t=FT;}
   if(q.ph==='fount'&&q.t<=0){S.boom(x,y-.3*s,fs?Math.min(S.W,S.H)*.5:1.05*s);q.ph='gone';q.t=95;}
   if(q.ph==='gone'&&q.t<=0){q.ph='idle';q.t=lit?30:0;}
   const h=.55*s,tw=.07*s,bw=.27*s;
   c.fillStyle='rgba(0,0,0,.35)';c.beginPath();c.ellipse(x,y,bw*1.1,.04*s,0,0,TAU);c.fill();
   if(q.ph==='gone'){c.fillStyle='rgba(40,30,25,.8)';c.beginPath();c.ellipse(x,y-.01*s,bw*.8,.03*s,0,0,TAU);c.fill();sl.fuse=null;return;}
   const on=q.ph==='fount',p=on?1-q.t/FT:0,cx=x+(on&&p>.75?rr(-1,1)*(p-.75)*8:0),top=y-h,lt=on||lit;
   const g=c.createLinearGradient(cx-bw,0,cx+bw,0),[d,b]=lt?['#7f1d1d','#ef4444']:['#3b1414','#6b2a2a'];g.addColorStop(0,d);g.addColorStop(.45,b);g.addColorStop(1,d);
   c.fillStyle=g;c.beginPath();c.moveTo(cx-tw,top);c.lineTo(cx+tw,top);c.lineTo(cx+bw,y-.03*s);c.quadraticCurveTo(cx,y+.03*s,cx-bw,y-.03*s);c.closePath();c.fill();
   const band=(f1,f2)=>{const w1=tw+(bw-tw)*f1,w2=tw+(bw-tw)*f2;c.beginPath();c.moveTo(cx-w1,top+h*f1);c.lineTo(cx+w1,top+h*f1);c.lineTo(cx+w2,top+h*f2);c.lineTo(cx-w2,top+h*f2);c.closePath();c.fill();};
   c.fillStyle=lt?'#facc15':'#5f5220';band(.22,.34);band(.55,.67);
   c.fillStyle=lt?'#f5d0a9':'#5b4f43';c.beginPath();c.ellipse(cx,top,tw*1.25,.025*s,0,0,TAU);c.fill();
   sl.fuse=on?{x:cx,y:top}:fuse(c,cx,top,s,1);
   if(on){sl.hot={x:cx,y:top};fountain(S,sl,k,cx,top,4,40,.85+.3*p);
     if(p>.85)lighter(c,()=>glow(c,cx,y-.3*s,s*.5*p,'rgba(255,200,120,.35)'));}}},

 sheeta:{name:'Special Patakha',frame(S,sl,k,lit){const c=S.c,{x,y,s,fs}=sl,q=sl.st,FT=220;
   if(!q.ph){q.ph='idle';q.t=0;}
   if(q.t>0)q.t-=k;
   if(q.ph==='idle'&&lit&&q.t<=0){q.ph='fount';q.t=FT;}
   if(q.ph==='fount'&&q.t<=0){S.boom(x,y-.45*s,fs?Math.min(S.W,S.H)*.55:1.15*s);q.ph='gone';q.t=95;}
   if(q.ph==='gone'&&q.t<=0){q.ph='idle';q.t=lit?30:0;}
   const w=.14*s,h=.9*s;
   c.fillStyle='rgba(0,0,0,.35)';c.beginPath();c.ellipse(x,y,w*1.5,.035*s,0,0,TAU);c.fill();
   if(q.ph==='gone'){c.fillStyle='rgba(40,30,25,.8)';c.beginPath();c.ellipse(x,y-.01*s,w*1.1,.025*s,0,0,TAU);c.fill();sl.fuse=null;return;}
   const on=q.ph==='fount',p=on?1-q.t/FT:0,cx=x+(on?rr(-1,1)*p*2.2:0),top=y-h;
   // blue tube
   let g=c.createLinearGradient(cx-w,0,cx+w,0);g.addColorStop(0,'#1e3a8a');g.addColorStop(.35,'#3b82f6');g.addColorStop(.55,'#93c5fd');g.addColorStop(1,'#1e3a8a');
   c.fillStyle=g;c.fillRect(cx-w,top,w*2,h);
   // dark base band
   g=c.createLinearGradient(cx-w,0,cx+w,0);g.addColorStop(0,'#111827');g.addColorStop(.45,'#4b5563');g.addColorStop(1,'#111827');
   c.fillStyle=g;c.fillRect(cx-w,y-h*.09,w*2,h*.09);c.beginPath();c.ellipse(cx,y,w,.02*s,0,0,TAU);c.fill();
   // yellow badge with star
   const bx=cx,by=y-h*.42,br=w*.55;c.fillStyle='#facc15';c.beginPath();c.arc(bx,by,br,0,TAU);c.fill();
   c.fillStyle='#dc2626';c.beginPath();for(let i=0;i<10;i++){const r=i&1?br*.32:br*.75,a=-Math.PI/2+i*Math.PI/5;c.lineTo(bx+Math.cos(a)*r,by+Math.sin(a)*r);}c.closePath();c.fill();
   // sparkle dots like the wrapper print
   c.fillStyle='rgba(255,255,255,.85)';for(const [u,v] of [[-.6,.08],[.55,.18],[-.5,.78],[.6,.6]]){c.beginPath();c.arc(cx+u*w,top+v*h,Math.max(.8,w*.06),0,TAU);c.fill();}
   if(!on&&!lit){c.fillStyle='rgba(6,8,14,.45)';c.fillRect(cx-w,top,w*2,h+.02*s);}   // unlit = dimmer
   if(on){c.fillStyle='#1f2937';c.beginPath();c.ellipse(cx,top,w,.025*s,0,0,TAU);c.fill();
     c.fillStyle='#000';c.beginPath();c.ellipse(cx,top,w*.45,.012*s,0,0,TAU);c.fill();
     sl.fuse={x:cx,y:top};sl.hot={x:cx,y:top};
     const hue=(Math.floor(performance.now()/700)&1)?125:45;
     fountain(S,sl,k,cx,top,3+p*4,hue,.65+.45*p);
     if(p>.85)lighter(c,()=>glow(c,cx,y-.45*s,s*.55*p,'rgba(255,200,120,.35)'));
   }else{
     // crinkled purple paper twist on top
     const pd=lit?['#4c1d95','#6d28d9','#8b5cf6','#c4b5fd']:['#241743','#33235e','#46337a','#5d4a92'];
     const blobs=[[-.75,.02,.42],[.75,.03,.42],[-.35,-.06,.5],[.35,-.05,.5],[0,-.12,.48],[-.55,-.1,.32],[.55,-.11,.3],[0,.03,.55]];
     for(let i=0;i<blobs.length;i++){const [u,v,r]=blobs[i];c.fillStyle=pd[i%3];c.beginPath();c.ellipse(cx+u*w,top+v*s,r*w,r*w*.8,i,0,TAU);c.fill();}
     c.strokeStyle=pd[3];c.lineWidth=Math.max(1,w*.06);c.lineCap='round';           // crinkle highlights
     for(const [u,v,l,a] of [[-.5,-.08,.35,-.6],[.2,-.14,.3,.5],[.55,-.04,.28,-.3],[-.15,-.02,.25,.9],[.05,-.2,.2,-.2]]){
       c.beginPath();c.moveTo(cx+u*w,top+v*s);c.lineTo(cx+u*w+Math.cos(a)*l*w,top+v*s+Math.sin(a)*l*w);c.stroke();}
     sl.fuse=fuse(c,cx+w*.3,top-.08*s,s,1);}
 }},

 chakri:{name:'Chakri',frame(S,sl,k,lit){const c=S.c,{x,y,s,fs}=sl,q=sl.st,cy=y-1.2*s,r=.22*s,m=fs?2.2:1;
   q.a=(q.a||0)+(lit?.3*k:0);
   const cols=lit?['#16a34a','#dc2626','#facc15']:['#1e3a24','#3b1414','#4a4016'];
   for(let i=0;i<6;i++){c.fillStyle=cols[i%3];c.beginPath();c.moveTo(x,cy);c.arc(x,cy,r,q.a+i*TAU/6,q.a+(i+1)*TAU/6);c.closePath();c.fill();}
   c.strokeStyle='rgba(0,0,0,.35)';c.lineWidth=Math.max(1,.01*s);c.beginPath();c.arc(x,cy,r*.68,0,TAU);c.stroke();c.beginPath();c.arc(x,cy,r*.38,0,TAU);c.stroke();
   c.fillStyle=lit?'#e7e5e4':'#57534e';c.beginPath();c.arc(x,cy,r*.16,0,TAU);c.fill();
   const fa=q.a+.3;sl.fuse={x:x+Math.cos(fa)*r*1.08,y:cy+Math.sin(fa)*r*1.08};
   if(lit){sl.hot={x,y:cy};q.ht=(q.ht||0)-k;if(q.ht<=0){q.hue=[45,120,0,190,300][R()*5|0];q.ht=110;}
     for(let j=0;j<2;j++){const a0=q.a+j*Math.PI,px=x+Math.cos(a0)*r,py=cy+Math.sin(a0)*r;
       for(let i=amt(k,3,s);i--;){const v=s*rr(.016,.036)*m,tx=Math.sin(a0),ty=-Math.cos(a0);
         S.spark(px,py,tx*v+rr(-.004,.004)*s,ty*v+rr(-.004,.004)*s,{g:s*.0005,life:rr(25,50)*(fs?1.7:1),max:50*(fs?1.7:1),h:q.hue+rr(-8,8),l:rr(60,80),w:rr(1,2)});}}
     lighter(c,()=>glow(c,x,cy,r*1.7,'rgba(255,230,170,.55)'));}}},

 rocket:{name:'Rocket',frame(S,sl,k,lit){const c=S.c,{x,y,s,fs}=sl,q=sl.st;
   // ONE rocket each time focus crosses the threshold; it flies off at a random angle
   if(!q.ph){q.ph='idle';q.pl=lit;}
   if(lit&&!q.pl&&q.ph==='idle'){q.ph='fly';q.a=rr(-1,1)*1.0;q.v=s*.01;q.rx=x;q.ry=y-.5*s;q.d=0;
     q.len=fs?Math.max(1.8*s,Math.min(S.H*rr(.5,.75),y-S.H*.16)):1.7*s;}
   if(!lit&&q.ph==='done')q.ph='idle';                   // back in the bottle, ready for the next crossing
   q.pl=lit;
   if(q.ph==='fly'){const vmax=fs?Math.max(s*.05,S.H*.014):s*.05;q.v=Math.min(q.v+vmax/25*k,vmax);q.a+=Math.sin(performance.now()/90)*.012*k;
     const dx=Math.sin(q.a)*q.v*k,dy=-Math.cos(q.a)*q.v*k;q.rx+=dx;q.ry+=dy;q.d+=q.v*k;
     for(let i=amt(k,4,s);i--;)S.spark(q.rx-Math.sin(q.a)*.05*s,q.ry+Math.cos(q.a)*.05*s,-Math.sin(q.a)*s*rr(.004,.014)+rr(-.003,.003)*s,Math.cos(q.a)*s*rr(.004,.014),{g:s*.0003,life:rr(14,30),max:30,h:rr(30,45),l:rr(60,80),w:rr(1,1.8),bn:false});
     const out=q.rx<S.W*.12||q.rx>S.W*.88||q.ry<S.H*.16;
     if(q.d>=q.len||out){const bx=q.rx+Math.sin(q.a)*.4*s,by=q.ry-Math.cos(q.a)*.4*s;
       S.burst(bx,by,fs?Math.min(S.W,S.H)*rr(.2,.3):.7*s,fs?180:amt(1,130,s));S.burst(bx,by,fs?Math.min(S.W,S.H)*.12:.4*s,fs?80:50);q.ph='done';}}
   c.fillStyle='rgba(0,0,0,.35)';c.beginPath();c.ellipse(x,y,.14*s,.03*s,0,0,TAU);c.fill();
   const drawRocket=(rx,ry,a,on)=>{c.save();c.translate(rx,ry);c.rotate(a);
     c.strokeStyle='#a16207';c.lineWidth=Math.max(1,.014*s);c.beginPath();c.moveTo(.03*s,0);c.lineTo(.03*s,.42*s);c.stroke();
     const g=c.createLinearGradient(-.04*s,0,.04*s,0);g.addColorStop(0,on?'#991b1b':'#3b1414');g.addColorStop(.5,on?'#f87171':'#6b2a2a');g.addColorStop(1,on?'#991b1b':'#3b1414');
     c.fillStyle=g;c.fillRect(-.04*s,-.3*s,.08*s,.3*s);
     c.fillStyle=on?'#facc15':'#5f5220';c.fillRect(-.04*s,-.2*s,.08*s,.04*s);
     c.beginPath();c.moveTo(-.05*s,-.3*s);c.lineTo(.05*s,-.3*s);c.lineTo(0,-.42*s);c.closePath();c.fill();c.restore();};
   if(q.ph==='idle'){drawRocket(x,y-.5*s,0,lit);sl.fuse={x:x+.03*s,y:y-.48*s};}
   else{sl.fuse=null;if(q.ph==='fly'){sl.hot={x:q.rx,y:q.ry};drawRocket(q.rx,q.ry,q.a,true);lighter(c,()=>glow(c,q.rx,q.ry,.16*s,'rgba(255,220,150,.9)'));}}
   c.fillStyle='rgba(150,190,255,.07)';c.strokeStyle='rgba(200,220,255,.35)';c.lineWidth=Math.max(1,.008*s);
   c.beginPath();c.moveTo(x-.1*s,y);c.lineTo(x-.1*s,y-.22*s);c.quadraticCurveTo(x-.1*s,y-.3*s,x-.035*s,y-.33*s);c.lineTo(x-.035*s,y-.42*s);
   c.lineTo(x+.035*s,y-.42*s);c.lineTo(x+.035*s,y-.33*s);c.quadraticCurveTo(x+.1*s,y-.3*s,x+.1*s,y-.22*s);c.lineTo(x+.1*s,y);c.closePath();c.fill();c.stroke();}},

 sparkler:{name:'Phuljhadi',frame(S,sl,k,lit){const c=S.c,{x,y,s,fs}=sl,m=fs?1.8:1,bx=x-.16*s,by=y-.06*s,tx=x+.12*s,ty=y-1.95*s,mx=bx+(tx-bx)*.38,my=by+(ty-by)*.38;
   c.lineCap='round';c.strokeStyle='#9ca3af';c.lineWidth=Math.max(1,.012*s);c.beginPath();c.moveTo(bx,by);c.lineTo(mx,my);c.stroke();
   c.strokeStyle=lit?'#57534e':'#3f3b38';c.lineWidth=Math.max(2,.035*s);c.beginPath();c.moveTo(mx,my);c.lineTo(tx,ty);c.stroke();
   sl.fuse={x:tx,y:ty};
   if(lit){sl.hot={x:tx,y:ty};for(let i=amt(k,7,s);i--;){const a=R()*TAU,big=R()<.1,v=s*m*(big?rr(.04,.06):rr(.008,.045));
       S.spark(tx,ty,Math.cos(a)*v,Math.sin(a)*v,{g:s*.0004,life:(big?rr(16,24):rr(5,15))*(fs?1.5:1),max:(big?24:15)*(fs?1.5:1),h:rr(40,55),l:rr(75,95),w:rr(.8,1.6)});}
     lighter(c,()=>{glow(c,tx,ty,s*.32,'rgba(255,250,235,.8)');glow(c,tx,ty,s*.07,'rgba(255,255,255,1)');});}}},

 skyshot:{name:'Sky shot',frame(S,sl,k,lit){const c=S.c,{x,y,s,fs}=sl,q=sl.st,bw=.27*s,bh=.34*s;
   if(!q.b){q.b=[];q.cd=0;}
   c.fillStyle='rgba(0,0,0,.35)';c.beginPath();c.ellipse(x,y,bw*1.15,.035*s,0,0,TAU);c.fill();
   c.fillStyle=lit?'#b91c1c':'#3b1414';c.fillRect(x-bw,y-bh,bw*2,bh);
   c.fillStyle=lit?'#facc15':'#5f5220';c.fillRect(x-bw,y-bh*.72,bw*2,bh*.12);c.fillRect(x-bw,y-bh*.36,bw*2,bh*.12);
   c.fillStyle='#1c1917';for(const o of[-.16,0,.16]){c.beginPath();c.ellipse(x+o*s,y-bh,.055*s,.02*s,0,0,TAU);c.fill();}
   sl.fuse=fuse(c,x+bw,y-.12*s,s,1);
   if(lit){sl.hot={x,y:y-bh};q.cd-=k;if(q.cd<=0){const o=[-.16,0,.16][R()*3|0],bx=x+o*s,by=y-bh;
       const g=fs?S.H*.0004:s*.0006, hT=fs?Math.max(2*s,by-S.H*rr(.06,.35)):s*rr(1.5,1.9);
       q.b.push({x:bx,y:by,vx:rr(-.004,.004)*s*(fs?2:1),vy:-Math.sqrt(2*g*hT),g});q.cd=rr(28,46);
       lighter(c,()=>glow(c,bx,by,.15*s,'rgba(255,230,170,.9)'));}}
   for(let i=q.b.length-1;i>=0;i--){const b=q.b[i];b.vy+=b.g*k;b.x+=b.vx*k;b.y+=b.vy*k;
     S.spark(b.x,b.y,rr(-.003,.003)*s,s*.004,{g:0,life:rr(10,20),max:20,h:40,l:72,w:1.3,bn:false});
     lighter(c,()=>glow(c,b.x,b.y,.05*s,'rgba(255,240,200,1)'));
     if(b.vy>-b.g*3){S.burst(b.x,b.y,fs?Math.min(S.W,S.H)*rr(.14,.24):.6*s,fs?140:amt(1,100,s));q.b.splice(i,1);}}}}
};
let cur=store.get('fp.cracker','anaar'); if(!CR[cur]) cur='anaar';
const fsOn=true;   // sparks always fly over the whole page

const fx=new Stage($('fx'),2600), slot={st:{}};
let cardLit=false, cardPending=false, testUntil=0, prevLit=false;
const isLit=()=>cardLit||performance.now()<testUntil;
function pick(c){cur=c;store.set('fp.cracker',c);slot.st={};
  document.querySelectorAll('#picks button').forEach(b=>b.classList.toggle('active',b.dataset.c===c));
  paintLabel(lastD);}
document.querySelectorAll('#picks button').forEach(b=>b.onclick=()=>pick(b.dataset.c));

function ignite(){                       // the moment the cracker catches fire
  fx.fs=fsOn;
  if(fsOn){fx.burst(slot.x,slot.y-slot.s*.9,Math.min(fx.W,fx.H)*.2,120);}
  else fx.burst(slot.x,slot.y-slot.s*1.2,slot.s*.7,90);
}

/* ---------- animation loop (browser only, never touches the device) ---------- */
let lastT=performance.now();
function frame(t){
  const k=Math.min(3,(t-lastT)/16.67); lastT=t;
  const c=fx.c; c.clearRect(0,0,fx.W,fx.H);
  const box=$('art').getBoundingClientRect();
  if(box.width>4&&box.height>4){
    Object.assign(slot,{x:box.left+box.width/2,y:box.bottom-4,s:Math.min(box.height/2.75,box.width/1.25),fs:fsOn});
    fx.fs=fsOn; fx.surf=fsOn?[...document.querySelectorAll('.card')].map(e=>e.getBoundingClientRect()):null;
    const lit=isLit(); if(lit&&!prevLit) ignite(); prevLit=lit;
    if(!fsOn){c.save();c.beginPath();c.rect(box.left,box.top,box.width,box.height);c.clip();}
    slot.hot=null;
    CR[cur].frame(fx,slot,k,lit);
    if(slot.hot){const h=slot.hot;lighter(c,()=>glow(c,h.x,h.y,slot.s*.85,'rgba(251,146,60,.22)'));}
    if(cardPending&&!lit&&slot.fuse){const f=slot.fuse;for(let i=0;i<2;i++){const a=R()*TAU,v=slot.s*rr(.004,.012);fx.spark(f.x,f.y,Math.cos(a)*v,Math.sin(a)*v,{g:slot.s*.0004,life:rr(8,18),max:18,h:42,l:75,w:1,bn:false});}}
    fx.run(k);
    if(!fsOn)c.restore();
  }
  requestAnimationFrame(frame);
}

/* ================= data from NPG ================= */
function paintLabel(d){
  const test=performance.now()<testUntil;
  if(!d&&!test)return;
  const on=test||(d.xiao&&d.xiaoLed);
  $('cLabel').textContent=on?CR[cur].name.toUpperCase()+' LIT!':'NOT LIT';
  $('cLabel').classList.toggle('on',on);
  $('cSub').textContent=test?'Test (page only)':d.xiao?(d.mode===1?'Forced on':d.mode===2?'Forced off':'Auto'):(d.led?'XIAO not connected':'Waiting for XIAO');
}
function update(d){
  const first=!lastD; lastD=d;
  if(!editing && Math.abs(d.th-th)>0.01) setTh(d.th,false);
  setModeUI(d.mode);
  const above=d.focus>=th;
  $('focusNum').textContent=d.focus.toFixed(1);
  setGauge(d.focus,d.led);
  const s=$('focusState'); s.textContent=(d.test?'Button test \u00b7 ':'')+(above?'Focused':'Below threshold'); s.classList.toggle('hot',above);

  $('xiaoDot').className='dot'+(d.xiao?(d.pps>=5?' ok':' warn'):'');
  $('xiaoMeta').textContent=d.xiao?' · '+d.pps+'/s':' offline';
  const on=d.xiao&&d.xiaoLed;
  if(first) prevLit=on;                      // no ignition effect on page load
  cardLit=on; cardPending=d.led&&!d.xiao;
  $('art').classList.toggle('lit',on||performance.now()<testUntil);
  paintLabel(d);

  const now=Date.now();
  hist.push({t:now,v:d.focus,on:d.led});
  while(hist.length&&now-hist[0].t>WIN) hist.shift();
  st.sum+=d.focus; st.n++; st.peak=Math.max(st.peak,d.focus); if(d.led) st.above++;
  $('stAvg').textContent=(st.sum/st.n).toFixed(1)+'%';
  $('stPeak').textContent=st.peak.toFixed(1)+'%';
  $('stAbove').textContent=Math.round(100*st.above/st.n)+'%';
  draw();
}
function netUI(ok,d){
  const dot=$('npgDot');
  if(!ok){dot.className='dot';$('npgMeta').textContent=' · no reply';return;}
  const slow=rtt>400||(d&&d.hz&&d.hz<480);
  dot.className='dot '+(slow?'warn':'ok');
  $('npgMeta').textContent=' · '+Math.round(rtt)+' ms'+(d&&d.hz?' · '+d.hz+' Hz':'');
}

const cv=$('chart'),cx=cv.getContext('2d');
function draw(){
  const dpr=Math.min(2,window.devicePixelRatio||1),w=cv.clientWidth,h=cv.clientHeight;
  if(w<10||h<10) return;
  if(cv.width!==Math.round(w*dpr)||cv.height!==Math.round(h*dpr)){cv.width=Math.round(w*dpr);cv.height=Math.round(h*dpr);}
  cx.setTransform(dpr,0,0,dpr,0,0); cx.clearRect(0,0,w,h);
  let peak=th*1.5; for(const p of hist) if(p.v>peak) peak=p.v;
  const max=Math.max(10,peak*1.1), L=34, Rr=w-8, now=Date.now();
  const X=t=>L+(Rr-L)*(1-(now-t)/WIN), Y=v=>h-8-(h-16)*v/max;
  cx.font='11px system-ui'; cx.lineWidth=1;
  const rows=h<90?2:4;
  for(let k=0;k<=rows;k++){const v=max*k/rows,y=Y(v);cx.strokeStyle='#1f2534';cx.beginPath();cx.moveTo(L,y);cx.lineTo(Rr,y);cx.stroke();cx.fillStyle='#6b7390';cx.fillText(v.toFixed(0)+'%',2,y+4);}
  cx.setLineDash([6,5]);cx.strokeStyle='#fbbf24';cx.lineWidth=1.5;cx.beginPath();cx.moveTo(L,Y(th));cx.lineTo(Rr,Y(th));cx.stroke();cx.setLineDash([]);
  if(hist.length<2) return;
  const g=cx.createLinearGradient(0,0,0,h);g.addColorStop(0,'rgba(139,92,246,.35)');g.addColorStop(1,'rgba(139,92,246,0)');
  cx.beginPath();cx.moveTo(X(hist[0].t),Y(0));for(const p of hist)cx.lineTo(X(p.t),Y(p.v));cx.lineTo(X(hist[hist.length-1].t),Y(0));cx.closePath();cx.fillStyle=g;cx.fill();
  cx.lineWidth=2.5;cx.lineJoin='round';cx.lineCap='round';
  for(let i=1;i<hist.length;i++){cx.strokeStyle=hist[i].on?'#fbbf24':'#a78bfa';cx.beginPath();cx.moveTo(X(hist[i-1].t),Y(hist[i-1].v));cx.lineTo(X(hist[i].t),Y(hist[i].v));cx.stroke();}
}

/* one request at a time, 1.5 s timeout, so a slow reply can never pile up */
async function poll(){
  const t0=performance.now(), ac=new AbortController(), to=setTimeout(()=>ac.abort(),1500);
  try{
    const r=await fetch(base+'/data',{cache:'no-store',signal:ac.signal}); const d=await r.json();
    const dt=performance.now()-t0; rtt=rtt?rtt*.7+dt*.3:dt; fails=0;
    if(!base&&location.hostname.endsWith('.local')&&d.ip) base='http://'+d.ip;   // skip repeated .local lookups
    update(d); netUI(true,d);
  }catch(e){ if(++fails>=3&&base) base=''; netUI(false); }
  finally{ clearTimeout(to); setTimeout(poll,150); }
}

new ResizeObserver(()=>{fx.resize();draw();}).observe(document.body);
fx.resize(); pick(cur); setTh(8,false); requestAnimationFrame(frame); poll();
</script></body></html>

)HTML";
