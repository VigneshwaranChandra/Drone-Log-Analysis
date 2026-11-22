"""Self-contained 3D flight replay (single HTML file, no internet needed).

Plain canvas + a tiny perspective projection, so it opens offline in any browser.
Drag = rotate, wheel = zoom, space = play/pause.
"""
from __future__ import annotations

import json

import numpy as np

TEMPLATE = r"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Flight Replay 3D</title>
<style>
:root{--bg:#0f1418;--fg:#e8eef2;--mut:#8aa0ad;--acc:#4fc3f7}
body{margin:0;background:var(--bg);color:var(--fg);font:13px system-ui,Segoe UI,Arial}
#top{display:flex;gap:10px;align-items:center;padding:8px 12px;flex-wrap:wrap}
#c{display:block;width:100%;height:calc(100vh - 150px);cursor:grab}
input[type=range]{flex:1;min-width:200px}
button,select{background:#1f2a31;color:var(--fg);border:1px solid #33444f;border-radius:6px;padding:5px 10px}
#hud{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:4px 14px;padding:6px 12px;color:var(--mut)}
#hud b{color:var(--fg)} .bad{color:#ff6b6b!important}
</style></head><body>
<div id="top"><b>__TITLE__</b><button id="pp">Play</button>
<select id="sp"><option>0.25</option><option>0.5</option><option selected>1</option><option>2</option><option>5</option></select>x
<input id="sl" type="range" min="0" max="1000" value="0"><span id="tt"></span>
<button id="fol">Follow: off</button></div>
<canvas id="c"></canvas><div id="hud"></div>
<script>
const D=__DATA__;
const N=D.t.length,cv=document.getElementById('c'),cx=cv.getContext('2d');
let i=0,play=false,speed=1,az=-0.7,el=0.45,zoom=1,follow=false,last=null;
const ext=Math.max(10,...D.e.map(Math.abs),...D.n.map(Math.abs),...D.u.map(Math.abs));
function resize(){cv.width=cv.clientWidth*devicePixelRatio;cv.height=cv.clientHeight*devicePixelRatio}
addEventListener('resize',resize);resize();
function proj(x,y,z,c){ // x=E,y=N,z=U relative to center c
 x-=c[0];y-=c[1];z-=c[2];
 const ca=Math.cos(az),sa=Math.sin(az),ce=Math.cos(el),se=Math.sin(el);
 let X=x*ca-y*sa,Y=x*sa+y*ca; let Z=z*ce-Y*se,Dp=Y*ce+z*se;
 const s=Math.min(cv.width,cv.height)/(2.6*ext/zoom),f=1/(1+Dp/(6*ext/zoom));
 return [cv.width/2+X*s*f,cv.height/2-Z*s*f];}
function rot(r,p,y){r*=Math.PI/180;p*=Math.PI/180;y*=Math.PI/180;
 const cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(p),sp=Math.sin(p),cy=Math.cos(y),sy=Math.sin(y);
 return [[cp*cy,sr*sp*cy-cr*sy,cr*sp*cy+sr*sy],[cp*sy,sr*sp*sy+cr*cy,cr*sp*sy-sr*cy],[-sp,sr*cp,cr*cp]];}
function line(a,b,col,w){cx.strokeStyle=col;cx.lineWidth=w;cx.beginPath();cx.moveTo(a[0],a[1]);cx.lineTo(b[0],b[1]);cx.stroke()}
function draw(){
 cx.fillStyle='#0f1418';cx.fillRect(0,0,cv.width,cv.height);
 const c=follow?[D.e[i],D.n[i],D.u[i]]:[0,0,0];
 // ground grid
 const st=Math.pow(10,Math.floor(Math.log10(ext/2)));
 for(let g=-Math.ceil(ext/st)*st;g<=ext;g+=st){line(proj(g,-ext,0,c),proj(g,ext,0,c),'#1c262d',1);line(proj(-ext,g,0,c),proj(ext,g,0,c),'#1c262d',1)}
 line(proj(0,0,0,c),proj(0,ext*0.25,0,c),'#e57373',2);cx.fillStyle='#e57373';const nn=proj(0,ext*0.28,0,c);cx.fillText('N',nn[0],nn[1]);
 // track
 for(let k=1;k<N;k++){const a=proj(D.e[k-1],D.n[k-1],D.u[k-1],c),b=proj(D.e[k],D.n[k],D.u[k],c);
  line(a,b,k<=i?(D.ph[k]=="UPSET"?"#ff5252":"#4fc3f7"):"#3a4a55",k<=i?2:1)}
 // shadow
 const sh=proj(D.e[i],D.n[i],0,c),pp=proj(D.e[i],D.n[i],D.u[i],c);line(sh,pp,'#44535c',1);
 // drone: NED body FRD -> world ENU
 const R=rot(D.r[i],D.p[i],D.y[i]),L=Math.max(ext*0.1,2);
 function w(bx,by,bz){const v=[R[0][0]*bx+R[0][1]*by+R[0][2]*bz,R[1][0]*bx+R[1][1]*by+R[1][2]*bz,R[2][0]*bx+R[2][1]*by+R[2][2]*bz];
   return proj(D.e[i]+v[1]*L,D.n[i]+v[0]*L,D.u[i]-v[2]*L,c)}
 const ctr=w(0,0,0);
 D.mpos.forEach((m,k)=>{const tip=w(m[0],m[1],0);const lvl=D.m[k]?D.m[k][i]:0;
   const col=`hsl(${120-120*lvl},85%,55%)`;line(ctr,tip,'#cfd8dc',3);cx.fillStyle=col;cx.beginPath();cx.arc(tip[0],tip[1],7*devicePixelRatio,0,7);cx.fill();
   cx.fillStyle='#fff';cx.fillText('M'+(k+1),tip[0]+9,tip[1]-6)});
 line(ctr,w(1.6,0,0),'#ffd54f',3); // nose
 line(ctr,w(0,0,-1.2),'#81c784',2); // thrust (up)
 // hud
 const h=document.getElementById('hud');
 h.innerHTML=`<div>t <b>${D.t[i].toFixed(2)} s</b></div><div>phase <b class="${D.ph[i]=='UPSET'?'bad':''}">${D.ph[i]}</b></div>`+
 `<div>alt <b>${D.u[i].toFixed(1)} m</b></div><div>speed <b>${D.hs[i].toFixed(1)} m/s</b></div>`+
 `<div>roll <b>${D.r[i].toFixed(0)}</b> pitch <b>${D.p[i].toFixed(0)}</b> yaw <b>${D.y[i].toFixed(0)}</b></div>`+
 D.m.map((m,k)=>`<div>M${k+1} <b>${D.mraw[k][i].toFixed(0)}</b></div>`).join('');
 document.getElementById('tt').textContent=D.t[i].toFixed(1)+' / '+D.t[N-1].toFixed(1)+' s';
 document.getElementById('sl').value=Math.round(1000*i/(N-1));}
function tick(ts){if(play){if(last!=null){let tn=D.t[i]+(ts-last)/1000*speed;while(i<N-1&&D.t[i]<tn)i++;if(i>=N-1)play=false}last=ts}else last=null;draw();requestAnimationFrame(tick)}
document.getElementById('pp').onclick=()=>{play=!play;if(i>=N-1)i=0;document.getElementById('pp').textContent=play?'Pause':'Play'};
document.getElementById('sp').onchange=e=>speed=+e.target.value;
document.getElementById('sl').oninput=e=>{i=Math.round(e.target.value/1000*(N-1))};
document.getElementById('fol').onclick=e=>{follow=!follow;e.target.textContent='Follow: '+(follow?'on':'off')};
addEventListener('keydown',e=>{if(e.code=='Space'){e.preventDefault();document.getElementById('pp').click()}});
let drag=null;cv.onmousedown=e=>drag=[e.clientX,e.clientY,az,el];addEventListener('mouseup',()=>drag=null);
addEventListener('mousemove',e=>{if(drag){az=drag[2]+(e.clientX-drag[0])/200;el=Math.max(-0.2,Math.min(1.5,drag[3]+(e.clientY-drag[1])/200))}});
cv.onwheel=e=>{e.preventDefault();zoom*=e.deltaY<0?1.15:1/1.15};
cv.ontouchstart=e=>{const t=e.touches[0];drag=[t.clientX,t.clientY,az,el]};
cv.ontouchmove=e=>{e.preventDefault();const t=e.touches[0];if(drag){az=drag[2]+(t.clientX-drag[0])/200;el=Math.max(-0.2,Math.min(1.5,drag[3]+(t.clientY-drag[1])/200))}};
requestAnimationFrame(tick);
</script></body></html>"""

POS = {"front-right": (1, 1), "front-left": (1, -1), "rear-left": (-1, -1), "rear-right": (-1, 1)}


def build_replay(res, out_path, title="Flight replay"):
    fd, cfg = res.fd, res.cfg
    t, g = fd.grid(25.0)
    need = ("pos_n", "pos_e", "pos_u", "roll", "pitch", "yaw")
    if not all(k in g for k in need):
        return None
    ok = np.all([np.isfinite(g[k]) for k in need], axis=0)
    t = t[ok]
    sel = {k: g[k][ok] for k in g}
    names = fd.motor_names()
    pmin, pmax = cfg["airframe"]["pwm_min"], cfg["airframe"]["pwm_max"]
    lay = cfg["airframe"].get("motor_layout", {})
    ph = res.grid.get("_phase")
    phase = [str(ph[np.argmin(np.abs(res.grid_t - x))]) for x in t] if ph is not None else ["-"] * len(t)
    r2 = lambda a: [round(float(x), 2) for x in np.nan_to_num(a)]
    data = {
        "t": r2(t), "e": r2(sel["pos_e"]), "n": r2(sel["pos_n"]), "u": r2(sel["pos_u"]),
        "r": r2(sel["roll"]), "p": r2(sel["pitch"]), "y": r2(sel["yaw"]),
        "hs": r2(np.hypot(sel.get("vel_n", 0 * t), sel.get("vel_e", 0 * t))),
        "m": [r2(np.clip((sel[n] - pmin) / (pmax - pmin), 0, 1)) for n in names],
        "mraw": [r2(sel[n]) for n in names],
        "mpos": [list(np.array(POS.get(lay.get(n, ""), (0, 0))) * 0.7071) for n in names],
        "ph": phase,
    }
    html = TEMPLATE.replace("__DATA__", json.dumps(data, separators=(",", ":"))).replace("__TITLE__", title)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    return out_path
