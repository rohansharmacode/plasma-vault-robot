"""Live mission dashboard for Team PLASMA: http://localhost:8765
Started by:  python plasma_vault.py --dashboard [--camera] --mission <anything>
Shows exactly what the robot sees and decides: camera, distance, tape offset, colours, QR, AprilTags,
sound direction, battery/tilt, evidence + Gemma verdict, what it is saying, and the full log."""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>PLASMA Mission Control</title>
<style>
:root{--bg:#0b0f14;--panel:#121923;--line:#1f2a38;--txt:#d8e2ee;--dim:#7d8ea3;--acc:#ff5a1f;--ok:#2ecc71;--bad:#ff4d4d;--warn:#ffb020}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--txt);font:14px/1.4 system-ui,Segoe UI,sans-serif}
header{display:flex;align-items:center;gap:16px;padding:12px 16px;border-bottom:1px solid var(--line)}
h1{margin:0;font-size:18px;letter-spacing:.08em}h1 b{color:var(--acc)}
#mission{padding:4px 10px;border-radius:6px;background:var(--acc);color:#000;font-weight:700}
#say{flex:1;color:var(--dim);font-style:italic;overflow:hidden;white-space:nowrap;text-overflow:ellipsis}
#clock{font-variant-numeric:tabular-nums;color:var(--dim)}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:12px;padding:12px 16px}
.p{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px;min-width:0}
.p h2{margin:0 0 8px;font-size:12px;letter-spacing:.12em;color:var(--dim);text-transform:uppercase}
.big{font-size:40px;font-weight:700;font-variant-numeric:tabular-nums}
.bar{position:relative;height:16px;background:#0e141c;border-radius:8px;overflow:hidden;margin-top:6px}
.bar i{position:absolute;top:0;bottom:0;background:var(--acc)}
.mid{position:absolute;left:50%;top:0;bottom:0;width:2px;background:#fff6}
table{width:100%;border-collapse:collapse;font-size:13px}td{padding:3px 4px;border-bottom:1px solid var(--line);word-break:break-all}
.sw{display:inline-block;width:12px;height:12px;border-radius:50%;vertical-align:-2px;margin-right:6px}
#cam{position:relative;width:100%;aspect-ratio:4/3;background:#000;border-radius:8px;overflow:hidden}
#cam img,#cam canvas{position:absolute;inset:0;width:100%;height:100%}
#log{height:260px;overflow:auto;font:12px/1.35 ui-monospace,Consolas,monospace;white-space:pre-wrap;background:#080b10;padding:8px;border-radius:6px}
.core{font-size:30px;font-weight:800;text-transform:uppercase}
.wide{grid-column:1/-1}.dim{color:var(--dim)}
</style></head><body>
<header><h1>TEAM <b>PLASMA</b> · MISSION CONTROL</h1><span id="mission">IDLE</span><span id="say"></span><span id="clock"></span></header>
<main>
<section class="p"><h2>Camera</h2><div id="cam"><img id="img" alt=""><canvas id="ov" width="640" height="480"></canvas></div>
<div class="dim" id="camnote">start with --camera to stream video. Boxes = what the vision models detect.</div></section>
<section class="p"><h2>Distance sensor</h2><div class="big" id="dist">--</div><div class="bar"><i id="distbar"></i></div>
<div class="dim">target 15–25 cm (green zone)</div></section>
<section class="p"><h2>Tape tracking</h2><div class="big" id="off">--</div><div class="bar"><span class="mid"></span><i id="offbar"></i></div>
<div id="ltype" class="dim"></div></section>
<section class="p"><h2>Robot</h2><table id="robot"></table></section>
<section class="p"><h2>Colours seen</h2><table id="colors"></table></section>
<section class="p"><h2>QR codes / AprilTags</h2><table id="qr"></table></section>
<section class="p"><h2>Evidence → core</h2><div class="core" id="core">?</div><div id="why" class="dim"></div><table id="ev"></table></section>
<section class="p wide"><h2>Live log</h2><div id="log"></div></section>
</main>
<script>
const $=id=>document.getElementById(id);const SW={red:'#ff4d4d',blue:'#3d8bff',green:'#2ecc71',yellow:'#ffd400',orange:'#ff9f1c',purple:'#b06cff'};
let camOn=false;
async function tick(){
 try{const s=await (await fetch('/state')).json();
  $('mission').textContent=s.mission||'IDLE';$('say').textContent=s.say?('🔊 '+s.say):'';$('clock').textContent=s.t;
  const d=s.dist;$('dist').textContent=d==null?'--':d.toFixed(1)+' cm';
  const w=d==null?0:Math.max(0,Math.min(100,d/150*100));const db=$('distbar');db.style.left='0';db.style.width=w+'%';
  db.style.background=(d!=null&&d>=15&&d<=25)?'var(--ok)':'var(--acc)';
  let L=s.line||{};if(s.tape&&s.eye_active){const T=s.tape;const v=T.near!=null?T.near:(T.mid!=null?T.mid:null);
   L={offset:v==null?null:Math.round(v*320),type:v==null?0:(T.fork?2:1)};}
  const o=L.offset;$('off').textContent=o==null?'--':o;
  const ob=$('offbar');if(o!=null){const p=Math.max(-320,Math.min(320,o))/320*50;ob.style.left=(p<0?50+p:50)+'%';ob.style.width=Math.abs(p)+'%';}
  $('ltype').textContent=['no tape','tape','Y-fork','crossroad'][L.type]!==undefined?('detected: '+['no tape','tape','Y-fork','crossroad'][L.type]):'';
  const r=s.robot||{};$('robot').innerHTML=Object.entries(r).map(([k,v])=>`<tr><td class=dim>${k}</td><td>${v}</td></tr>`).join('');
  $('colors').innerHTML=(s.colors||[]).map(c=>`<tr><td><span class=sw style="background:${SW[c.color]||'#888'}"></span>${c.color} ${c.shape||''}</td><td>x ${Math.round(c.cx)} y ${Math.round(c.cy)}</td><td>area ${Math.round(c.area)}</td></tr>`).join('')||'<tr><td class=dim>none</td></tr>';
  $('qr').innerHTML=[...(s.qr||[]).map(q=>`<tr><td>QR</td><td>${esc(q.text)}</td></tr>`),...(s.tags||[]).map(t=>`<tr><td>Tag ${t.id}</td><td>dist ${t.dist} · area ${Math.round(t.area)}</td></tr>`)].join('')||'<tr><td class=dim>none</td></tr>';
  const e=s.evidence||{};$('core').textContent=e.core||'?';$('core').style.color=SW[e.core]||'var(--txt)';$('why').textContent=e.why||'';
  $('ev').innerHTML=(e.fragments||[]).map((f,i)=>`<tr><td>#${i+1}</td><td><span class=dim>${esc(f.raw)}</span><br>→ ${esc(f.decoded)} <span class=dim>(${f.method})</span></td></tr>`).join('')+(e.gemma?`<tr><td>Gemma 4</td><td>${e.gemma.core} (${Math.round((e.gemma.confidence||0)*100)}%) – ${esc(e.gemma.reasoning||'')}</td></tr>`:'');
  const lg=$('log');const atBottom=lg.scrollTop+lg.clientHeight>=lg.scrollHeight-20;lg.textContent=(s.log||[]).join('\n');if(atBottom)lg.scrollTop=lg.scrollHeight;
  draw(s);
  if(s.camera&&!camOn){camOn=true;$('camnote').textContent='live camera';refreshCam();}
 }catch(e){$('mission').textContent='DISCONNECTED';}
}
function esc(t){return String(t==null?'':t).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}
function draw(s){const c=$('ov').getContext('2d');c.clearRect(0,0,640,480);c.lineWidth=3;c.font='16px sans-serif';
 for(const b of s.colors||[]){const r=Math.sqrt(b.area)/2;c.strokeStyle=SW[b.color]||'#fff';c.strokeRect(b.cx-r,b.cy-r,2*r,2*r);c.fillStyle=c.strokeStyle;c.fillText(b.color,b.cx-r,b.cy-r-4);}
 for(const q of [...(s.qr||[]),...(s.tags||[])]){const r=Math.sqrt(q.area)/2;c.strokeStyle='#fff';c.strokeRect(q.cx-r,q.cy-r,2*r,2*r);c.fillStyle='#fff';c.fillText(q.text?'QR':'tag '+q.id,q.cx-r,q.cy-r-4);}
 const T=s.tape;if(T){const B={near:[400,479],mid:[336,399],far:[272,335]};c.lineWidth=6;
  for(const k of ['near','mid','far']){c.strokeStyle='#ffd400';c.lineWidth=1;c.strokeRect(0,B[k][0],640,B[k][1]-B[k][0]);
   if(T[k]!=null){const x=(T[k]+1)/2*640;c.strokeStyle='#ff2020';c.lineWidth=6;c.beginPath();c.moveTo(x,B[k][0]);c.lineTo(x,B[k][1]);c.stroke();}}
  c.fillStyle='#ffd400';c.fillText('black-tape vision '+(T.fps||0)+' fps'+(T.fork?'  FORK':''),8,20);}
 else{const L=s.line||{};if(L.type){c.strokeStyle='#ff5a1f';c.beginPath();const x=320+Math.max(-320,Math.min(320,L.offset));c.moveTo(x,480);c.lineTo(x,300);c.stroke();}}
}
let lastUrl=null;
async function refreshCam(){try{const r=await fetch('/frame.jpg?'+Date.now());if(r.ok){const u=URL.createObjectURL(await r.blob());
 $('img').src=u;if(lastUrl)URL.revokeObjectURL(lastUrl);lastUrl=u;}}catch(e){}setTimeout(refreshCam,150);}
setInterval(tick,300);tick();
</script></body></html>"""


def start(state, got, evidence_file, chosen_core_fn, camera=False, port=8765, lan=False):
    """Run the dashboard server + a slow robot-status poller in background threads."""
    frame = {"jpg": None}
    state.setdefault("robot", {})

    def poll():
        while True:
            try:
                p = got.get_power_status() or [None]
                g = got.read_gyro_data()
                a = got.get_attitude()
                tilt = [n for n, f in zip(("forward", "back", "left", "right"), a[:4]) if f]
                state["robot"] = {"battery": "%s%%" % p[0], "pitch / roll / yaw": "%.1f / %.1f / %.1f" % tuple(g[:3]),
                                  "tilt": ", ".join(tilt) or "level",
                                  "sound angle": state.get("sound_angle")}
            except Exception as e:
                state["robot"] = {"status": "poll error: %s" % e}
            time.sleep(2)

    def cam():
        try:
            got.open_camera()
        except Exception as e:
            state["camera"] = False
            print("[dash] camera unavailable: %s" % e)
            return
        state["camera"] = True
        while True:
            if state.get("eye_active"):          # tape vision already pulls frames: reuse them
                time.sleep(0.2)
                continue
            try:
                data = got.read_camera_data()
                if data:
                    frame["jpg"] = data
            except Exception:
                pass
            time.sleep(0.15)

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.startswith("/state"):
                ev = {}
                try:
                    with open(evidence_file) as f:
                        ev = json.load(f)
                except Exception:
                    pass
                try:
                    core, why = chosen_core_fn()
                except Exception:
                    core, why = None, ""
                body = {k: v for k, v in dict(state).items()       # copy first: other threads add keys
                        if not isinstance(v, (bytes, bytearray))}      # camera frames go via /frame.jpg
                body["t"] = time.strftime("%H:%M:%S")
                body["evidence"] = {"fragments": [{k: fr.get(k) for k in ("raw", "decoded", "method")}
                                                  for fr in ev.get("fragments", [])],
                                    "gemma": ev.get("gemma"), "core": core if ev.get("fragments") else None,
                                    "why": why if ev.get("fragments") else "no evidence yet"}
                self._send(200, "application/json", json.dumps(body, default=str).encode())
            elif self.path.startswith("/frame.jpg"):
                jpg = state.get("frame_jpg") if state.get("eye_active") else frame["jpg"]
                if jpg:
                    self._send(200, "image/jpeg", jpg)
                else:
                    self._send(404, "text/plain", b"no frame")
            else:
                self._send(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))

    class Server(ThreadingHTTPServer):
        allow_reuse_address = False      # Windows would let an old run keep answering on the same port
    try:
        srv = Server(("0.0.0.0" if lan else "127.0.0.1", port), H)
    except OSError:
        print("[dash] port %d busy - close the previous run's window (Ctrl+C) to see this run" % port, flush=True)
        return None
    if lan:
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("10.255.255.255", 1))
            ip = s.getsockname()[0]
            s.close()
        except Exception:
            ip = "<this-laptop-ip>"
        print("[dash] open on your phone (same Wi-Fi): http://%s:%d" % (ip, port), flush=True)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    threading.Thread(target=poll, daemon=True).start()
    if camera:
        threading.Thread(target=cam, daemon=True).start()
    print("[dash] live dashboard: http://localhost:%d" % port, flush=True)
    return srv
