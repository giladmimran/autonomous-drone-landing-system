import os, sys, time, threading, subprocess
from pathlib import Path
from contextlib import asynccontextmanager
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from vision_api import router as vision_router, service as vision_service
import urllib.request, json

BASE = Path(__file__).resolve().parent
CHILD_PORT = 8001
MODULES = {
    'dashboard': ('main:app', '/'),
    'radio': ('radio:app', '/'),
    'parameters': ('radio:app', '/parameters'),
    'calibration': ('calibration:app', '/'),
    'firmware': ('firmware_app:app', '/'),
    'frame': ('frame_setup:app', '/'),
}
active = {'name': None, 'process': None, 'since': None, 'error': None}
lock = threading.RLock()


def stop_child():
    p = active.get('process')
    if p and p.poll() is None:
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill(); p.wait(timeout=3)
    active.update({'name': None, 'process': None, 'since': None})


def start_child(name):
    if name not in MODULES:
        raise HTTPException(404, 'Unknown GCS section')
    with lock:
        if active['name'] == name and active['process'] and active['process'].poll() is None:
            return {'name': name, 'path': MODULES[name][1]}
        stop_child()
        module, path = MODULES[name]
        cmd = [sys.executable, '-m', 'uvicorn', module, '--host', '0.0.0.0', '--port', str(CHILD_PORT), '--log-level', 'warning']
        try:
            p = subprocess.Popen(cmd, cwd=str(BASE), stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        except Exception as e:
            active['error'] = str(e)
            raise HTTPException(500, f'Could not start {name}: {e}')
        active.update({'name': name, 'process': p, 'since': time.time(), 'error': None})
    deadline = time.time() + 12
    while time.time() < deadline:
        if p.poll() is not None:
            with lock:
                active['error'] = f'{name} service exited with code {p.returncode}'
            raise HTTPException(500, active['error'])
        try:
            urllib.request.urlopen(f'http://127.0.0.1:{CHILD_PORT}/', timeout=0.4)
            break
        except Exception:
            time.sleep(0.25)
    return {'name': name, 'path': path}


@asynccontextmanager
async def lifespan(app):
    start_child('dashboard')
    yield
    vision_service.stop()
    with lock:
        stop_child()

app = FastAPI(title='Roberon Unified Drone GCS', lifespan=lifespan)
app.include_router(vision_router)

@app.get('/', response_class=HTMLResponse)
def home():
    return HTML

@app.post('/api/open/{name}')
def open_section(name: str):
    result = start_child(name)
    return {'success': True, **result, 'url': f'http://{request_host() }:{CHILD_PORT}{result["path"]}'}

def request_host():
    return os.environ.get('GCS_HOST', '192.168.0.2')

@app.get('/api/active')
def active_status():
    with lock:
        name = active['name']; p = active['process']; err = active['error']
        running = bool(p and p.poll() is None)
    data = {'active': name, 'running': running, 'error': err}
    if not running or not name:
        return data
    path = {'dashboard':'/api/status','radio':'/api/radio','parameters':'/api/radio','calibration':'/api/status','firmware':'/api/status','frame':'/api/status'}[name]
    try:
        with urllib.request.urlopen(f'http://127.0.0.1:{CHILD_PORT}{path}', timeout=1.5) as r:
            child = json.loads(r.read().decode())
        if name in ('radio','parameters'):
            data.update({'connected': child.get('connected', False), 'armed': child.get('armed', False), 'mode':'--', 'firmware':'--', 'battery':None, 'gps_fix':0, 'satellites':0})
        elif name in ('calibration','frame'):
            data.update({'connected': child.get('connected', False), 'armed': child.get('armed', False), 'mode':'--', 'firmware':'--'})
        elif name == 'firmware':
            data.update({'connected': False, 'armed': False, 'mode':'BOOTLOADER / FLASH', 'firmware': child.get('status','--'), 'battery':None, 'gps_fix':0, 'satellites':0})
        else:
            data.update(child)
    except Exception as e:
        data['status_error'] = str(e)
    return data

@app.post('/api/stop')
def stop_active():
    with lock: stop_child()
    return {'success': True}

HTML = r'''<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Roberon Drone GCS</title>
<style>
:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#080e19;color:#e8eef8;font:14px system-ui,Arial}.dock{position:sticky;top:0;z-index:10;background:#111d2d;border-bottom:1px solid #2b3c54;padding:10px 16px;box-shadow:0 4px 16px #0006}.brand{font-weight:800;letter-spacing:.08em;color:#7db5ff;margin-right:18px}.stats{display:flex;gap:9px;flex-wrap:wrap;margin-top:8px}.pill{background:#1c2b40;border:1px solid #344760;border-radius:8px;padding:7px 10px;color:#c6d4e6}.pill b{color:#fff;margin-left:4px}.ok{color:#48dfa5!important}.bad{color:#ff7788!important}.nav{display:flex;gap:7px;flex-wrap:wrap;padding:12px 16px;background:#0d1726;border-bottom:1px solid #26374e}.nav button{background:#1a2a40;border:1px solid #344a66;color:#dce8f7;padding:9px 12px;border-radius:8px;cursor:pointer}.nav button.active{background:#2862a8;border-color:#4b91e7;color:white}iframe{display:block;width:100%;height:calc(100vh - 148px);min-height:600px;border:0;background:#0b1220}.hint{padding:10px 16px;color:#91a4bc;font-size:12px}@media(max-width:600px){iframe{height:calc(100vh - 220px);min-height:650px}.dock{padding:9px}.pill{padding:6px 8px}}
</style></head><body>
<header class="dock"><span class="brand">ROBERON | DRONE GCS</span><span id="conn" class="bad">CONNECTING</span><div class="stats">
<span class="pill">Mode <b id="mode">--</b></span><span class="pill">Armed <b id="armed">--</b></span><span class="pill">Firmware <b id="fw">--</b></span><span class="pill">Battery <b id="bat">--</b></span><span class="pill">GPS <b id="gps">--</b></span><span class="pill">Section <b id="section">Dashboard</b></span>
</div></header>
<nav class="nav">
<button data-name="dashboard" class="active">Dashboard</button><button data-name="vision">Human & IR Landing</button><button data-name="calibration">Accelerometer / Compass</button><button data-name="radio">Radio Calibration</button><button data-name="parameters">Parameters & Safety</button><button data-name="firmware">Firmware</button><button data-name="frame">Frame Setup</button>
</nav><div class="hint" id="hint">Only one Pixhawk-connected service is active at a time. Switch sections using the navigation above.</div>
<iframe id="view" title="Drone GCS section"></iframe>
<script>
const frame=document.getElementById('view');let current='dashboard';
frame.src=location.protocol+'//'+location.hostname+':8001/';
document.querySelector('[data-name="vision"]').addEventListener('click',()=>{frame.src='/vision';document.getElementById('section').textContent='Human & IR Landing';document.getElementById('hint').textContent='Vision diagnostics run alongside the active telemetry module. Flight commands disabled.';document.querySelectorAll('.nav button').forEach(b=>b.classList.toggle('active',b.dataset.name==='vision'));});
async function openSection(name){document.querySelectorAll('.nav button').forEach(b=>b.classList.toggle('active',b.dataset.name===name));document.getElementById('hint').textContent='Switching to '+name+'â€¦';try{let r=await fetch('/api/open/'+name,{method:'POST'});let d=await r.json();if(!r.ok)throw Error(d.detail||'Could not open section');current=name;document.getElementById('section').textContent=name;frame.src=location.protocol+'//'+location.hostname+':8001'+d.path;document.getElementById('hint').textContent='Active module: '+name+'. The previous module has been stopped to prevent competing MAVLink readers.';}catch(e){document.getElementById('hint').textContent=e.message;}}
document.querySelectorAll('.nav button:not([data-name="vision"])').forEach(b=>b.addEventListener('click',()=>openSection(b.dataset.name)));
async function refresh(){try{let d=await(await fetch('/api/active',{cache:'no-store'})).json();let connected=!!d.connected;let c=document.getElementById('conn');c.textContent=connected?'PIXHAWK CONNECTED':(current==='firmware'?'FIRMWARE MODE':'PIXHAWK DISCONNECTED');c.className=connected?'ok':'bad';document.getElementById('mode').textContent=d.mode||'--';document.getElementById('armed').textContent=d.armed?'ARMED':'DISARMED';document.getElementById('armed').className=d.armed?'bad':'ok';document.getElementById('fw').textContent=d.firmware||'--';document.getElementById('bat').textContent=d.battery==null?'N/A':d.battery+' V';document.getElementById('gps').textContent=d.gps_fix==null?'N/A':('Fix '+d.gps_fix+' / '+(d.satellites??0)+' sats');}catch(e){document.getElementById('conn').textContent='STATUS UNAVAILABLE'}}
setInterval(refresh,1000);refresh();
</script></body></html>'''
