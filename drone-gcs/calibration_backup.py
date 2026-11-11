from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pymavlink import mavutil
from threading import Thread, Lock
from queue import Queue, Empty
import time

app = FastAPI()
PORT = '/dev/serial/by-id/usb-Holybro_Pixhawk6C_2E0042001651333337363133-if00'

master = None
connected = False
armed = False
lock = Lock()
events = Queue()

cal = {
    "running": False,
    "position": 0,
    "status": "READY",
    "error": None
}

positions = {
    1: ("LEVEL", "Keep the drone flat, normally resting on its landing gear."),
    2: ("LEFT", "Turn the drone onto its LEFT side."),
    3: ("RIGHT", "Turn the drone onto its RIGHT side."),
    4: ("NOSE DOWN", "Point the front of the drone towards the ground."),
    5: ("NOSE UP", "Point the front of the drone towards the ceiling."),
    6: ("BACK", "Turn the drone upside down.")
}

def worker():
    global master, connected, armed

    while True:
        try:
            print("Connecting to Pixhawk...")
            master = mavutil.mavlink_connection(
                PORT, baud=115200, source_system=255
            )
            master.wait_heartbeat(timeout=15)
            master.target_component = 1
            connected = True
            print("Pixhawk connected!")

            while True:
                msg = master.recv_match(blocking=True, timeout=1)
                if msg is None:
                    continue

                kind = msg.get_type()

                if kind == "HEARTBEAT" and msg.get_srcComponent() == 1:
                    armed = bool(msg.base_mode & 128)

                elif kind == "COMMAND_LONG":
                    if msg.command == mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS:
                        events.put(int(msg.param1))

                elif kind == "COMMAND_ACK":
                    if msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                        events.put(("ACK", msg.result))

        except Exception as e:
            print("MAVLink:", e)

        finally:
            connected = False
            master = None
            with lock:
                cal["running"] = False
                cal["status"] = "DISCONNECTED"

        time.sleep(3)


Thread(target=worker, daemon=True).start()


@app.get("/api/status")
def status():
    return {
        "connected": connected,
        "armed": armed,
        "calibration": dict(cal)
    }


@app.post("/api/accel/start")
def start():
    if not connected or master is None:
        raise HTTPException(503, "Pixhawk disconnected")
    if armed:
        raise HTTPException(403, "Disarm drone first")

    if cal["running"]:
        raise HTTPException(409, "Calibration already running")

    while not events.empty():
        try:
            events.get_nowait()
        except Empty:
            break

    cal.update({
        "running": True,
        "position": 0,
        "status": "STARTING",
        "error": None
    })

    master.mav.command_long_send(
        master.target_system, 1,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0,
        0, 0, 0, 0, 1, 0, 0
    )

    return {"success": True, "message": "Calibration requested"}


@app.post("/api/accel/confirm")
def confirm():
    if not cal["running"] or cal["position"] not in positions:
        raise HTTPException(400, "No position is awaiting confirmation")

    position = cal["position"]

    master.mav.command_long_send(
        master.target_system, 1,
        mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS,
        0,
        position, 0, 0, 0, 0, 0, 0
    )

    cal["status"] = "CAPTURING"
    cal["position"] = 0

    return {"success": True}


@app.get("/api/accel/event")
def event():
    try:
        item = events.get_nowait()

        if isinstance(item, tuple):
            result = item[1]

            if result not in [0, 5]:
                cal.update({
                    "running": False,
                    "status": "FAILED",
                    "error": f"Calibration rejected. MAV_RESULT={result}"
                })
                return {"type": "FAILED", "message": cal["error"]}

            return {"type": "ACK", "result": result}

        if item == 16777215:
            cal.update({
                "running": False,
                "status": "SUCCESS",
                "position": 0
            })
            return {"type": "SUCCESS"}

        if item == 16777216:
            cal.update({
                "running": False,
                "status": "FAILED",
                "position": 0
            })
            return {"type": "FAILED"}

        if item in positions:
            cal.update({
                "position": item,
                "status": "POSITION_REQUIRED"
            })
            return {
                "type": "POSITION",
                "position": item,
                "name": positions[item][0],
                "instruction": positions[item][1]
            }

        return {"type": "UNKNOWN"}

    except Empty:
        return {"type": "WAITING"}


@app.get("/", response_class=HTMLResponse)
def home():
    return """
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Accelerometer Calibration</title>
<style>
*{box-sizing:border-box}
body{margin:0;background:#f3f6fb;color:#172033;font-family:Arial}
header{background:#101827;color:white;padding:22px}
header h2{margin:0;color:#38bdf8}
main{max-width:650px;margin:30px auto;padding:15px}
.card{background:white;padding:25px;border-radius:18px;text-align:center;box-shadow:0 5px 20px #0001}
.badge{background:#dbeafe;color:#1d4ed8;padding:9px 15px;border-radius:30px;font-size:12px}
h1{font-size:26px}
p{color:#64748b;line-height:1.6}
.progress{display:flex;gap:6px;margin:25px 0}
.progress span{height:7px;flex:1;background:#dbe3ee;border-radius:10px}
.progress .done{background:#16a34a}
.progress .active{background:#0284c7}
.stage{font-size:20px;font-weight:bold;color:#0284c7}
button{background:#0284c7;color:white;border:0;padding:16px 25px;border-radius:10px;font-size:16px;font-weight:bold;margin:10px;cursor:pointer}
button:disabled{opacity:.4}
svg{width:230px;height:190px;margin:15px}
.status{background:#eff6ff;padding:15px;border-radius:10px;margin-top:20px}
.success{background:#dcfce7;color:#166534}
.error{background:#fee2e2;color:#991b1b}
</style>
</head>
<body>
<header>
<h2>DRONE SETUP</h2>
<p style="color:#bac7d9">Accelerometer calibration · Step 02</p>
</header>
<main>
<div class="card">
<span class="badge">SENSOR CALIBRATION</span>
<h1>Accelerometer Setup</h1>
<p>We will calibrate the drone in six different positions. Follow each instruction carefully.</p>

<div class="progress" id="progress">
<span></span><span></span><span></span><span></span><span></span><span></span>
</div>

<div class="stage" id="stage">READY TO BEGIN</div>

<svg viewBox="0 0 240 190">
<g id="drone" stroke="#26374e" stroke-width="13" stroke-linecap="round">
<path d="M120 95 L48 23"/>
<path d="M120 95 L192 23"/>
<path d="M120 95 L48 167"/>
<path d="M120 95 L192 167"/>
</g>
<g fill="#38bdf8">
<circle cx="48" cy="23" r="19"/>
<circle cx="192" cy="23" r="19"/>
<circle cx="48" cy="167" r="19"/>
<circle cx="192" cy="167" r="19"/>
</g>
<rect x="103" y="78" width="34" height="34" rx="8" fill="#0284c7"/>
</svg>

<h2 id="position">Place drone on LEVEL surface</h2>
<p id="instruction">Remove propellers and keep the drone stationary.</p>

<button id="start" onclick="startCal()">START CALIBRATION</button>
<button id="confirm" onclick="confirmPosition()" hidden>POSITION READY</button>

<div class="status" id="status">Pixhawk connection checking...</div>
</div>
</main>

<script>
let current=0;
let running=false;

const names={
1:"LEVEL",2:"LEFT",3:"RIGHT",
4:"NOSE DOWN",5:"NOSE UP",6:"BACK"
};

function updateProgress(n){
 document.querySelectorAll('.progress span').forEach((el,i)=>{
  el.className=i<n-1?'done':i===n-1?'active':'';
 });
}

async function startCal(){
 if(!confirm("Remove all propellers. Keep the drone disarmed and on a stable surface. Start calibration?"))return;

 document.getElementById('start').disabled=true;
 document.getElementById('status').textContent="Starting calibration...";
 try{
  let r=await fetch('/api/accel/start',{method:'POST'});
  let d=await r.json();
  if(!r.ok)throw Error(d.detail);
  running=true;
  poll();
 }catch(e){
  document.getElementById('status').textContent=e.message;
  document.getElementById('start').disabled=false;
 }
}

async function confirmPosition(){
 document.getElementById('confirm').hidden=true;
 document.getElementById('status').textContent="Capturing sensor readings. Keep still...";
 await fetch('/api/accel/confirm',{method:'POST'});
}

async function poll(){
 if(!running)return;
 try{
  let d=await(await fetch('/api/accel/event')).json();

  if(d.type==='POSITION'){
   current=d.position;
   document.getElementById('stage').textContent='POSITION '+current+' / 6';
   document.getElementById('position').textContent=d.name;
   document.getElementById('instruction').textContent=d.instruction;
   document.getElementById('confirm').hidden=false;
   document.getElementById('status').textContent='Place the drone correctly, keep it stationary, then confirm.';
   updateProgress(current);
  }

  if(d.type==='ACK'){
   document.getElementById('status').textContent='Flight controller accepted calibration request. Waiting for first position...';
  }

  if(d.type==='SUCCESS'){
   running=false;
   updateProgress(7);
   document.getElementById('stage').textContent='CALIBRATION SUCCESSFUL';
   document.getElementById('position').textContent='✓ Six positions completed';
   document.getElementById('instruction').textContent='Accelerometer calibration completed successfully.';
   document.getElementById('status').className='status success';
   document.getElementById('status').textContent='Pixhawk reports calibration successful.';
   return;
  }

  if(d.type==='FAILED'){
   running=false;
   document.getElementById('stage').textContent='CALIBRATION FAILED';
   document.getElementById('status').className='status error';
   document.getElementById('status').textContent=d.message||'Pixhawk reported calibration failure.';
   return;
  }
 }catch(e){}
 setTimeout(poll,200);
}

async function refresh(){
 try{
  let d=await(await fetch('/api/status')).json();
  if(!running){
   document.getElementById('status').textContent=
    d.connected?(d.armed?'Disarm the drone before calibration.':'Pixhawk connected · Drone disarmed'):'Pixhawk disconnected';
  }
  if(d.armed)document.getElementById('start').disabled=true;
  else if(!running)document.getElementById('start').disabled=!d.connected;
 }catch(e){}
}
setInterval(refresh,1500);
refresh();
</script>
</body>
</html>
"""

