import time
import threading
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pymavlink import mavutil

PORT = '/dev/serial/by-id/usb-Holybro_Pixhawk6C_2E0042001651333337363133-if00'
BAUD = 115200

master = None
connected = False
armed = False
lock = threading.RLock()

accel = {
    "running": False,
    "position": 0,
    "status": "READY",
    "error": None,
    "completed": 0
}

compass = {
    "running": False,
    "status": "READY",
    "error": None,
    "progress": {},
    "reports": {}
}

messages = []
message_lock = threading.Lock()

positions = {
    1: ("LEVEL", "Place the drone flat on its landing gear.", "level"),
    2: ("LEFT", "Rotate the drone onto its LEFT side.", "left"),
    3: ("RIGHT", "Rotate the drone onto its RIGHT side.", "right"),
    4: ("NOSE DOWN", "Point the nose directly towards the ground.", "nose-down"),
    5: ("NOSE UP", "Point the nose towards the ceiling.", "nose-up"),
    6: ("BACK", "Turn the drone completely upside down.", "back")
}

def send_command(command, *params):
    if master is None or not connected:
        raise RuntimeError("Pixhawk is disconnected")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,
        *params
    )

def worker():
    global master, connected, armed

    while True:
        try:
            print("Connecting to Pixhawk...")
            master = mavutil.mavlink_connection(
                PORT, baud=BAUD, source_system=255
            )
            master.wait_heartbeat(timeout=15)
            master.target_component = 1
            connected = True
            print("Pixhawk connected")

            # Request EXTRA3 telemetry for compass calibration messages.
            try:
                master.mav.request_data_stream_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
                    10,
                    1
                )
            except Exception as e:
                print("Stream request:", e)

            while True:
                msg = master.recv_match(blocking=True, timeout=1)
                if msg is None:
                    continue

                kind = msg.get_type()

                with lock:
                    if kind == "HEARTBEAT" and msg.get_srcComponent() == 1:
                        armed = bool(msg.base_mode & 128)

                    elif kind == "COMMAND_LONG":
                        print(
                            "[MAVLINK RAW] COMMAND_LONG",
                            "source_system=", msg.get_srcSystem(),
                            "source_component=", msg.get_srcComponent(),
                            "message=", repr(msg.to_dict()),
                            flush=True
                        )

                        if msg.command == mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS:
                            print(
                                "[ACCELCAL EVENT] position_value=",
                                msg.param1,
                                flush=True
                            )
                            value = int(msg.param1)

                            if accel["running"] and value in positions:
                                accel.update({
                                    "position": value,
                                    "status": "POSITION_REQUIRED"
                                })
                            elif accel["running"] and value == 16777215:
                                accel.update({
                                    "running": False,
                                    "position": 0,
                                    "status": "SUCCESS",
                                    "completed": 6
                                })
                            elif accel["running"] and value == 16777216:
                                accel.update({
                                    "running": False,
                                    "position": 0,
                                    "status": "FAILED",
                                    "error": "Pixhawk reported accelerometer calibration failure."
                                })

                    elif kind == "COMMAND_ACK":
                        print(
                            "[MAVLINK RAW] COMMAND_ACK",
                            "source_system=", msg.get_srcSystem(),
                            "source_component=", msg.get_srcComponent(),
                            "message=", repr(msg.to_dict()),
                            flush=True
                        )

                        if msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                            if msg.result not in (0, 5):
                                accel.update({
                                    "running": False,
                                    "status": "FAILED",
                                    "error": f"Accelerometer command rejected. MAV_RESULT={msg.result}"
                                })

                        elif msg.command == mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL:
                            if msg.result not in (0, 5):
                                compass.update({
                                    "running": False,
                                    "status": "FAILED",
                                    "error": f"Compass calibration rejected. MAV_RESULT={msg.result}"
                                })

                    elif kind == "STATUSTEXT":
                        raw_text = msg.text
                        if isinstance(raw_text, bytes):
                            text_value = raw_text.decode("utf-8", errors="replace")
                        else:
                            text_value = str(raw_text)

                        text_value = text_value.rstrip("\\x00").strip()

                        if text_value:
                            severity = int(msg.severity)
                            severity_names = {
                                0: "EMERGENCY",
                                1: "ALERT",
                                2: "CRITICAL",
                                3: "ERROR",
                                4: "WARNING",
                                5: "NOTICE",
                                6: "INFO",
                                7: "DEBUG"
                            }

                            entry = {
                                "time": time.strftime("%H:%M:%S"),
                                "severity": severity,
                                "level": severity_names.get(severity, "INFO"),
                                "text": text_value,
                                "source_system": msg.get_srcSystem(),
                                "source_component": msg.get_srcComponent()
                            }

                            with message_lock:
                                # Avoid duplicate messages arriving repeatedly.
                                if not messages or (
                                    messages[-1]["text"] != text_value
                                    or messages[-1]["severity"] != severity
                                ):
                                    messages.append(entry)
                                    del messages[:-150]

                            print(
                                "[PIXHAWK STATUSTEXT]",
                                entry,
                                flush=True
                            )

                    elif kind == "MAG_CAL_PROGRESS":
                        cid = str(msg.compass_id)
                        compass["progress"][cid] = {
                            "percent": int(msg.completion_pct),
                            "status": "CALIBRATING",
                            "direction": int(msg.cal_mask)
                        }

                    elif kind == "MAG_CAL_REPORT":
                        cid = str(msg.compass_id)
                        success_code = getattr(
                            mavutil.mavlink, "MAG_CAL_SUCCESS", 4
                        )
                        success = msg.cal_status == success_code

                        compass["reports"][cid] = {
                            "success": success,
                            "fitness": float(msg.fitness),
                            "autosaved": bool(msg.autosaved)
                        }

                        compass["progress"][cid] = {
                            "percent": 100 if success else
                                compass["progress"].get(cid, {}).get("percent", 0),
                            "status": "SUCCESS" if success else "FAILED",
                            "direction": 0
                        }

                        if not success:
                            compass["error"] = (
                                f"Compass {cid} calibration failed. "
                                f"Fitness: {msg.fitness:.2f}"
                            )
                            compass["status"] = "FAILED"
                            compass["running"] = False

                        elif compass["progress"] and all(
                            p["status"] == "SUCCESS"
                            for p in compass["progress"].values()
                        ):
                            compass["status"] = "SUCCESS"
                            compass["running"] = False

        except Exception as e:
            print("MAVLink connection error:", e)

        finally:
            connected = False
            master = None

            with lock:
                if accel["running"]:
                    accel.update({
                        "running": False,
                        "status": "DISCONNECTED",
                        "error": "Pixhawk connection lost."
                    })

                if compass["running"]:
                    compass.update({
                        "running": False,
                        "status": "DISCONNECTED",
                        "error": "Pixhawk connection lost."
                    })

        time.sleep(3)


@asynccontextmanager
async def lifespan(app):
    threading.Thread(target=worker, daemon=True).start()
    yield


app = FastAPI(title="Pixhawk Calibration Station", lifespan=lifespan)


@app.get("/", response_class=HTMLResponse)
def home():
    return HTML


@app.get("/api/status")
def status():
    with lock:
        return {
            "connected": connected,
            "armed": armed,
            "accelerometer": dict(accel),
            "compass": {
                **compass,
                "progress": dict(compass["progress"]),
                "reports": dict(compass["reports"])
            },
            "messages": list(messages)
        }


@app.post("/api/prearm/check")
def request_prearm_check():
    with lock:
        if not connected:
            raise HTTPException(503, "Pixhawk disconnected")

        try:
            # MAV_CMD_RUN_PREARM_CHECK = 401.
            # Request checks without attempting to arm.
            send_command(401, 0, 0, 0, 0, 0, 0, 0)
        except Exception as e:
            raise HTTPException(500, str(e))

    return {"success": True, "message": "Pre-arm checks requested."}


@app.post("/api/messages/clear")
def clear_messages():
    with message_lock:
        messages.clear()
    return {"success": True}


@app.post("/api/accel/start")
def accel_start():
    with lock:
        if not connected:
            raise HTTPException(503, "Pixhawk disconnected")
        if armed:
            raise HTTPException(403, "Disarm the drone first.")
        if accel["running"] or compass["running"]:
            raise HTTPException(409, "Another calibration is running.")

        accel.update({
            "running": True,
            "position": 0,
            "status": "STARTING",
            "error": None,
            "completed": 0
        })

        try:
            send_command(
                mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                0, 0, 0, 0, 1, 0, 0
            )
        except Exception as e:
            accel.update({"running": False, "status": "FAILED", "error": str(e)})
            raise HTTPException(500, str(e))

    return {"success": True}


@app.post("/api/accel/confirm")
def accel_confirm():
    with lock:
        if not accel["running"] or accel["position"] not in positions:
            raise HTTPException(400, "No position is awaiting confirmation.")

        position = accel["position"]

        try:
            send_command(
                mavutil.mavlink.MAV_CMD_ACCELCAL_VEHICLE_POS,
                position, 0, 0, 0, 0, 0, 0
            )
        except Exception as e:
            raise HTTPException(500, str(e))

        accel.update({
            "position": 0,
            "status": "CAPTURING",
            "completed": min(6, accel["completed"] + 1)
        })

    return {"success": True}


@app.post("/api/accel/cancel")
def accel_cancel():
    with lock:
        accel.update({
            "running": False,
            "position": 0,
            "status": "CANCELLED"
        })
    return {"success": True}


@app.post("/api/compass/start")
def compass_start():
    with lock:
        if not connected:
            raise HTTPException(503, "Pixhawk disconnected")
        if armed:
            raise HTTPException(403, "Disarm the drone first.")
        if accel["running"] or compass["running"]:
            raise HTTPException(409, "Another calibration is running.")

        compass.update({
            "running": True,
            "status": "STARTING",
            "error": None,
            "progress": {},
            "reports": {}
        })

        try:
            # All enabled compasses, retry disabled, autosave enabled.
            send_command(
                mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,
                0, 0, 1, 0, 0, 0, 0
            )
        except Exception as e:
            compass.update({
                "running": False,
                "status": "FAILED",
                "error": str(e)
            })
            raise HTTPException(500, str(e))

    return {"success": True}


@app.post("/api/compass/cancel")
def compass_cancel():
    with lock:
        if not compass["running"]:
            raise HTTPException(400, "No compass calibration is running.")

        try:
            send_command(
                mavutil.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL,
                0, 0, 0, 0, 0, 0, 0
            )
        except Exception as e:
            raise HTTPException(500, str(e))

        compass.update({
            "running": False,
            "status": "CANCELLED"
        })

    return {"success": True}


HTML = r"""
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pixhawk Calibration Station</title>
<style>
:root {
 --bg:#0b1020; --panel:#131b2e; --panel2:#19243a;
 --text:#edf3ff; --muted:#91a0b8; --blue:#548cff;
 --green:#31d49a; --red:#ff6577; --border:#283650;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);
 font-family:Inter,system-ui,-apple-system,sans-serif}
header{padding:22px 5%;border-bottom:1px solid var(--border);
 display:flex;justify-content:space-between;align-items:center;gap:15px}
.brand{font-size:18px;font-weight:750;letter-spacing:.2px}
.brand span{color:var(--blue)}
.connection{font-size:13px;color:var(--muted)}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;
 background:var(--red);margin-right:7px}
.dot.on{background:var(--green);box-shadow:0 0 12px var(--green)}
main{max-width:1050px;margin:35px auto;padding:0 20px}
h1{font-size:30px;margin:0 0 9px}
.subtitle{color:var(--muted);margin-bottom:28px;line-height:1.6}
.tabs{display:flex;gap:10px;margin-bottom:20px}
.tab{background:var(--panel);border:1px solid var(--border);
 color:var(--muted);padding:12px 20px;border-radius:10px;
 cursor:pointer;font-weight:650}
.tab.active{background:#20375f;color:white;border-color:var(--blue)}
.card{background:var(--panel);border:1px solid var(--border);
 border-radius:17px;padding:25px;margin-bottom:18px}
.cardhead{display:flex;justify-content:space-between;gap:15px;
 align-items:flex-start;margin-bottom:22px}
h2{font-size:20px;margin:0 0 7px}
.desc{font-size:13px;color:var(--muted);line-height:1.5}
.badge{font-size:11px;padding:7px 10px;border-radius:20px;
 background:var(--panel2);color:var(--muted);white-space:nowrap}
.status{padding:13px 15px;border-radius:10px;background:var(--panel2);
 color:var(--muted);font-size:13px;margin:16px 0}
.status.good{color:var(--green)}
.status.bad{color:var(--red)}
button{border:0;border-radius:10px;padding:13px 19px;
 font-weight:700;cursor:pointer;font-size:14px}
button:disabled{opacity:.4;cursor:not-allowed}
.primary{background:var(--blue);color:white}
.secondary{background:var(--panel2);color:white;border:1px solid var(--border)}
.danger{background:#482332;color:#ff9ba7}
.actions{display:flex;gap:10px;flex-wrap:wrap;margin-top:18px}
.hidden{display:none!important}
.progress-track{height:8px;background:#27334b;border-radius:10px;overflow:hidden}
.progress-fill{height:100%;width:0;background:var(--blue);transition:width .3s}
.steps{display:grid;grid-template-columns:repeat(6,1fr);gap:8px;margin:22px 0}
.step{padding:12px 5px;text-align:center;background:var(--panel2);
 border:1px solid var(--border);border-radius:10px;font-size:11px;color:var(--muted)}
.step.active{border-color:var(--blue);color:white;background:#20375f}
.step.done{border-color:var(--green);color:var(--green)}
.visual{height:210px;display:flex;align-items:center;justify-content:center;
 background:var(--panel2);border:1px solid var(--border);border-radius:14px;
 margin:18px 0}
.drone{width:155px;height:155px;transition:transform .5s}
.drone .body{fill:#9eb7db;stroke:#dce9ff;stroke-width:3}
.drone .arm{stroke:#9eb7db;stroke-width:9;stroke-linecap:round}
.drone .motor{fill:#548cff;stroke:#dce9ff;stroke-width:2}
.instruction{font-size:19px;font-weight:700;margin:10px 0}
.small{font-size:13px;color:var(--muted);line-height:1.6}
.compass-list{display:grid;gap:12px;margin-top:18px}
.compass-row{padding:15px;background:var(--panel2);border-radius:11px}
.rowtop{display:flex;justify-content:space-between;margin-bottom:10px;font-size:13px}
footer{text-align:center;color:#62718a;font-size:12px;padding:30px}
@media(max-width:600px){
 header{padding:17px}.brand{font-size:15px}
 main{margin:24px auto;padding:0 13px}
 h1{font-size:25px}.card{padding:18px}
 .steps{grid-template-columns:repeat(3,1fr)}
 .tab{flex:1;padding:12px 7px;font-size:12px}
}

.message-panel {
 background:var(--panel);border:1px solid var(--border);
 border-radius:17px;padding:22px;margin-top:20px
}
.message-list {
 max-height:360px;overflow-y:auto;margin-top:15px;
 display:flex;flex-direction:column;gap:8px
}
.message-item {
 padding:12px;border-radius:9px;background:var(--panel2);
 border-left:4px solid var(--blue);font-size:13px
}
.message-item.ERROR,.message-item.CRITICAL,.message-item.EMERGENCY,.message-item.ALERT {
 border-left-color:var(--red)
}
.message-item.WARNING {border-left-color:#ffbd69}
.message-item.INFO,.message-item.NOTICE {border-left-color:var(--green)}
.message-meta {
 display:flex;justify-content:space-between;gap:10px;
 margin-bottom:5px;font-size:11px;color:var(--muted)
}
.message-level {font-weight:800}
.message-empty {color:var(--muted);font-size:13px;padding:15px}
.message-actions {display:flex;gap:10px;flex-wrap:wrap;margin-top:15px}
.prearm-note {font-size:12px;color:var(--muted);line-height:1.5;margin-top:12px}
</style>
</head>
<body>
<header>
 <div class="brand">ROBERON <span>| CALIBRATION STATION</span></div>
 <div class="connection"><span class="dot" id="dot"></span><span id="connection">Connecting...</span></div>
</header>
<main>
 <h1>Sensor Calibration</h1>
 <div class="subtitle">Configure and calibrate your flight controller. Keep the aircraft disarmed and stationary when instructed.</div>

 <div class="tabs">
  <button class="tab active" onclick="showTab('accel',this)">Accelerometer</button>
  <button class="tab" onclick="showTab('compass',this)">Compass</button>
 </div>

 <section id="accel">
  <div class="card">
   <div class="cardhead">
    <div><h2>Accelerometer calibration</h2>
    <div class="desc">Six-position calibration of the flight controller's accelerometers.</div></div>
    <span class="badge" id="accelBadge">READY</span>
   </div>

   <div class="small">Before starting: remove propellers, disconnect the battery if appropriate for your setup, and place the drone on a stable surface. Keep it completely still during each capture.</div>

   <div class="steps" id="steps">
    <div class="step">1 · LEVEL</div><div class="step">2 · LEFT</div>
    <div class="step">3 · RIGHT</div><div class="step">4 · NOSE DOWN</div>
    <div class="step">5 · NOSE UP</div><div class="step">6 · BACK</div>
   </div>

   <div class="visual">
    <svg class="drone" id="drone" viewBox="0 0 200 200">
     <g class="arm">
      <line x1="100" y1="100" x2="35" y2="35"/>
      <line x1="100" y1="100" x2="165" y2="35"/>
      <line x1="100" y1="100" x2="35" y2="165"/>
      <line x1="100" y1="100" x2="165" y2="165"/>
     </g>
     <g>
      <circle class="motor" cx="35" cy="35" r="16"/>
      <circle class="motor" cx="165" cy="35" r="16"/>
      <circle class="motor" cx="35" cy="165" r="16"/>
      <circle class="motor" cx="165" cy="165" r="16"/>
     </g>
     <path class="body" d="M100 70 L126 100 L100 130 L74 100 Z"/>
     <path fill="#548cff" d="M100 70 L92 87 L108 87 Z"/>
    </svg>
   </div>

   <div class="instruction" id="accelInstruction">Ready to begin?</div>
   <div class="small" id="accelHelp">Start calibration when the drone is stationary and disarmed.</div>
   <div class="status" id="accelStatus">Waiting to start.</div>

   <div class="progress-track"><div class="progress-fill" id="accelProgress"></div></div>
   <div class="actions">
    <button class="primary" id="accelStart" onclick="post('/api/accel/start')">Start calibration</button>
    <button class="primary hidden" id="accelConfirm" onclick="post('/api/accel/confirm')">Confirm position</button>
    <button class="danger hidden" id="accelCancel" onclick="post('/api/accel/cancel')">Cancel</button>
   </div>
  </div>
 </section>

 <section id="compass" class="hidden">
  <div class="card">
   <div class="cardhead">
    <div><h2>Compass calibration</h2>
    <div class="desc">Calibrate all enabled magnetometers detected by ArduPilot.</div></div>
    <span class="badge" id="compassBadge">READY</span>
   </div>

   <div class="status">Important: move away from vehicles, steel structures, magnets, speakers and power cables. Perform calibration outdoors where possible. Disconnect propellers and keep the drone disarmed.</div>

   <div class="visual">
    <svg width="180" height="180" viewBox="0 0 180 180">
     <circle cx="90" cy="90" r="67" fill="none" stroke="#344763" stroke-width="2" stroke-dasharray="5 6"/>
     <circle cx="90" cy="90" r="45" fill="#20375f" stroke="#548cff" stroke-width="2"/>
     <path d="M90 26 L80 90 L90 81 L100 90 Z" fill="#548cff"/>
     <path d="M90 154 L80 90 L90 99 L100 90 Z" fill="#91a0b8"/>
     <text x="90" y="15" text-anchor="middle" fill="#edf3ff" font-size="11">NOSE</text>
     <text x="90" y="174" text-anchor="middle" fill="#edf3ff" font-size="11">TAIL</text>
     <text x="90" y="94" text-anchor="middle" fill="white" font-size="10">ROTATE</text>
    </svg>
   </div>

   <div class="instruction">Rotate the drone slowly through all orientations</div>
   <div class="small">Follow the live calibration progress. Slowly rotate the aircraft around every axis: nose, tail, left side, right side, top and bottom. Avoid sudden movements.</div>

   <div class="compass-list" id="compassList">
    <div class="small">No compass progress received yet.</div>
   </div>
   <div class="status" id="compassStatus">Ready to calibrate.</div>
   <div class="actions">
    <button class="primary" id="compassStart" onclick="post('/api/compass/start')">Start compass calibration</button>
    <button class="danger hidden" id="compassCancel" onclick="post('/api/compass/cancel')">Cancel calibration</button>
   </div>
  </div>
 </section>
</main>
<section class="message-panel">
 <div class="cardhead">
  <div>
   <h2>Flight Controller Messages</h2>
   <div class="desc">Live messages received directly from Pixhawk.</div>
  </div>
  <span class="badge" id="messageCount">0 messages</span>
 </div>

 <div class="message-actions">
  <button class="primary" id="prearmButton" onclick="runPrearm()">Run Pre-Arm Checks</button>
  <button class="secondary" onclick="clearMessages()">Clear Messages</button>
 </div>

 <div class="prearm-note">
  Requests the flight controller's pre-arm checks without attempting to arm.
  Keep the aircraft disarmed and propellers removed.
 </div>

 <div class="message-list" id="messageList">
  <div class="message-empty">Waiting for messages from Pixhawk...</div>
 </div>
</section>

<footer>Roberon Technologies · Pixhawk Calibration Station</footer>

<script>
let currentTab='accel';
function showTab(name,btn){
 currentTab=name;
 document.getElementById('accel').classList.toggle('hidden',name!=='accel');
 document.getElementById('compass').classList.toggle('hidden',name!=='compass');
 document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active'));
 btn.classList.add('active');
}
async function post(url){
 try{
  const r=await fetch(url,{method:'POST'});
  const d=await r.json();
  if(!r.ok) throw new Error(d.detail||'Request failed');
  await refresh();
 }catch(e){alert(e.message)}
}
function setStatus(id,msg,type=''){
 const el=document.getElementById(id);
 el.textContent=msg;
 el.className='status '+type;
}
function renderAccel(a){
 const active=a.position||0;
 document.getElementById('accelBadge').textContent=a.status;
 document.getElementById('accelStart').classList.toggle('hidden',a.running);
 document.getElementById('accelConfirm').classList.toggle('hidden',!active);
 document.getElementById('accelCancel').classList.toggle('hidden',!a.running);
 document.getElementById('accelProgress').style.width=(a.completed/6*100)+'%';

 document.querySelectorAll('.step').forEach((el,i)=>{
  el.classList.toggle('active',i+1===active);
  el.classList.toggle('done',i+1<=a.completed);
 });

 if(active){
  const p={
   1:[0,'LEVEL','Place the drone flat on its landing gear.'],
   2:[90,'LEFT','Rotate the drone onto its LEFT side.'],
   3:[-90,'RIGHT','Rotate the drone onto its RIGHT side.'],
   4:[90,'NOSE DOWN','Point the nose towards the ground.'],
   5:[-90,'NOSE UP','Point the nose towards the ceiling.'],
   6:[180,'BACK','Turn the drone completely upside down.']
  }[active];
  document.getElementById('drone').style.transform='rotate('+p[0]+'deg)';
  document.getElementById('accelInstruction').textContent=p[1];
  document.getElementById('accelHelp').textContent=p[2]+' Keep it still, then confirm.';
  setStatus('accelStatus','Position '+active+' of 6: waiting for your confirmation.');
 }else if(a.status==='SUCCESS'){
  setStatus('accelStatus','✓ Accelerometer calibration completed successfully.','good');
  document.getElementById('accelInstruction').textContent='Calibration complete';
  document.getElementById('accelHelp').textContent='Pixhawk reported successful calibration.';
  document.getElementById('accelProgress').style.width='100%';
 }else if(a.status==='FAILED'||a.status==='DISCONNECTED'){
  setStatus('accelStatus',a.error||a.status,'bad');
 }else if(a.running){
  setStatus('accelStatus','Pixhawk is processing calibration. Keep the drone still.');
 }else{
  setStatus('accelStatus',a.status==='CANCELLED'?'Calibration cancelled.':'Ready to begin.');
 }
}
function renderCompass(c){
 document.getElementById('compassBadge').textContent=c.status;
 document.getElementById('compassStart').classList.toggle('hidden',c.running||c.status==='SUCCESS');
 document.getElementById('compassCancel').classList.toggle('hidden',!c.running);

 const list=document.getElementById('compassList');
 const entries=Object.entries(c.progress||{});
 if(entries.length){
  list.innerHTML=entries.map(([id,p])=>`
   <div class="compass-row">
    <div class="rowtop"><strong>Compass ${id}</strong><span>${p.status==='SUCCESS'?'Completed':p.percent+'%'}</span></div>
    <div class="progress-track"><div class="progress-fill" style="width:${p.percent}%;background:${p.status==='SUCCESS'?'var(--green)':'var(--blue)'}"></div></div>
   </div>`).join('');
 }else{
  list.innerHTML='<div class="small">Waiting for compass telemetry from Pixhawk.</div>';
 }
 if(c.status==='SUCCESS'){
  setStatus('compassStatus','✓ Compass calibration completed. Check that each report was autosaved.','good');
 }else if(c.status==='FAILED'||c.status==='DISCONNECTED'){
  setStatus('compassStatus',c.error||c.status,'bad');
 }else if(c.running){
  setStatus('compassStatus','Calibration running. Rotate the drone slowly through all axes.');
 }else if(c.status==='CANCELLED'){
  setStatus('compassStatus','Calibration cancelled.');
 }else{
  setStatus('compassStatus','Ready to calibrate.');
 }
}
function renderMessages(items){
 const list=document.getElementById('messageList');
 const count=document.getElementById('messageCount');

 count.textContent=items.length+' messages';

 if(!items.length){
  list.innerHTML='<div class="message-empty">No messages received yet. Run Pre-Arm Checks to request messages from Pixhawk.</div>';
  return;
 }

 list.innerHTML=items.slice().reverse().map(m=>`
  <div class="message-item ${m.level}">
   <div class="message-meta">
    <span class="message-level">${m.level}</span>
    <span>${m.time}</span>
   </div>
   <div>${escapeMessage(m.text)}</div>
  </div>
 `).join('');
}

function escapeMessage(value){
 return String(value).replace(/[&<>"']/g,c=>({
  '&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'
 }[c]));
}

async function runPrearm(){
 const button=document.getElementById('prearmButton');
 button.disabled=true;
 button.textContent='Requesting checks...';
 try{
  const r=await fetch('/api/prearm/check',{method:'POST'});
  const d=await r.json();
  if(!r.ok)throw new Error(d.detail||'Request failed');
  button.textContent='Checks requested';
 }catch(e){
  alert(e.message);
  button.textContent='Run Pre-Arm Checks';
 }finally{
  setTimeout(()=>{
   button.disabled=false;
   button.textContent='Run Pre-Arm Checks';
  },2500);
 }
}

async function clearMessages(){
 if(!confirm('Clear the displayed message history?'))return;
 await fetch('/api/messages/clear',{method:'POST'});
 await refresh();
}

async function refresh(){
 try{
  const r=await fetch('/api/status',{cache:'no-store'});
  const d=await r.json();
  document.getElementById('dot').classList.toggle('on',d.connected);
  document.getElementById('connection').textContent=d.connected?'Pixhawk connected':'Pixhawk disconnected';
  renderAccel(d.accelerometer);
  renderCompass(d.compass);
  renderMessages(d.messages||[]);
  if(d.armed){
   document.getElementById('accelStart').disabled=true;
   document.getElementById('compassStart').disabled=true;
  }else{
   document.getElementById('accelStart').disabled=!d.connected;
   document.getElementById('compassStart').disabled=!d.connected;
  }
 }catch(e){
  document.getElementById('connection').textContent='Dashboard connection error';
 }
}
refresh();
setInterval(refresh,500);
</script>
</body>
</html>
"""
