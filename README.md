# Drone Swarm Simulator — Module 1 (GUI Application)

A PySide6 ground control station for simulating and monitoring a swarm of drones:
configure drone profiles, pick a route on a live map, fly the mission, and inject
faults to see how the swarm reacts.

The GUI is **Module 1** of a larger system. It talks to **Module 2** (the
Orchestration & Swarm Router) over a documented REST + WebSocket contract and has
no direct knowledge of MAVLink encoding. When Module 2 is not running, the GUI
falls back to a built-in local flight preview so the application is still fully
demonstrable on its own.

---

## Repository layout

```
MTP_Project/
├── simulator/simulator/         <- Module 1 GUI (current, active work)
│   ├── main.py                  entry point
│   ├── contracts/               shared Pydantic schemas for the Module 2 contract
│   ├── gui/                     all Qt widgets and panels
│   ├── qml/Map.qml              QtLocation map (OpenStreetMap tiles)
│   ├── services/                REST/WS client, JSON storage, local flight preview
│   └── data/                    saved profiles, presets, cached map tiles
└── Simulator_V1/                earlier gRPC + Leaflet prototype, kept for reference
```

`Simulator_V1/` is superseded. New work happens in `simulator/simulator/`.

---

## Requirements

- **Python 3.11**
- PySide6 ≥ 6.7, pydantic ≥ 2.0, requests ≥ 2.31
- An internet connection on first run, to fetch map tiles (they are then cached
  locally and the app works offline over the same area)

---

## Setup

```bash
cd simulator/simulator
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

## Running

```bash
.venv\Scripts\python.exe main.py
```

The working directory does not matter — paths are resolved relative to the source
files. In VS Code, select `.venv\Scripts\python.exe` as the interpreter
(`Ctrl+Shift+P` → *Python: Select Interpreter*) and press ▶.

---

## Using the application

### 1. Create a drone

**Drone Management → New.** A profile holds name, MAVLink SYSID, mass, max
velocity, cruise altitude, battery capacity and sensor set. Profiles are saved as
one JSON file each under `data/profiles/` and reload automatically on next launch.

Select a row before **Edit** or **Delete** — ticking the checkbox only marks a
drone for the active swarm, it does not select it for editing.

### 2. Plan a route

Set **Click mode** to *Set Start Point*, click the map, then switch to
*Set Destination Point* and click again. Tick the drones to include, then press
**Emulate**.

### 3. Watch the flight

The drone climbs to its cruise altitude, runs the course, and descends. The map
shows live position and the flown path; the Global State Matrix and Drone
Inspector show status, altitude, battery, heading, ground speed, artificial
horizon and a raw MAVLink log. The status bar reports elapsed and remaining
flight time.

Real missions take hours, so the animation is **time-compressed** to roughly 40
seconds while the *reported* durations stay physically true. Compression is
applied per phase, so the climb and descent remain visible instead of being
stepped over by a single tick.

---

## Mission behaviour

These rules are implemented in the local flight preview
(`services/local_flight.py`). When Module 2 is connected, it owns this behaviour
instead.

### Pre-flight battery check

Before anything takes off, each drone's full profile (climb + cruise + descent)
is costed in mAh against the battery it carries. Drones that cannot complete the
leg **stay grounded**, with a dialog naming each one and the shortfall; the rest
fly. If no drone can make it, nothing launches and the configuration inputs
unlock again.

### Re-routing in flight

Placing a new destination while a mission is running is a diversion order. Every
airborne drone turns for the new point from wherever it currently is.

### Low battery (20%)

The mission pauses and asks the controller for permission to break off:

- **Granted** — the drone diverts to the nearest landing point *it can still
  reach*, counting climb, travel and descent. If nothing is in range it lands
  where it stands rather than crashing en route to an unreachable point.
- **Denied** — it presses on until the battery is flat, then enters `FAILSAFE`
  with a `battery_fail` fault and is lost.

Every point placed on the map — the start and every destination, including
superseded ones — is a candidate landing site.

### GPS loss

Injecting a **GPS Loss** fault makes the drone hold station and report in:

- **Granted** — it abandons the mission and returns to its start point, subject
  to the same reachability rule as above.
- **Denied** — it hovers, truly stationary, while the battery keeps draining. If
  it reaches 20% while holding, it asks again. When GPS returns, the hold
  releases and the mission resumes from the hover position.

Set a **Duration** on the fault to have GPS restore itself automatically; leave
it *Indefinite* and it holds until cleared.

> Only GPS loss is modelled locally. Comms Loss and Battery Failure are forwarded
> to Module 2 and have no local effect while it is disconnected.

---

## Architecture

### Drones are objects, not threads

A drone exists in three forms and never gets a thread of its own:

| Form | Type | Lifetime |
|---|---|---|
| `DroneConfig` | Pydantic model | saved profile, one JSON file per drone |
| `_Flight` | dataclass | live mission state while flying |
| `DroneTelemetry` | Pydantic model | one immutable snapshot, 10× per second |

A single `QTimer` at 10 Hz steps every drone in one loop on the GUI thread —
cooperative time-stepping, so a swarm cannot race with itself and the whole
mission pauses atomically when a decision is needed. The only threading in the
project is a `QThreadPool` in `services/api_client.py`, one short-lived worker
per outgoing REST call, so an absent Module 2 never freezes the interface.

### Module 2 contract

Defined once in `contracts/gui_orchestration.py` and shared by both sides.

**REST commands (Module 1 → Module 2)**

| Method | Path | Body |
|---|---|---|
| `POST` | `/drones` | `RegisterDrone` |
| `GET` | `/drones` | — |
| `POST` | `/drones/{sysid}/takeoff` | `Takeoff` |
| `POST` | `/drones/{sysid}/goto` | `Goto` |
| `POST` | `/drones/{sysid}/inject_fault` | `InjectFault` |
| `POST` | `/swarm/flock` | `FlockCommand` |
| `POST` | `/swarm/stop` | `StopSwarm` |

**Telemetry (Module 2 → Module 1)** — `WS /telemetry` streams
`SwarmTelemetryBatch`, each carrying a list of `DroneTelemetry`.

Enter the endpoints in the toolbar and press **Connect**. On connection, live
telemetry supersedes the local preview automatically.

### Map

`qml/Map.qml` uses Qt's `osm` plugin pointed at standard OpenStreetMap tiles.
The plugin's online provider repository is disabled deliberately — by default it
resolves most map types to Thunderforest, which serves *"API Key Required"*
placeholder tiles without a paid key.

Tiles are cached under `data/tiles/` (git-ignored). Use **Download / Cache This
Region** to warm a bounding box for offline use.

---

## Known limitations

The local flight preview is a visualisation aid, not a flight model:

- No wind, acceleration envelope or turn radius; position interpolates linearly
  in lat/lon, which is indistinguishable from a great circle at the ranges one
  battery allows.
- Battery consumption assumes a flat 20 A draw (`NOMINAL_DRAW_A`), so `mass_kg`
  does not currently affect endurance.
- The "Offline mode" checkbox is inert: Qt plugin parameters are read once at
  construction, and QtLocation has no runtime online/offline switch. The map
  already falls back to the disk cache when tiles cannot be fetched.

---

## Troubleshooting

**Imports unresolved in VS Code** — the interpreter is pointing at an environment
without the dependencies. Select `.venv\Scripts\python.exe`.

**Map shows "API Key Required"** — a stale tile cache from before the OSM fix.
Delete `data/tiles/` and restart.

**Status bar reads "Disconnected"** — expected when Module 2 is not running. The
local flight preview covers it; everything except live telemetry still works.
