# 🌐 Geoscan Mission Planner: Web Platform User Guide
## Руководство пользователя интерактивного веб-интерфейса диспетчера
### Tactical Ground Control Station (GCS) Manual | Track 05: GC «Geoscan»

---

## 🚀 1. Quick Start & Launch

### Prerequisites
- Python 3.9+ (Windows, Linux, or macOS)
- Standard modern browser: Google Chrome, Microsoft Edge, Mozilla Firefox, or Safari.

### Starting the Server
Open a terminal in the project directory and run:
```bash
python serve.py
```
*(Or double-click `run_server.bat` on Windows).*

Once started, the terminal will display:
```
INFO:     Started server process [Uvicorn]
INFO:     Waiting for application startup.
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
```

### Accessing the System
- **Tactical GCS Web Interface:** [http://127.0.0.1:8000](http://127.0.0.1:8000)
- **Interactive OpenAPI (Swagger) Docs:** [http://127.0.0.1:8000/docs](http://127.0.0.1:8000/docs)
- **Direct Health Check:** [http://127.0.0.1:8000/api/health](http://127.0.0.1:8000/api/health)

---

## 🖥️ 2. Platform Architecture & Dashboard Layout

The web dashboard is organized into four tactical zones:

```
+-----------------------------------------------------------------------------------+
|  [🛰️ GEOSCAN FLEET DISPATCHER]   Status: ONLINE   Fleet: 3 UAVs   Weather: -15°C  |
+--------------------------+--------------------------------------------------------+
|                          |                                                        |
|  [1. FLEET & SENSORS]    |               [3. TACTICAL 2D/3D MAP]                  |
|  ☑ Geoscan 201 (Airplane)|                                                        |
|  ☑ Geoscan 801 (VTOL)    |   • Interactive Leaflet / High-Res Satellite           |
|  ☑ Gemini (Copter)       |   • Survey polygon boundaries (Cyan)                   |
|                          |   • No-Fly Zones (NFZ Red buffer)                      |
|  [2. ENVIRONMENT & GSD]  |   • Synthesized flight paths (Dubins curves)           |
|  • Wind: 6.0 m/s @ 240°  |   • Parachute drift cone & landing dispersion          |
|  • Temp: -15°C           |   • Base Station & Alternate landing site              |
|  • Target GSD: 5.0 cm/px |                                                        |
|  • Overlap: 75% / 70%    |                                                        |
|                          +--------------------------------------------------------+
|  [OPTIMIZATION MODE]     |             [4. 4D TIMELINE & TELEMETRY]               |
|  (o) Min Time (Parallel) |   [▶ PLAY] [❚❚ PAUSE]  ━━━●━━━━━━━━━━━━ [T+00:45:12]   |
|  ( ) Min Energy (-64%)   |   • UAV 1: Geoscan 201 | Alt: 388m | Spd: 28m/s | SOC: 82% |
|                          |   • UAV 2: Geoscan 801 | Alt: 100m | Spd: 15m/s | SOC: 64% |
|  [⚡ COMPUTE MISSION]    |   • Multi-Sortie Gantt Chart with 10m Battery Swaps    |
+--------------------------+--------------------------------------------------------+
```

---

## 🎯 3. Step-by-Step Mission Planning Workflow

### Step 1: Select Your Fleet & Sensor Payloads
In the left sidebar under **Fleet Selection**:
- Check the UAV models you wish to dispatch:
  - **Geoscan 201 (Fixed-Wing):** For large territorial surveys ($V_a = 23.6\text{ m/s}$, endurance up to $3\text{ h}$, parachute recovery).
  - **Geoscan 801 (Heavy VTOL):** For high-density LiDAR and magnetic anomaly mapping ($15\text{ kg}$ MTOW, hover capability).
  - **Geoscan Gemini (Geodetic Multirotor):** For precision photogrammetry and facility inspection (Sony RX1R II 42 MP).
- Select the attached sensor for each aircraft (RGB Sony RX1R II, Sony A6000, FLIR Duo Pro R, or LiDAR AGM-MS3).

### Step 2: Configure Environment & Quality Parameters
- **Wind Speed ($V_w$):** Enter current or forecasted wind velocity ($0 - 15\text{ m/s}$).
- **Wind Direction ($\psi_w$):** Set wind azimuth ($0^\circ - 360^\circ$). The system will automatically compute groundspeed and drift vectors.
- **Ambient Temperature:** Set temperature (e.g. $-15^\circ\text{C}$ or $-30^\circ\text{C}$). The system automatically activates Arctic Battery Derating mode if $T < 0^\circ\text{C}$.
- **Target GSD:** Enter required ground resolution (e.g. $5.0\text{ cm/px}$). The engine automatically calculates the required altitude $H$ and strip spacing $D_{\text{strip}}$.

### Step 3: Define Survey Polygons & Obstacles
You can define survey areas using either of two methods:
1. **Interactive Map Drawing:**
   - Click the **Polygon Tool** on the map toolbar to click and draw any polygon boundary.
   - Click the **NFZ Tool** to draw restricted airspace zones.
2. **File Upload (Drag & Drop):**
   - Click **"📂 Import GeoJSON / KML"** and select a standard GIS polygon file.
   - The boundary, coordinates, and any defined No-Fly Zones will be immediately projected onto the map.

### Step 4: Choose Optimization Criterion
- **Minimum Time (Min Makespan):** Maximizes simultaneous multi-UAV parallelization. Recommended for urgent surveys or short daylight windows.
- **Minimum Cost / Energy (Min TCO):** Optimizes for lowest battery cycle degradation, lowest crew turnaround costs, and lowest flight-hour cost (delivers up to **$-64.3\%$** savings).

### Step 5: Compute Optimal Mission
Click the **"⚡ Compute Mission Plan"** button. The backend engine will:
1. Execute Boustrophedon Cellular Decomposition (BCD).
2. Distribute cells across available aircraft according to kinematic limits.
3. Generate curvature-constrained Dubins paths ($R \ge 45\text{ m}$) with Omega bulb turns.
4. Calculate altitude echeloning ($\ge 50\text{ m}$ separation).
5. Render synthesized 3D flight paths on the map in seconds.

---

## 🎛️ 4. Advanced Tactical Features

### 4.1. 4D Timeline Scrubber & Flight Simulation
- Use the **Timeline Slider** at the bottom to scrub forward and backward through the mission from $T+00:00$ to completion.
- Click **▶ Play** to run real-time or accelerated multi-UAV flight simulation.
- Watch animated UAV icons move across the map, demonstrating synchronized coverage, altitude separation, and safe return to base.

### 4.2. Parachute Drift Cone & Alternate Landing Site
- For the Geoscan 201, click the **"🪂 Calculate Parachute Drift"** button.
- The platform calculates the exact airborne release point based on wind velocity:
  $$\Delta L = V_{\text{wind}} \cdot \frac{H_{\text{alt}}}{V_{\text{descent}}}$$
- The map draws:
  - Cyan release point.
  - Wind drift trajectory arrow.
  - Safe touchdown landing pad with dispersion safety ellipse.
  - Alternate Emergency Runway/Pad #1 if wind exceeds safe envelope.

### 4.3. Dynamic In-Flight NFZ Injection
- To simulate an emergency NOTAM or newly detected restricted airspace during an ongoing mission:
  1. Click **"⚠️ Inject In-Flight NFZ"**.
  2. Draw or confirm the obstacle coordinates on the map.
  3. The dispatcher instantly intercepts active UAV trajectories, invokes the Visibility Graph / A* rerouter, and displays the collision-free evasive path with safety margins.

### 4.4. Multi-Sortie Gantt Chart
- When surveying mega-scale territories ($>3000\text{ ha}$), UAV battery capacity requires multiple flights.
- The **Gantt Chart panel** displays each sortie sequence with:
  - Flight phase duration (Takeoff, Survey, Return).
  - 10-minute ground battery swap intervals ($t_{\text{swap}} = 10\text{ min}$).
  - Continuation waypoint handover.

---

## 💾 5. Exporting Mission Plans for Field Autopilots

Once the mission is computed, export industry-standard flight files using the **Export Menu**:

| Export Format | Standard | Target Autopilot / Software | Usage |
| :--- | :--- | :--- | :--- |
| **QGroundControl (`.plan`)** | QGC v3.0 / MAVLink | Geoscan Planner, PX4, ArduPilot | Load directly onto tablet or GCS ground station |
| **GeoJSON (`.geojson`)** | RFC 7946 | QGIS, ArcGIS, Web GIS | Geospatial validation, boundary layers, reporting |
| **Google Earth (`.kml`)** | OGC KML 2.2 | Google Earth Pro, 3D GIS | 3D terrain profile visualization with clamp-to-ground |
| **MAVLink Waypoints** | MAVLink v2.0 | Embedded UAV Autopilot | Binary packet upload over telemetry radio link |

---

## 🔧 6. REST API Reference Summary

The web frontend communicates with the FastAPI backend through these RESTful endpoints:

- `POST /api/optimize`: Primary mission solver (accepts polygon, fleet selection, wind, temperature, GSD, and optimization criterion).
- `GET /api/fleet`: Returns full technical specifications for Geoscan 201, 801, and Gemini.
- `POST /api/landing/alternate_site`: Calculates parachute drift cone, release coordinates, and alternate landing sites.
- `POST /api/telemetry/inject_nfz`: Dynamically injects emergency restricted airspace and recalculates paths.
- `POST /api/export/qgc`: Generates `.plan` mission file for QGroundControl.
- `POST /api/export/geojson`: Generates RFC 7946 GeoJSON.
- `POST /api/export/kml`: Generates 3D KML with altitude styling.

Full interactive documentation, request schemas, and live test forms are available at:  
👉 **[http://127.0.0.1:8000/docs](http://127.0.0.1:8000/docs)**
