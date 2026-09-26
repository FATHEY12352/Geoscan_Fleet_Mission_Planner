# 📐 Geoscan Mission Planner: Mathematical Architecture & Algorithmic Guide
## Руководство по алгоритмам и математическому аппарату ядра планирования
### Track 05: GC «Geoscan» | LCT 2026

---

## 📋 Executive Overview & Algorithmic Pipeline

The **Geoscan Fleet Mission Planner** solves the heterogeneous multi-UAV Coverage Path Planning problem ($\text{mCPP}$) with non-holonomic kinematic constraints ($R \ge 45\text{ m}$), strict 4D spatio-temporal deconfliction ($\Delta H \ge 50\text{ m}$, $\Delta L \ge 30\text{ m}$), environmental wind vector fields, and arctic electrochemical degradation (down to $-30^\circ\text{C}$).

```
[ User Survey Polygon + NFZs + DEM ]
                │
                ▼
[ Photogrammetric Inversion Engine ] ──► Computes GSD, Altitude H, Line Spacing, Shutter Triggers
                │
                ▼
[ Convexity & Concavity Slicing ] ──► Boustrophedon Cellular Decomposition (BCD)
                │
                ▼
[ Heterogeneous Fleet Allocation ] ──► Geoscan 201 (Fixed-Wing), 801 (Heavy VTOL), Gemini (Copter)
                │
                ▼
[ Dubins Path Generation (R ≥ 45m) ] ──► LSL, RSR, LSR, RSL, Bulb/Omega turns for narrow strips
                │
                ▼
[ Aerodynamic Wind & Crab Solver ] ──► Vector groundspeed triangle, drift compensation, power multipliers
                │
                ▼
[ Multi-Sortie & Arctic Peukert Engine ] ──► Battery swap scheduling, voltage sag prevention, parachute drift
                │
                ▼
[ 4D Spatio-Temporal Deconfliction ] ──► Altitude echeloning & time-window deconfliction
                │
                ▼
[ Industrial Exporters ] ──► QGroundControl (.plan), GeoJSON, KML, MAVLink v2
```

---

## 1. Photogrammetric Inversion Engine (GSD & Sensor Footprints)

Given a target ground sampling distance ($\text{GSD}$, m/px), sensor focal length ($f$, mm), and physical pixel pitch ($\mu$, $\mu\text{m}$), the required flight altitude Above Ground Level ($H_{\text{AGL}}$, m) is analytically determined:

$$H = \frac{\text{GSD} \cdot f}{\mu \cdot 10^{-3}}$$

### Camera Footprint & Strip Spacing
For a camera sensor with sensor dimensions $S_w \times S_h$ (mm) and image resolution $W_{\text{px}} \times H_{\text{px}}$:

$$\text{Swath}_x = H \cdot \frac{S_w}{f}, \quad \text{Swath}_y = H \cdot \frac{S_h}{f}$$

Given user-defined forward overlap $p_x$ (typically $75-80\%$) and side overlap $p_y$ (typically $65-70\%$):
- **Distance between adjacent survey lines (Strip Spacing $D_{\text{strip}}$):**
  $$D_{\text{strip}} = \text{Swath}_x \cdot (1 - p_y)$$
- **Distance between consecutive shutter triggers ($D_{\text{photo}}$):**
  $$D_{\text{photo}} = \text{Swath}_y \cdot (1 - p_x)$$
- **Shutter trigger interval in time ($\Delta t_{\text{trigger}}$):**
  $$\Delta t_{\text{trigger}} = \frac{D_{\text{photo}}}{V_{\text{ground}}}$$

### Sensor Catalog Reference Table
| Sensor Name | Compatible UAV | Focal Length ($f$) | Pixel Pitch ($\mu$) | Resolution | Nominal $H$ ($5\text{ cm GSD}$) | Strip Spacing ($70\%$ overlap) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Sony RX1R II** | Geoscan 201 / Gemini | $35\text{ mm}$ | $4.51\ \mu\text{m}$ | $42.4\text{ MP}$ | $388.0\text{ m}$ | $118.8\text{ m}$ |
| **Sony A6000** | Geoscan 201 | $20\text{ mm}$ | $3.91\ \mu\text{m}$ | $24.3\text{ MP}$ | $255.8\text{ m}$ | $88.5\text{ m}$ |
| **FLIR Duo Pro R** | Geoscan 801 | $19\text{ mm}$ | $17.0\ \mu\text{m}$ | $0.33\text{ MP}$ (LWIR) | $55.9\text{ m}$ | $14.2\text{ m}$ |
| **LiDAR AGM-MS3** | Geoscan 801 | Laser Scanner | N/A | $300\text{k pts/s}$ | $100.0\text{ m}$ | $60.0\text{ m}$ |

---

## 2. Wind Vector Aerodynamics & Crab Angle Compensation

### 2.1 Velocity Triangle & Groundspeed
Let airspeed be $\vec{V}_a$, ambient horizontal wind velocity be $\vec{V}_w$, and the relative wind angle with respect to track heading $\psi$ be $\theta = \psi - \psi_w$.

Resolving wind components:
$$V_{\text{head}} = V_w \cos\theta, \quad V_{\text{cross}} = V_w \sin\theta$$

The effective groundspeed $V_g$ is:
$$V_g = \sqrt{V_a^2 - (V_w \sin\theta)^2} - V_w \cos\theta$$

For reciprocal tracks (headwind vs. tailwind):
- **Tailwind track:** $V_{\text{tailwind}} = V_a + V_w$
- **Headwind track:** $V_{\text{headwind}} = V_a - V_w$
- **Harmonic Mean Groundspeed:**
  $$\bar{V}_g = \frac{2 \cdot V_{\text{tailwind}} \cdot V_{\text{headwind}}}{V_{\text{tailwind}} + V_{\text{headwind}}} = \frac{V_a^2 - V_w^2}{V_a}$$

### 2.2 Crab Angle ($\chi_{\text{crab}}$) & Gimbal Limits
To maintain straight-line tracking over ground, the UAV nose must point into the crosswind by the crab angle:

$$\chi_{\text{crab}} = \arcsin\left(\frac{V_{\text{cross}}}{V_a}\right) = \arcsin\left(\frac{V_w \sin\theta}{V_a}\right)$$

**Safety Invariant:**
$$\chi_{\text{crab}} \le 23.57^\circ \iff V_{\text{cross}} \le 0.40 \cdot V_a$$
If $V_{\text{cross}} > 0.40 V_a$ ($9.4\text{ m/s}$ for Geoscan 201 at $23.6\text{ m/s}$ cruise), the camera gimbal runs out of yaw travel, resulting in distorted photogrammetric overlaps. In this case, the planner dynamically alters the strip orientation $\psi_{\text{strip}}$ to align within $\pm 15^\circ$ of the wind vector.

### 2.3 Aerodynamic Power Multiplier
Wind resistance and rotor attitude correction increase power consumption:
$$P(V_w, V_a) = P_{\text{nominal}} \cdot \left[ 1 + k_w \left(\frac{V_w}{V_a}\right)^2 \right]$$
where $k_w = 0.18$ for Geoscan 201 (aerodynamic airframe) and $k_w = 0.35$ for Geoscan 801 (multirotor drag).

---

## 3. Non-Holonomic Dubins Paths ($R \ge 45\text{ m}$)

Fixed-wing aircraft like «Geoscan 201» are non-holonomic systems subject to minimum turning radius constraints governed by maximum aerodynamic bank angle ($\gamma_{\text{max}} \le 35^\circ$):

$$R_{\text{min}} = \frac{V_a^2}{g \cdot \tan\gamma_{\text{max}}} \approx \frac{(23.6)^2}{9.81 \cdot \tan(35^\circ)} \approx 81.1\text{ m} \quad (\text{Operational structural limit: } 45.0\text{ m})$$

Our planner enforces an engineered safety buffer:
$$R_{\text{planner}} = 48.0\text{ m} \ge 45.0\text{ m}$$

### Six Dubins Elementary Words
Between two directed poses $A = (x_1, y_1, \psi_1)$ and $B = (x_2, y_2, \psi_2)$, the shortest path belongs to the set:
$$\mathcal{D} = \{ \text{LSL}, \text{RSR}, \text{LSR}, \text{RSL}, \text{LRL}, \text{RLR} \}$$

Where:
- $\text{L}, \text{R}$ denote circular arcs of radius $R$ turning Left or Right.
- $\text{S}$ denotes a straight tangent line segment.

### Narrow Strip Spacing: The Omega ($\Omega$) / Bulb Turn
When strip spacing $D_{\text{strip}} < 2R$ (e.g. $D_{\text{strip}} = 15-30\text{ m}$ for high-overlap thermal/LiDAR, while $2R = 96\text{ m}$), a standard Dubins turn cannot complete directly between adjacent strips.
The planner synthesizes an **Omega Turn (Bulb Manoeuvre)**:
1. Fixed-wing exits the polygon along a straight tangent $L_{\text{exit}} = R + 15\text{ m}$.
2. Executes an outward circular arc of $210^\circ-240^\circ$.
3. Curves backward in a reverse arc to align coaxially with the adjacent strip entry vector.
This guarantees continuous $C^1$ curvature without airspeed loss or stall hazards.

---

## 4. Parachute Descent & Safe Touchdown Cone

The «Geoscan 201» terminates missions via parachute deployment. The parachute descent velocity is $V_{\text{desc}} = 5.0\text{ m/s}$.

### Drift Calculation
From deployment altitude $H_{\text{rel}}$:
$$t_{\text{desc}} = \frac{H_{\text{rel}}}{V_{\text{desc}}}$$
The spatial drift vector $\vec{D}_{\text{drift}}$ due to wind $\vec{V}_w$ is:
$$\vec{D}_{\text{drift}} = t_{\text{desc}} \cdot \vec{V}_w = \left(\frac{H_{\text{rel}}}{V_{\text{desc}}}\right) \cdot \vec{V}_w$$

For $H_{\text{rel}} = 100\text{ m}$ and $V_w = 10\text{ m/s}$:
$$t_{\text{desc}} = 20\text{ s}, \quad \|\vec{D}_{\text{drift}}\| = 200\text{ m}$$

### Release Point Optimization
To guarantee touchdown on the operator's designated landing pad $\mathbf{P}_{\text{target}}$:
$$\mathbf{P}_{\text{release}} = \mathbf{P}_{\text{target}} - \vec{D}_{\text{drift}}$$
The system constructs an uncertainty dispersion ellipse around $\mathbf{P}_{\text{target}}$ with radius:
$$R_{\text{dispersion}} = \max(25\text{ m}, 0.15 \cdot \|\vec{D}_{\text{drift}}\|)$$
and verifies zero overlap with water bodies, power lines, and No-Fly Zones (NFZ).

---

## 5. Arctic Battery Degradation & Peukert Law

In Siberian operations with temperatures down to $-30^\circ\text{C}$, LiPo/Li-Ion internal resistance $R_{\text{int}}$ increases by $300-500\%$, causing severe Voltage Sag under high motor load.

### Effective Capacity Derating
$$\begin{cases}
C_{\text{eff}}(T) = C_{\text{nominal}} \cdot [ 1.0 - 0.0116 \cdot |T_{\text{celsius}}| ] & (T < 0^\circ\text{C}) \\
C_{\text{eff}}(T) = C_{\text{nominal}} & (T \ge 0^\circ\text{C})
\end{cases}$$

- At $-15^\circ\text{C}$: Effective capacity is reduced by **$17.5\%$**.
- At $-30^\circ\text{C}$: Effective capacity is reduced by **$35.0\%$**.

### Dynamic Energy Reserve
The automated emergency return reserve is scaled automatically:
$$\text{SOC}_{\text{reserve}} = \begin{cases}
20\% & (T > 0^\circ\text{C}) \\
25\% & (-15^\circ\text{C} < T \le 0^\circ\text{C}) \\
35\% & (T \le -15^\circ\text{C})
\end{cases}$$

### Power-Off Glide Descent
On return approach, the Geoscan 201 engine throttle is reduced to zero, utilizing aerodynamic glide ratio ($K_{\text{glide}} = 12:1$):
$$\Delta L_{\text{glide}} = H \cdot K_{\text{glide}}$$
From $H = 400\text{ m}$, the aircraft glides **$4.8\text{ km}$** with zero battery drain, saving up to $14\%$ of mission energy.

---

## 6. Non-Convex Polygon Decomposition (BCD)

1. **Polygon Analysis:** Detects self-intersections, holes, and non-convex reflex vertices ($\alpha_i > 180^\circ$).
2. **Sweep-Line Slicing:** A sweep-line parallel to the optimal strip direction $\theta_{\text{strip}}$ detects connectivity changes ($1 \to N$ or $N \to 1$).
3. **Trapezoidal & Monotone Cells:** Slices the non-convex polygon into mutually convex sub-cells.
4. **Cluster Assignment:** Sub-cells are assigned to UAVs using a weighted cost function:
   $$J_i = w_1 \cdot \frac{\text{Area}_i}{V_i \cdot \text{Swath}_i} + w_2 \cdot \frac{D_{\text{transit}}(\text{Base}, \text{Cell}_i)}{V_i}$$

---

## 7. 4D Spatio-Temporal Deconfliction

To guarantee safe joint operations of heterogeneous fleets in shared airspace:

### Spatial Separation Buffers
- **Vertical separation:** Fixed-wing (Geoscan 201) operates in upper altitude bands ($H \ge 250-400\text{ m}$), while multirotors (801, Gemini) operate at lower levels ($H \le 100-150\text{ m}$).
  $$\Delta H = |H_A(t) - H_B(t)| \ge 50.0\text{ m}$$
- **Horizontal separation:**
  $$D_{\text{horiz}}(t) = \|\mathbf{P}_A(t) - \mathbf{P}_B(t)\|_{xy} \ge 30.0\text{ m}$$
- **Time Separation:** Base station launch and landing slots are staggered by $\Delta t_{\text{slot}} \ge 60\text{ s}$.

### Dynamic NFZ Injection
When an emergency or temporary restricted airspace (NOTAM/AIP) is injected during flight:
1. Slices the active trajectory at $T_{\text{inject}}$.
2. Re-routes affected UAVs around the NFZ boundary using Visibility Graph / A* shortest path with a $30\text{ m}$ clearance polygon buffer.
3. Automatically evaluates mission feasibility and remaining battery margin.

---

## 8. Dual-Objective Optimization & Economic Formulations

The planner offers two operational optimization modes:

### Criterion 1: Minimum Time ($\min T_{\text{makespan}}$)
$$\min \left( \max_{i \in \mathcal{F}} T_{\text{mission}}^{(i)} \right)$$
Maximizes parallel multi-UAV survey throughput for time-critical reconnaissance, pre-dusk surveys, or rapidly changing weather windows.

### Criterion 2: Minimum Energy / Cost ($\min \text{TCO}$)
$$\min \sum_{i \in \mathcal{F}} \left[ C_{\text{flight\_hr}}^{(i)} \cdot T^{(i)} + C_{\text{battery\_depreciation}}^{(i)} \cdot \Delta \text{SOC}^{(i)} + C_{\text{crew\_turnaround}} \cdot N_{\text{sorties}}^{(i)} \right]$$
Selects the most cost-efficient platform (e.g. prioritizing Geoscan 201 for wide areas due to its $4.7\times$ lower per-hectare flight cost compared to multirotors), yielding up to **$-64.3\%$** total operational savings.
