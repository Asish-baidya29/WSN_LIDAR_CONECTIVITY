# Autonomous Drone Landing System — Full Build Spec
### Folder structure, file responsibilities, and detailed content overview
*(No code yet — this is the blueprint to hand to a coding model/developer so implementation can start directly.)*

---

## 0. Confirmed project shape (from all prior discussion)

- **No GPS, no camera** in the first working version.
- **7 ZigBee boards (WSNADK_EK / ATmega324PM / XBee Pro)**:
  - 4× **Router** — staked at zone corners (boundary)
  - 1× **Router** — staked at zone center (reference point)
  - 1× **Coordinator** — ground station, wired via USB/serial to the laptop
  - 1× **End device** — mounted on the drone
- **Drone (End device)** locally determines in-zone/out-of-zone from RSSI of nearby Routers, homes toward the zone if outside, then runs a servo-swept LiDAR (TF03-180) scan once inside to find the flattest landing patch.
- **Ground laptop** runs `listener.py` (already exists) which reads the Coordinator's serial output and logs everything to a MySQL database (`wvs`) for monitoring/dashboard — **not** part of the drone's real-time decision loop.
- Flight-controller/MAVLink integration is a **later phase**, out of scope for this build pass — note the hook point only.

---

## 1. Top-level folder structure

```
drone-landing-system/
│
├── firmware/                     # mikroC PRO for AVR project — burned to all 7 boards
│   ├── rtss.h                    # shared pin/macro/prototype header (extend existing)
│   ├── rtss.c                    # main() + role dispatch (Coordinator / Router / End device)
│   ├── rtss_zone.c               # NEW — zone detection + homing logic (End device only)
│   ├── rtss_lidar.c              # NEW — servo sweep + LiDAR read + flatness scoring (End device only)
│   ├── rtss_packets.c            # existing pkt_to_coo_*, pkt_broadcast, etc. (unchanged, kept as-is or split out)
│   └── project.mcppav            # mikroC project file — lists all .c files above for one build target
│
├── backend/                      # Python service on the laptop, wired to the Coordinator
│   ├── main.py                   # existing — starts listener thread + web/WebSocket server
│   ├── config.py                 # existing — extend with new packet tags + serial settings
│   ├── database.py               # existing — session/procedure/query helpers (unchanged)
│   ├── listener.py               # existing — EXTEND with new parser functions (see §3)
│   ├── websocket_manager.py      # existing — unchanged, broadcasts new event types too
│   └── requirements.txt          # existing
│
├── database/
│   ing── wvs_schema.sql          # existing schema (from wvs.sql) — base, unchanged
│   └── wvs_landing_extension.sql # NEW — additive migration: new tables/rows for this project
│
└── docs/
    ├── System_Build_Spec.md      # this file
    └── packet_format_reference.md # NEW — single source of truth for every wire packet format
```

*(If your mikroC setup makes multi-file projects awkward, `rtss_zone.c` and `rtss_lidar.c` can be pasted directly into `rtss.c` as clearly marked sections instead — functionally identical, just less separated. Flag this to me before the coder starts if you want single-file only.)*

---

## 2. Firmware layer (`firmware/`) — detailed content per file

### 2.1 `rtss.h` — additions needed on top of the existing header

| New macro / prototype | Purpose |
|---|---|
| `#define SERVO_PWM_PIN ...` + `DDRx.DDxN=1` dir macro | Pin driving the servo signal line (pick a free pin, e.g. spare `PORTD` bit) |
| `#define LIDAR_UART...` — **use UART1** on the End-device board (confirmed: End device = UART0→XBee, UART1→LiDAR; this is swapped vs. the Coordinator board's UART0→PC/UART1→XBee) | TF03-180 RX/TX — no 3rd UART or bit-banging needed, ATmega324A only has 2 UARTs total |
| `void Init_Servo(void);` | Prototype |
| `void Set_Servo_Angle(unsigned char angle_deg);` | Prototype |
| `void Init_Lidar(void);` | Prototype |
| `unsigned int Read_Lidar_Distance(void);` | Prototype |
| `unsigned char Is_Inside_Zone(void);` | Prototype (End device local zone check) |
| `void Home_Toward_Zone(void);` | Prototype (RSSI-based nudge logic, produces a direction hint) |
| `void Run_Lidar_Sweep(void);` | Prototype (drives servo across arc, samples LiDAR, fills grid array) |
| `unsigned char Find_Best_Cell(void);` | Prototype (variance scoring, returns index/coords of flattest cell) |
| `void Send_Zone_Status_To_PC(void);` | Prototype (relay zone in/out to Coordinator via nearest Router — logging only) |
| `void Send_Lidar_Data_To_PC(void);` | Prototype (relay scan points/best cell to Coordinator — logging only) |
| New globals: `unsigned int lidar_grid[N][N];`, `unsigned char best_cell_x, best_cell_y, best_cell_score;`, `unsigned char zone_status;` | Shared state for the scan pipeline |

### 2.2 `rtss.c` — what stays the same vs. what changes per role

- **Coordinator (`case 'C':`)** — unchanged from current file. Its only job: receive relayed packets from Routers/End device, forward to PC over UART0. (`Send_RSSI_To_PC()` addition from earlier can stay or be removed if zone-status now comes from the End device instead — **decide**: keep Coordinator dumb/passthrough, since zone detection moved to the End device.)
- **Router (`case 'R':`)** — unchanged. Already broadcasts health/RSSI and relays. No new code needed for boundary or center Routers — same firmware for all 5.
- **End device (`case 'E':`)** — this is where nearly all new logic plugs in, inside the existing `while(1)` loop, alongside the existing `sec_cnt_sensor` / `sec_cnt_health` timed blocks:
  1. New periodic block (e.g. every 2–5 sec): call `Is_Inside_Zone()` using the End device's own `dbm[]`/`node[]` table (already populated by existing `scan_pkt_received` handling — no change needed there).
  2. If outside zone → call `Home_Toward_Zone()` (logic: find strongest-RSSI Router in `dbm[]`, output a simple direction hint — for v1 this can just be a printed/logged suggestion, since there's no flight-controller link yet in this phase).
  3. If inside zone → call `Run_Lidar_Sweep()` once, then `Find_Best_Cell()`.
  4. Relay results opportunistically (existing `Sort_decending_node_signal_strength()` + `Send_data()` pattern, same as current sensor-sending code) via `Send_Zone_Status_To_PC()` / `Send_Lidar_Data_To_PC()`.

### 2.3 `rtss_zone.c` (new) — content overview

- `Is_Inside_Zone(void)` — reuse logic already drafted earlier: counts Routers in `dbm[]` below a threshold, returns 1/0. **Threshold + min-beacon-count need to be `#define`s here, tunable after field calibration.**
- `Home_Toward_Zone(void)` — for v1 (no flight controller yet): identify strongest Router by RSSI, output/log which Router that is (e.g. `"HOME:TR002"` over the relay path) — this is the seed for later feeding a real direction vector to the flight controller once each Router's fixed position is known.

### 2.4 `rtss_lidar.c` (new) — content overview

- `Init_Servo(void)` — configure PWM timer or GPIO+delay bit-bang for the servo signal pin.
- `Set_Servo_Angle(angle)` — move servo to a given angle (e.g. 0–180, mapped to pulse width).
- `Init_Lidar(void)` — configure whichever UART/pins are assigned to TF03-180, matching its datasheet frame format (confirm baud rate + frame header bytes before writing this — **open item**).
- `Read_Lidar_Distance(void)` — parse one TF03-180 frame, return distance in cm.
- `Run_Lidar_Sweep(void)` — loop: step servo through fixed arc (e.g. -45° to +45° in N steps), call `Read_Lidar_Distance()` at each step, store into `lidar_grid[]`.
- `Find_Best_Cell(void)` — compute variance per cell/window of `lidar_grid[]`, return index of lowest-variance (flattest) region into `best_cell_x/y/score`.
- **Keep this simple per the ATmega324PM's RAM/flash limits** — small fixed-size grid (e.g. 5×5 or 1×N line profile for v1), basic variance math only, no floating point if avoidable.

### 2.5 Packet formats this firmware will need to send (define once, reuse everywhere — see `docs/packet_format_reference.md`)

| New packet | Format | Sent by | Purpose |
|---|---|---|---|
| Zone status | `Z<node_id>**<0 or 1>**<parent>` | End device | inside/outside flag |
| LiDAR scan point | `L<node_id>**<scan_seq>,<angle>,<distance>**<parent>` | End device | one sweep sample, correlated to a scan session via scan_seq |
| Best landing cell | `B<node_id>**<x>,<y>,<score>**<parent>` | End device | final chosen landing point per scan cycle |

*(These are new prefixes, distinct from the existing `S` sensor-packet reuse trick discussed earlier — recommended because LiDAR angle/distance pairs don't fit the single-value sensor format cleanly. Finalize exact prefix letters before coding so firmware and `listener.py` agree.)*

---

## 3. Backend layer (`backend/`) — detailed content per file

### 3.1 `config.py` — additions

```
ZONE_TAG = "Z"
LIDAR_POINT_TAG = "L"
BEST_CELL_TAG = "B"
```
(alongside existing `SENSOR_TAG`, `END_DEV_TAG`, `ROUTER_TAG`, etc.)

### 3.2 `listener.py` — new parser functions to add (same pattern as existing `parse_sensor_packet` etc.)

- `parse_zone_packet(fields, db)` — inserts into new `zone_status_log` table.
- `parse_lidar_point_packet(fields, db)` — inserts into existing `lidar_point` table (already in your schema — confirm column names match, see §4).
- `parse_best_cell_packet(fields, db)` — inserts into a new `landing_decision_log` table.
- New `elif` branches in `dispatch_packet()` routing to each, following the exact same `if prefix.startswith(...)` pattern already used for `S`/`TE`/`TR`/`ME`/etc.
- New WebSocket `event_type` strings (`"zone_update"`, `"lidar_point"`, `"landing_decision"`) so the dashboard can react live — reuses existing `ws_manager.broadcast()` call, no new mechanism needed.

### 3.3 `database.py` — no changes expected; existing `call_procedure`/`fetch_one`/`execute_sql` helpers are reused as-is.

---

## 4. Database layer (`database/wvs_landing_extension.sql`) — new content

Your schema already has empty `lidar_scan`, `lidar_point`, `lidar_object`, `lidar_config` tables — **reuse these rather than creating duplicates**; the extension file should:

1. `DESCRIBE` / confirm existing column names of `lidar_scan`/`lidar_point` match what `parse_lidar_point_packet` will insert (angle, distance, node_id, timestamp) — **open item: need their actual column list, not yet seen in the dump reviewed so far**.
2. Add `zone_status_log` table (if not equivalent to an existing one): `node_id`, `status` (in/out), `receive_time`.
3. Add `landing_decision_log` table: `node_id`, `cell_x`, `cell_y`, `flatness_score`, `receive_time`.
4. Add `sensor_type_master` row for RSSI (`'RS'`) — from earlier discussion, still needed if RSSI keeps riding the sensor-packet format for the Coordinator/Router side.

---

## 5. `docs/packet_format_reference.md` — new file, content

A single table listing **every** packet prefix, its exact field layout, which board sends it, and which DB table/column it lands in — the contract both the firmware coder and the backend coder build against, so nothing drifts out of sync. This should be written **before** either side's code, once the prefixes in §2.5 are finalized.

---

## 6. Open items — status after your hardware confirmation

1. ~~`lidar_scan`/`lidar_point` column names~~ → **Resolved:** tables don't exist yet in `wvs_1.sql`; fresh `CREATE TABLE` statements written in `database/wvs_landing_extension.sql`.
2. ~~Which UART is free for the LiDAR~~ → **Resolved (confirmed by you):** ATmega324A has exactly 2 UARTs, no 3rd/bit-banging needed. On the **drone's End-device board**: UART0 → XBee, UART1 → TF03-180 LiDAR — this is the opposite pin assignment from the Coordinator board (UART0=PC, UART1=XBee), so `rtss.c`'s `Initialize_uart()` needs a role-specific variant, not shared blindly across Coordinator and End-device builds.
3. ~~TF03-180 frame format/rate~~ → **Resolved (confirmed by you):** default 100 Hz, configurable 1–9,800 Hz. `Run_Lidar_Sweep()` should pace servo steps no faster than the configured output rate (default 100 Hz / ~10 ms per frame is already plenty for a 10–15 step arc sweep — no reconfiguration needed for v1).
4. ~~Z/L/B packet prefixes~~ → **Resolved:** no collisions, defined in `packet_format_reference.md`.
5. ~~`scan_id` correlation approach~~ → **Resolved — Option B confirmed:** firmware sends explicit `scan_seq` in every `L` packet; format locked as `L<node>**<scan_seq>,<angle>,<distance>**<parent>`.
6. ~~`LED1_on`/`LED1_off` macros~~ → **Resolved:** confirmed present in `rtssh.h`.
7. **Still open:** whether `Home_Toward_Zone()` in v1 just logs a suggestion, or drives anything physically, until the flight-controller phase begins.
8. **Still open (small firmware task):** `Byte_To_Ascii()` from earlier only handles 0–255 — LiDAR distance (up to ~18,000 cm) needs a 16-bit version, e.g. `Uint16_To_Ascii()`, in `rtss_lidar.c`.
9. **Still open:** confirm `node_type_master`'s type code for "End device" before finalizing the `node_master` insert for the drone's node_id in the SQL extension (currently commented out as a placeholder).

**Everything needed to start coding `rtss_lidar.c`, `rtss_zone.c`, and the corresponding `listener.py` parser additions is now confirmed except items 7–9, none of which block starting the work.**

