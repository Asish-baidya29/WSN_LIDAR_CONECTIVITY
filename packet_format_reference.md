# Packet Format Reference
### Single source of truth — both firmware (rtss.c) and backend (listener.py) must match this exactly.

All packets are ASCII, `**`-delimited, newline-terminated — same convention as your existing `S`/`TE`/`TR`/`ME`/`AE`/`AR`/`OT` packets.

| Prefix | Full format | Sent by | Meaning | Backend table |
|---|---|---|---|---|
| `S` *(existing)* | `S<type>E<node>**<value>**<parent>` | any node | generic sensor reading (T/H/CO, and RSSI reusing type `3`) | `data_txn` |
| `TE` *(existing)* | `TE<node>**<parent>` | Router | end-device tag seen | `node_txn` |
| `TR` *(existing)* | `TR<node>**<parent>` | Router | router beacon/registration | `node_txn` |
| `ME` *(existing)* | `ME<node>**<parent>**<msg>` | End device | LEFT/RIGHT/EMERGENCY message | `message_txn` |
| `AE`/`AR` *(existing)* | `AE<node>**<parent>` | End device/Router | acknowledgement | `delivery_log` |
| `OT` *(existing)* | `OT<node>**...` | Router | OTAP name request | — |
| **`Z`** *(new)* | `Z<node>**<0 or 1>**<parent>` | End device (drone) | zone status: 1 = inside, 0 = outside | `zone_status_log` |
| **`L`** *(new)* | `L<node>**<angle>,<distance>**<parent>` | End device (drone) | one LiDAR sweep sample | `lidar_point` (needs `scan_id` — see note) |
| **`B`** *(new)* | `B<node>**<x>,<y>,<score>**<parent>` | End device (drone) | chosen flattest landing cell for a scan | `landing_decision_log` |

## scan_id correlation — DECIDED: Option B

Firmware sends an explicit `scan_seq` counter, incremented once per `Run_Lidar_Sweep()` call. Locked format:

```
L<node>**<scan_seq>,<angle>,<distance>**<parent>
```

Backend resolves/creates `lidar_scan.scan_id` by `(node_id, scan_seq)` — no timing-based guessing needed. `parse_lidar_point_packet` in `listener.py` should: look up an existing `lidar_scan` row for `(node_id, scan_seq)`; if none exists, `INSERT` one first, then insert the point against its `scan_id`. The `B` packet (best cell) closes that scan — on receipt, `UPDATE lidar_scan SET end_time = NOW() WHERE node_id=... AND scan_id = (latest for that scan_seq)`.

## Hardware confirmed (End-device / drone board)

- **ATmega324A has exactly 2 UARTs — no 3rd UART, no bit-banging needed.** On the drone's End-device board specifically (not wired to a laptop, so UART0 is free): **UART0 → XBee, UART1 → TF03-180 LiDAR.** This is the *opposite* pin usage from the Coordinator board (where UART0=PC, UART1=XBee) — firmware for the End-device role needs its own `Initialize_uart()` variant reflecting this swapped assignment. Flag this clearly in `rtss.c`/`rtss_lidar.c` so it isn't accidentally copy-pasted with Coordinator-style UART wiring.
- **TF03-180 frame rate: default 100 Hz, configurable 1–9,800 Hz.** For `Run_Lidar_Sweep()`, this means each `Read_Lidar_Distance()` call can safely expect a fresh frame roughly every 10 ms at default rate — servo step timing in `Run_Lidar_Sweep()` should pace itself no faster than the LiDAR's configured output rate, or it'll re-read stale/duplicate frames. If you want fewer, more deliberate steps (e.g. 10-15 angles across the arc), the default 100 Hz is already more than fast enough — no need to reconfigure the LiDAR's frame rate for v1.

## Value ranges to keep in mind (firmware side)

- `angle`: signed range e.g. -45 to +45 — decide if you send as `-45` (needs a sign character) or offset to `0-90` and let the backend re-center. **Recommend offsetting to unsigned 0–90 in the packet**, simpler ASCII encoding, no sign-parsing edge cases.
- `distance`: TF03-180 range up to ~18000 cm — fits in an `unsigned int`, up to 5 ASCII digits, still fine within your byte-oriented packet builder as long as `Byte_To_Ascii`-style helper is upgraded to handle 2-byte values (the earlier `Byte_To_Ascii()` we wrote only handles 0–255 — **needs a 16-bit version for distance**).
- `x`, `y` (best cell): small grid coordinates, e.g. 0–10, fits comfortably in one byte.
- `score`: variance value — pick a scale that stays within a reasonable byte/int range for your grid size before finalizing.
