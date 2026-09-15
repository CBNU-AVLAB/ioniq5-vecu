# Exercise 0 — CAN by hand (cansend / candump)

Before writing Python, build a few CAN frames by hand. Every command below was checked by
decoding it with the dbc.

## 0. Setup

```bash
./scripts/setup_vcan.sh                                  # bring up the virtual CAN (safe to repeat)
PYTHONPATH=src .venv/bin/python -m ioniq5_vecu.vecu      # terminal 1: vECU
candump vcan0                                            # terminal 2: watch the bus
```

Frames arriving every 10 ms in terminal 2 are the ECU status frames.

## 1. candump — watch the bus

```bash
candump vcan0                      # everything
candump vcan0,104:7FF              # ID 0x104 only (steering feedback)
candump vcan0,204:7FF              # ID 0x204 only (brake feedback)
candump -t d vcan0,104:7FF         # time between frames -> check the 10 ms cycle
candump vcan0,7A0:7F8,7A8:7F8      # diagnostic requests/responses only (exercise 2)
```

A filter is `ID:mask`: `104:7FF` matches frames whose ID masked with 0x7FF equals 0x104.

Reading the output:

```
  vcan0  104   [8]  18 15 00 00 18 15 00 00
         ^ID  ^DLC  ^8 data bytes
```

## 2. cansend — send a command

Syntax: `cansend <interface> <ID>#<data in hex>`, with the bytes written together.

### Common commands

| Action | Command | Meaning |
|---|---|---|
| Steering servo ON | `cansend vcan0 100#E701FF` | SON=1 (+PC·TL·EMG·LSP·LSN·LOP=1, torque limit 100%) |
| Steering servo OFF | `cansend vcan0 100#E601FF` | SON=0 — manual steering works again |
| Steering +90 deg | `cansend vcan0 101#18150000` | target_pos = 90.0 deg |
| Steering -90 deg | `cansend vcan0 101#E8EAFFFF` | target_pos = -90.0 deg |
| Steering 400 deg | `cansend vcan0 101#C05D0000` | target_pos = 400.0 deg (used in exercise 2) |
| Steering 0 deg | `cansend vcan0 101#00000000` | target_pos = 0.0 deg |
| Brake servo ON | `cansend vcan0 200#E701FF` | same controller as steering, IDs 0x2xx |
| Brake 20 mm | `cansend vcan0 201#D0070000` | target_pos = 20.0 mm |
| Brake 170 mm (full) | `cansend vcan0 201#68420000` | target_pos = 170.0 mm (full stroke) |
| Accel 5% | `cansend vcan0 311#0400CD0C00000000` | AD override ON, APS 5% |
| Accel 50% | `cansend vcan0 311#0400FF7F00000000` | AD override ON, APS 50% |
| Accel release | `cansend vcan0 311#0000000000000000` | override OFF — the driver pedal passes through |

A servo follows position commands only **after SON=1**. With SON=0 it ignores them and
follows the keyboard (manual control) instead.

## 3. Where the data bytes come from — little-endian

The steering angle is defined in the dbc as **resolution 1/60 deg, 32 bits, little-endian**.

```
1) physical -> raw       90 deg ÷ (1/60) = 5400
2) hex                   5400 = 0x00001518
3) reverse the bytes     00 00 15 18  ->  18 15 00 00      (little-endian: low byte first)
4) result                cansend vcan0 101#18150000
```

Negative values use two's complement:

```
-90 deg ÷ (1/60) = -5400 = 0xFFFFEAE8  ->  E8 EA FF FF  ->  101#E8EAFFFF
```

The brake resolution is **1/100 mm**:

```
20 mm ÷ (1/100) = 2000 = 0x000007D0  ->  D0 07 00 00  ->  201#D0070000
```

> In exercise 1 the dbc and `cantools` do this conversion for you.

## 4. Bit fields — 0x100 and 0x311

In control frames each bit has its own meaning.

`0x100` (steering servo control, 3 bytes):

```
byte0  bit0 SON  bit1 PC  bit2 TL  bit3 RES  bit4 CR  bit5 EMG  bit6 LSP  bit7 LSN
byte1  bit0 LOP  bit1 DO1  bit2 DO2
byte2  TLA (torque limit %, 0xFF = 100%)

E7 = 1110 0111 -> SON=1 PC=1 TL=1 RES=0 CR=0 EMG=1 LSP=1 LSN=1
01 -> LOP=1
FF -> TLA = 255 x 0.392 = 100%
```

`0x311` (accel override, 8 bytes):

```
byte0    bit0 CAL_EN  bit1 OVR_VOLTAGE  bit2 OVR__PERCENT
byte2~3  APS_OVR_PERCENT_VALUE (16 bits, little-endian, resolution 0.00152588 %)

04 = 0000 0100 -> OVR__PERCENT=1 (override by percent)
5% ÷ 0.00152588 = 3277 = 0x0CCD -> CD 0C   (hence 0400CD0C...)
```

> `OVR__PERCENT` has two underscores, exactly as in the CAN matrix.

## 5. Try it

```bash
cansend vcan0 100#E701FF      # servo ON
cansend vcan0 101#18150000    # 90 deg
candump vcan0,104:7FF         # encoder_pos converges to 90 deg
```

- The needle also moves on the cluster (`console/cluster.py`, http://127.0.0.1:8088).
- Send `101#C05D0000` (400 deg). Where does it stop? A limit applies.
- Turn the servo off (`100#E601FF`) and use the arrow keys in `console/input.py`: the same
  `0x104` value now follows the keyboard.

## 6. A first diagnostic frame (preview of exercise 2)

Diagnostics use **request/response**: 0x7A0 (request) and 0x7A8 (response).

```bash
candump vcan0,7A0:7F8,7A8:7F8                 # terminal 2
cansend vcan0 7A0#0322F19500000000            # read the SW version (UDS 22 F1 95)
```

```
7A0#0322F19500000000     03 = 3 bytes of UDS data follow, 22 F1 95 is the request
7A8#0762F195563....      07 = 7 bytes, 62 = 22+0x40 (positive), F1 95 = the DID, then the value
```

Data longer than one frame (e.g. the 17-byte VIN) is split over several frames
(`10..` First Frame -> `30..` Flow Control -> `21.. 22..` Consecutive Frames).

## 7. Troubleshooting

| Symptom | Check |
|---|---|
| `candump` shows nothing | Is the vECU running? Is `ip link show vcan0` UP? |
| `cansend` reports `no buffer space` | Is the interface name `vcan0`? |
| Position commands do nothing | Did you send SON=1 first? (`100#E701FF`) |
| Values look wrong | Bytes reversed (little-endian)? Right resolution (steering 1/60, brake 1/100)? |
