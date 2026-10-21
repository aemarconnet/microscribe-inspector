# MicroScribe serial protocol & kinematics

© 2026 Marconnet Technologies — licensed under [CC BY 4.0](../LICENSE-docs).

This document describes the RS-232 protocol spoken by serial MicroScribe digitizers (called "HCI" in
open-source drivers) and how stylus positions are computed from it, as implemented by MicroScribe
Inspector. It is written for **interoperability**: so owners of these arms can keep using them.

Every statement is tagged with where it comes from:

| Tag | Source |
|---|---|
| **[M]** | [microcmm](https://github.com/Revise-Robotics/microcmm) — an MIT-licensed, open-source Python driver for the MicroScribe-3D (`hci.py`, `arm.py`) |
| **[V]** | [VRPN](https://github.com/vrpn/vrpn) — an open-source device library that documents the same `IMMC` auto-baud sign-on |
| **[O]** | Our own observation of a MicroScribe 3DX (model "DX", firmware "HCI 2.0", parameter format "Format DH0.5"), recorded with `MsProbe` using only the read-only commands below |

## 1. Transport

| Item | Value |
|---|---|
| Line | RS-232, 8 data bits, no parity, 1 stop bit, no flow control [M][O] |
| Baud | The device detects the baud rate from the `IMMC` sign-on [M][V]. Observed working at 9600 [O]; microcmm uses 115200 [M]. MicroScribe Inspector tries 9600, 115200, 38400, 19200. |
| Adapters | A USB-serial adapter (CH340 tested) works [O]. |
| Framing | In replies, the header byte has bit 7 set and all payload bytes have bit 7 clear [M]. |

## 2. Session

### 2.1 Sign-on [M][V][O]
1. Repeatedly send `0xC5` (END_SESSION, returns a running session to sign-on) followed by ASCII `IMMC`,
   pausing ~15 ms and reading what arrived, until the received bytes contain `IMMC` (the device echoes it).
2. Discard any further echoes.
3. Send ASCII `BEGIN`. The device answers with a NUL-terminated product-ID string. Observed: `MSCR` [O].

### 2.2 End of session [M]
Send `0xC5`.

## 3. Configuration commands [M][O]

The host sends one command byte; the device echoes the same byte and then the payload.

| Byte | Name [M] | Payload after the echo | Used here |
|---|---|---|---|
| C0 | GET_PARAMS | 1 length byte *n*, then *n* bytes [M]; *n* = 36 observed [O] | yes |
| C1 | GET_HOME_REF | 12 bytes observed [O] | diagnostic only |
| C2 | HOME_POS | none [M] — re-zeros the encoders at the current pose | only when the user asks |
| C3 | SET_HOME | — | **never** |
| C4 | SET_BAUD | — | never |
| C5 | END_SESSION | none [M] | yes |
| C6 | GET_MAXES | 24 bytes [M][O] (see 3.2) | yes |
| C7 | SET_PARAMS | — writes calibration | **never** |
| C8 | GET_PROD_NAME | NUL-terminated string [M] — `MicroScribe3D` [O] | yes |
| C9 | GET_PROD_ID | NUL-terminated string [M] — `MSCR` [O] | yes |
| CA | GET_MODEL_NAME | string [M] — `DX` [O] | yes |
| CB | GET_SERNUM | string [M] | yes |
| CC | GET_COMMENT | string [M] — `Standard+Beta` [O]; containing `Beta` means GET_EXT_PARAMS is valid [M] | yes |
| CD | GET_PRM_FORMAT | string [M] — `Format DH0.5` [O] | yes |
| CE | GET_VERSION | string [M] — `HCI 2.0` [O] | yes |
| CF | REPORT_MOTION | streaming mode [M] | no |
| D0 | SET_HOME_REF | — writes calibration | **never** |
| D1 | RESTORE_FACTORY | — | **never** |
| D2 | INSERT_MARKER | — | no |
| D3 | GET_EXT_PARAMS | length byte + bytes [M]; 2 bytes observed [O] | yes, when the comment contains `Beta` |

### 3.1 Parameter block (GET_PARAMS, Format DH0.5) [M]
Three groups of six **signed 16-bit big-endian** values, one per joint *i* = 0..5:

| Bytes | Field | Scale |
|---|---|---|
| 0–11 | αᵢ — link twist | raw / 32768 × π rad |
| 12–23 | Aᵢ — link length | raw / 1000 inch |
| 24–35 | Dᵢ — link offset | raw / 1000 inch |

D₅ is the stylus length. Example decode from one model-DX arm [O] (mm, degrees):

```
 i   alpha(deg)     A(mm)      D(mm)
 0      0.0000     0.0000   210.8200
 1     89.8956    24.3586   -22.0472
 2     -0.0659   260.4262     0.1778
 3     90.1978    13.8176   234.4928
 4    -89.9451   -10.1092     8.1026
 5    -89.9066    10.1854  -133.9088
 beta = 0.03296 deg
```

Other parameter formats (e.g. "Format DH1.0") are **untested**; MicroScribe Inspector decodes any
36-byte block this way and reports the format string so you can check.

### 3.2 GET_MAXES (24 bytes) [M][O]
| Bytes | Meaning |
|---|---|
| 0 | supported buttons bitmask — `0x03` (two pedals) observed [O] |
| 1–2 | max timer, uint16 BE |
| 3–11 | analog channel maxima (not used here) |
| 12–23 | 6 × uint16 BE encoder maxima. **Counts per revolution = max + 1** [M]. Observed: 16384, 16384, 8192, 4096, 4096 and **0** for the 6th encoder of the 5-axis DX [O]. |

### 3.3 Extended parameters (GET_EXT_PARAMS) [M]
When the comment contains `Beta`: β = int16BE(ext[0], ext[1]) / 32768 × π — the small skew between
joints 2 and 3. Otherwise β = 0.

## 4. Data query [M]

The host sends one byte with bit 7 clear. `0x20` requests the timer; the low two bits select the
encoders — `0x01` = 5 encoders, `0x03` = 6, `0x02` = 7 [M]. This project sends `0x01` (or `0x03` when
a 6th encoder max is reported).

Reply [M]: header byte (`command | 0x80`), then

| Field | Bytes | Decoding |
|---|---|---|
| buttons | 1 | bit0 = pedal/button 1, bit1 = pedal/button 2 [M][O] |
| timer (if requested) | 2 | `(b0 << 7) | b1` |
| encoders | 2 each | `(hi << 7) | lo` (14-bit) |

Observed polling rate with 5 encoders at 9600 baud: ~62 replies/s [O].

Encoder counts are zero at the arm's **home pose** — the pose at power-up, or when HOME_POS was last
sent [M]. Power the arm on resting in its cradle.

## 5. Kinematics [M]

Joint angle: θᵢ = (encᵢ mod cprᵢ) × 2π / cprᵢ (missing joints = 0) [M].

Link transform (modified Denavit–Hartenberg) for every joint except *i* = 2, with
c = cos θᵢ, s = sin θᵢ, cα = cos αᵢ, sα = sin αᵢ [M]:

```
| c        -s         0      Aᵢ      |
| s·cα      c·cα     -sα    -sα·Dᵢ   |
| s·sα      c·sα      cα     cα·Dᵢ   |
| 0         0         0      1       |
```

Joint 2 includes β (cb = cos β, sb = sin β) [M]:

```
| c·cb               -s·cb               sb       sb·D₂ + A₂ |
| s·cα + sα·sb·c      c·cα − sα·sb·s    −sα·cb   −sα·cb·D₂   |
| s·sα − cα·sb·c      c·sα + s·sb·cα     cα·cb    cα·cb·D₂   |
| 0                   0                  0        1          |
```

T = M₀·M₁·M₂·M₃·M₄·M₅. The stylus tip is the translation column of T, in the base frame [M]. The
stylus direction is the third column of the rotation part. A custom tip is modelled as an extra
translation in the stylus frame (found by pivot calibration in the app).

Consistency check on real hardware [O]: rotating only the base joint moved the tip around the base
axis while its radius (75.21 mm) and height (212.090 mm) stayed unchanged.

## 6. Commands this project never sends
`SET_PARAMS (C7)`, `SET_HOME_REF (D0)`, `RESTORE_FACTORY (D1)`, `SET_HOME (C3)` — they change the
arm's stored calibration. `HOME_POS (C2)` is sent only when the user explicitly re-homes with the
arm in its cradle.
