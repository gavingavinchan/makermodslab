# Star leader: power-capped position commands

Research only, 2026-10-20 America/Chicago (bench timestamp 2026-10-21 UTC).
Goal A2 in [viscous-haptics.md](viscous-haptics.md): a bounded pull toward the
follower while a pilot holds the leader, supporting input-gated bilateral control.
No motion or servo-state/configuration writes were performed. Only three
`sync_monitor([0,1,2,3,4,5,6])` calls reached the bus. No commits or pushes.

## Finding

FashionStar documents a per-command **power limit in mW** for position control.
Installed motorbridge 0.0.4 always encodes **zero**, meaning use the servo's
configured power-protection ceiling, not zero effort. This supports a protocol
route to A2, but does not establish a calibrated torque limit or verify this
leader's firmware behavior under load.

## Evidence and provenance

- Linux venv metadata: `motorbridge-smart-servo==0.0.4`, under
  `/home/gavin/Documents/makermodslab-viscous/.venv/lib/python3.12/site-packages/`.
  Installed `motorbridge_smart_servo/fashionstar.py:120-139` forwards four angle
  arguments; `:153-157` forwards the stop-mode power argument.
- Exact [PyPI release](https://pypi.org/project/motorbridge-smart-servo/0.0.4/),
  [source archive](https://files.pythonhosted.org/packages/e6/56/45af87189dc49abbe46157b792b7c71f502a5f819f04e7485de0cfa52d9b/motorbridge_smart_servo-0.0.4.tar.gz).
  SHA256 `fb65f3f6e765e6b1915071c255caaf112fad3796fa1761aeee0132d15b8a0989`.
  Downloaded directly from PyPI's JSON-listed URL and extracted as data in fresh
  `/tmp/star-servo-research.xboQX0/`; no build, import, or execution of downloaded code.
  All motorbridge file:line references below are relative to that sdist root.
- `README.md:6-8` explicitly labels angle write/control commands unsupported in
  this release line; the implementation nevertheless sends them.
- Manufacturer [UART/RS-485 protocol, Chinese](https://fashionstar.com.cn/wiki/protocols/servo/uart-rs485/)
  and [English](https://fashionstar.com.hk/wiki/protocols/servo/uart-rs485/),
  angle-control sections and configuration appendices.
- Manufacturer SDK pinned to
  [`f72877669af0a4dc3501bb98df291cacc0a8592e`](https://github.com/servodevelop/servo-uart-rs485-sdk/tree/f72877669af0a4dc3501bb98df291cacc0a8592e):
  [`micro-python/src/uservo.py`](https://github.com/servodevelop/servo-uart-rs485-sdk/blob/f72877669af0a4dc3501bb98df291cacc0a8592e/micro-python/src/uservo.py),
  lines 403-491 (angle commands and power-limit documentation), 653-658
  (monitor decoding and temperature conversion), 673-689 (stop modes).
  This downloaded SDK was only read, never imported or executed.

## What motorbridge sends

Trace: `src/motorbridge_smart_servo/fashionstar.py:120-139` →
`smart_servo_py/src/lib.rs:238-256` →
`smart_servo_vendors/fashionstar/src/controller.rs:241-250` →
`smart_servo_vendors/fashionstar/src/protocol.rs:122-187`.

| `multi_turn` | `interval_ms` | Command | Payload bytes |
|---|---|---|---:|
| false | 0 | `0x08`, basic single-turn | 7 |
| false | positive | `0x0B`, single-turn with acceleration/deceleration time | 11 |
| true | 0 | `0x0D`, basic multi-turn | 11 |
| true | positive | `0x0E`, multi-turn with acceleration/deceleration time | 15 |

Every branch ends with `power:u16 = 0`. Positive intervals also hard-code
`accel=20 ms`, `decel=20 ms`. Angles are rounded to 0.1°; single-turn targets
are clamped to ±180°. There is no power argument, raw-send method, or velocity
variant exposed by this class. `set_angle` returns after writing, without
waiting for a servo acknowledgement.

`makermodslab/star_follow.py:156-168` selects `0x0E` on ordinary positive-duration
ticks. Speed/travel limits do not create an effort cap. A further timing concern:
the encoder permits intervals shorter than its fixed 20+20 ms ramps; the vendor
SDK (`uservo.py:455-459`) raises multi-turn duration to at least their sum.
Do not assume sensible 100 Hz interpolation without characterization.

## Packet layouts for a future implementation

These are specifications only; no helper was implemented or packet sent.
All multibyte values are little-endian. Angle is signed tenths of a degree;
time/ramps are unsigned milliseconds; speed is unsigned tenths of a degree/s;
power is unsigned mW. `B=uint8`, `h=int16`, `H=uint16`, `i=int32`, `I=uint32`.

Request frame: `12 4C command payload_length payload checksum`.
Checksum is the sum of **all preceding bytes**, including header, modulo 256;
neither CRC nor complemented sum. Response header is `05 1C`, with its own sum.
Evidence: motorbridge `protocol.rs:39-60`; vendor `uservo.py:24-58`.

| Command | Payload, in order | Python struct format | Length |
|---|---|---|---:|
| `0x08` basic single-turn | id, angle, time, power | `<BhHH` | 7 |
| `0x0B` single-turn with interval | id, angle, time, accel, decel, power | `<BhHHHH` | 11 |
| `0x0C` single-turn with velocity | id, angle, speed, accel, decel, power | `<BhHHHH` | 11 |
| `0x0D` basic multi-turn | id, angle, time, power | `<BiIH` | 11 |
| `0x0E` multi-turn with interval | id, angle, time, accel, decel, power | `<BiIHHH` | 15 |
| `0x0F` multi-turn with velocity | id, angle, speed, accel, decel, power | `<BiHHHH` | 13 |
| `0x18` stop | id, mode, power | `<BBH` | 4 |
| `0x09` damping | id, power | `<BH` | 3 |

Use individual IDs 0-6, not broadcast. A basic `0x0D` with explicit positive
duration and power is the simplest multi-turn candidate when the host already
slews targets. `0x0E` needs a coherent motion profile. The manufacturer wiki
has an inconsistent `0x0F` speed row (four bytes alongside a 13-byte payload).
The formats above follow both vendor MicroPython `uservo.py:475` and STM32
[`fashion_star_uart_servo.c:514-548`](https://github.com/servodevelop/servo-uart-rs485-sdk/blob/f72877669af0a4dc3501bb98df291cacc0a8592e/STM32F103/STM32F103_SDK/User/fashion_star_uart_servo/fashion_star_uart_servo.c#L514),
which encode **two-byte speed**. Do not copy the inconsistent row literally.

The vendor SDK calls `power` a power limit (`uservo.py:419-420`). A positive
value below the configured protection ceiling limits execution power; zero or
a value above the ceiling selects the ceiling. This is not a signed torque
command, mechanical shaft power specification, or documented bound on transient
hand force. Whether the requested cap persists unchanged after reaching the
target, its bandwidth/overshoot, and its relation to stall torque remain untested.
**A zero-effort gate must not encode zero in this field.**

Stop mode `0x11` stops and holds the current position, with holding execution
power specified in mW. Mode `0x12` stops and enters adjustable damping: external
force can move it, with larger power giving greater resistance. Power is not a
documented viscous coefficient in N·m·s/rad. Neither has a target-angle field.
Thus `0x11 + power` cannot move the holding point toward the follower by itself;
interleaving ordinary uncapped angle writes would reintroduce their zero-power
ceiling. See protocol stop section and [damping API](https://fashionstar.com.hk/wiki/pan-tilt/servo-damping-mode/).

## Configuration, telemetry, and failure behavior

Documented configuration data IDs (decimal): stall protection switch 37;
stall fallback power 38 (mW); voltage limits 39/40 (mV); temperature threshold
41 (ADC in protocol appendix); power protection 42 (mW); current protection
43 (mA). Also power-on lock 46 and angle-limit enable/bounds 48/51/52.
These settings were neither read nor written: the authorized bus allowlist did
not include configuration reads. Actual thresholds and servo models/firmware
remain unknown. An older SDK header calls ID 41 Celsius, conflicting with the
protocol appendix; verify against exact firmware before any future setting.

The manufacturer [configuration guide](https://fashionstar.com.cn/wiki/software/servo/pc-config-software/)
and [protection guide](https://fashionstar.com.hk/wiki/documents/servo/setting-protection-parameters/)
describe power reduction versus unlocking on overload, according to the stall
switch. A [UART model's protection table](https://fashionstar.com.cn/wiki/uart-servo/rp8-u45h-m/)
also describes temperature derating, over-current release with automatic
recovery, and voltage faults requiring a power cycle. These are family examples,
not a verified description of this leader's unidentified servo firmware.

Monitor response `0x16`: 16-byte payload `<BHHHHBih` = id, voltage mV, current
mA, power mW, temperature ADC, status, signed angle in 0.1°, signed turn count.
Motorbridge `protocol.rs:215-244` agrees (and tolerates older 14-byte payloads
without turns). The wiki's unsigned angle type conflicts with its signed
position semantics; both SDKs decode signed. Current/power are unsigned
magnitudes: **not signed joint torque or contact direction**.
They are operating telemetry, intended to report energized consumption/load,
not the commanded cap; energized load response was not tested here. See
[monitor field documentation](https://fashionstar.com.hk/wiki/documents/servo/data-monitoring/).

Read-only sync request used by motorbridge: command `0x19`, payload
`16 01 07 00 01 02 03 04 05 06`; individual replies use `0x16`.
`protocol.rs:250-262` constructs it. `controller.rs:140-164` may return cached
telemetry on misses, and also marks samples unreliable when its angle filter
rejects near-zero readings. `reliable=False` alone does not prove a bus miss.

No communication-loss torque-release watchdog is documented in the protocol
or configuration list examined. Motorbridge loss detection only raises a host
error; closing the connection drops the port (`smart_servo_py/src/lib.rs:179-181`)
without releasing torque. Assume the last target/hold can persist through host
or bus loss until hardware behavior is characterized. A release command cannot
help once the bus is broken. Moreover, `controller.rs:194-197` only writes stop
packets: the claim in `star_follow.py:172` that successful release means every
servo acknowledged is not supported by this library.

## Read-only Linux bench

Immediately before opening the port, HTTP session check returned
`{"session":null,"last_ended":null}`, and `fuser` found no port owner.
Fresh installed `FashionStarServo`, 1,000,000 baud, specified by-id port;
three polls spaced 0.5 s, then `close()` in `finally`; `is_open` became false.
Final `fuser` also found no owner. Leader left untouched in its previously
released state; no command was sent to establish or verify release mode.

Poll times UTC: `2026-10-21T00:36:34.510590`, `00:36:35.015295`,
`00:36:35.520458`. Table gives ranges over the three returned samples.

| ID | angle ° | current mA | power mW | voltage mV | temp ADC | turn | reliable |
|---|---:|---:|---:|---:|---:|---:|---|
| 0 | -11.8 | 30 | 369 | 12307-12312 | 1906 | -1 | T,T,T |
| 1 | 0.0 | 30 | 366 | 12196 | 1960 | -1 | T,F,F |
| 2 | 1.5 | 30 | 362 | 12070-12075 | 1991-1992 | 0 | T,T,T |
| 3 | -2.4 | 30 | 365 | 12171-12176 | 1958 | -1 | T,T,T |
| 4 | 6.9-7.0 | 30 | 361 | 12036-12041 | 1947 | 0 | T,T,T |
| 5 | -1.4 | 30 | 361 | 12051-12055 | 1955 | -1 | T,T,T |
| 6 | -0.3 | 30 | 365 | 12186-12191 | 1934 | -1 | T,T,T |

Every returned status was zero. Nonzero idle current/power is present without
commanding holding torque: power numerically matches V×I to rounding, suggesting
supply-side consumption including electronics. It cannot by itself establish
that the servo is exerting torque. ID 1's later values carry the unreliable flag;
the near-zero filter (`smart_servo_core/src/reliability.rs:34-61`) explains a
possible cause; a cache hit cannot be excluded from this API's output alone.

Using the manufacturer's NTC formula (`uservo.py:658`), independently transcribed
into standard-library arithmetic, ADC converts as
`T_C = 1 / (ln(ADC/(4096-ADC))/3435 + 1/298.15) - 273.15`.
Here that is approximately 26.4-28.6 °C; this is a derived estimate, not a
calibrated temperature measurement. The ADC decreases as temperature increases.

## Recommendation for A2

Prefer a small **FashionStar-specific motorbridge patch** exposing explicit
`power_mw` through wrapper, PyO3, controller and encoder, using its existing
single-owner mutex/transport. Keep legacy defaults distinguishable from a capped
API that rejects zero, requires finite bounded targets/durations, and makes
zero gate an explicit release operation. This prevents concurrent native/raw
owners and avoids changing a vendor-neutral API for unrelated servo brands.
Pin the reviewed fork/release and check serialized bytes offline before a
separately authorized powered characterization.

A standalone pyserial helper is the smallest isolated bench prototype, using
the layouts above, 1 Mbaud 8N1 and one exclusive port owner. It must own reads
and writes, parse/check replies, and must never run alongside the native owner.
It is not an existing motorbridge capability. Stop-mode emulation is unsuitable
because it cannot set the target or guarantee a cap on a subsequent move.

Against a hand, even capped servos can heat during sustained stall, pinch at
mechanical stops, or suddenly yield/recover when protection intervenes. The
power-to-torque relationship, minimum usable cap above idle consumption,
post-arrival holding, transient force, thermal behavior, and bus-loss persistence
are unresolved hardware questions. The [Kanai paper, III and IV-B](https://arxiv.org/html/2509.08226v1)
limits current/torque/PWM input, with current-limited Dynamixel experiments;
using a FashionStar mW field is an adaptation requiring characterization, not
an already validated equivalent.
