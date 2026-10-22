# ADMT4000 Zephyr IIO — no-OS Parity Plan

Status: **proposed, not implemented**
Target: make `iio_info` against the Zephyr IIOD server match the no-OS
`iio_trigger_example` output as closely as the Zephyr libiio backend allows.

Reference output: no-OS `iio_info -u serial:COM22,230400,8n1`, captured
2026-10-22 (4 devices: `admt_evb`, `admt4000`, `tmc5240`, `trigger0`).

---

## 1. Scope

| Tier | Content | Touches |
|------|---------|---------|
| 1 | Channel ids, scan formats, raw register values, hardware scales, temp offset, attribute filenames | `iio_admt4000.c`, `admt4000.c`, `admt4000.h` |
| 2 | `faults`, `gpio0_raw`…`gpio5_raw`, `reinitialize`, debug `direct_reg_access` + `reset` | `iio_admt4000.c`, `admt4000.c`, `admt4000.h` |
| 3 | `admt_evb` as a second IIO device (`power_enable`, `pulse_shutdown`, `coil_reset`) | new driver + binding + overlay |

### Explicitly out of scope

| Item | Why |
|------|-----|
| `Current trigger: trigger0(admt4000-dev0)` | `zephyr_create_context()` never registers a trigger device and `zephyr_get_trigger()` in [backend.c](../../../../modules/lib/libiio/zephyr/backend.c) returns `NULL` unconditionally. Needs an upstream libiio-Zephyr module change. |
| Working buffer / scan elements | Backend `iio_backend_ops` implements only `create`, `read_attr`, `write_attr`, `get_trigger`. No `open_buffer`/`readbuf`, so `iio_device_get_buffer()` returns `NULL` and IIOD answers `-ENODEV`. |
| `tmc5240` device | Separate motor-controller IC, not present on this setup. A Zephyr `adi_tmc` driver exists but wiring it is a different project. |

Both unsupported items should get an explicit note in the driver README so the
difference is documented rather than looking like a defect.

---

## 2. Current vs target

Measured Zephyr values (SDP-K1, 2026-10-22) against the no-OS reference:

| Channel | no-OS id | no-OS format | no-OS raw | no-OS scale | Zephyr id | Zephyr format | Zephyr raw |
|---|---|---|---|---|---|---|---|
| turns | `rot` | `le:s9/16>>0` | `99` | `0.250000000` | `rot0` | `le:S32/32>>0` | `24250000` |
| angle | `angl` | `le:u12/16>>0` | `3323` | `0.087890625` | `angl0` | `le:S32/32>>0` | `97294921` |
| temp | `temp` | `le:u12/16>>0` | `1914` | `63.856960408` (offset `1168`) | `temp0` | `le:S32/32>>0` | `37009800` (offset `0`) |
| cosine | `count1` | `le:s14/16>>0` | `-2663` | — | `count1` | `le:S32/32>>0` | — |
| sine | `count2` | `le:s14/16>>0` | `-2471` | — | `count2` | `le:S32/32>>0` | — |
| radius | `voltage` | `le:u15/16>>0` | `12065` | `0.000924000` | `voltage0` | `le:S32/32>>0` | — |

Device attributes: no-OS **17**, Zephyr **12**.

- Zephyr missing: `faults`, `gpio0_raw`…`gpio5_raw`, `reinitialize`
- Zephyr extra: `eck_type`, `eck_type_available`, `coil_reset`
  (no-OS puts `coil_reset` on the `admt_evb` device instead)

Debug attributes: no-OS **2** (`direct_reg_access`, `reset`), Zephyr **0**.

---

## 3. Tier 1 — channel section

### 3.1 Channel ids and indexing

no-OS leaves `rot`/`angl`/`temp`/`voltage` non-indexed and marks only
cosine/sine `indexed = 1` with `.channel = 1` / `.channel = 2`. libiio renders
that as `rot`, `angl`, `temp`, `count1`, `count2`, `voltage`.

Zephyr's [iio_admt4000.c](iio_admt4000.c) hardcodes the id string, so this is a
table edit in `admt4000_iio_channels[]`:

```
"rot0"     -> "rot"
"angl0"    -> "angl"
"temp0"    -> "temp"
"count1"   -> "count1"   (unchanged)
"count2"   -> "count2"   (unchanged)
"voltage0" -> "voltage"
```

The `index` argument to `iio_device_add_channel()` stays 0..5 — that is the
scan index, which no-OS also keeps sequential.

**Risk:** this renames the attribute path clients use. Any existing script
calling `iio_attr -c admt4000 angl0 raw` must become `angl`.

### 3.2 Scan formats

`iio_device_add_channel()` takes a `struct iio_data_format`. Replace the single
shared 32-bit format with a per-channel one:

| Channel | `.length` | `.bits` | `.is_signed` |
|---|---|---|---|
| `rot` | 16 | 9 | true |
| `angl` | 16 | 12 | false |
| `temp` | 16 | 12 | false |
| `count1`/`count2` | 16 | 14 | true |
| `voltage` | 16 | 15 | false |

`.is_be = false`, `.shift = 0` throughout — matches no-OS `le:…>>0`.

Add `has_scale`/`has_offset` per channel as today, plus the format fields, to
the `admt4000_iio_channel` struct.

### 3.3 Raw values — the breaking change

**Approved**: `raw` switches from SI micro-units to hardware register counts.

The blocker is that every raw getter in
[admt4000.c](admt4000.c) is `static`:

| Function | Line | Returns |
|---|---|---|
| `admt4000_get_raw_turns_and_angle` | 457 | `uint8_t turns`, `uint16_t angle[]` |
| `admt4000_get_cos` | 502 | `int16_t` |
| `admt4000_get_sin` | 527 | `int16_t` |
| `admt4000_get_radius` | 557 | `uint16_t` |
| `admt4000_get_temp` | 596 | `uint16_t` |

The public header [admt4000.h](../../../../include/zephyr/drivers/sensor/admt4000.h)
exports only `admt4000_get_decoder()`.

**Chosen approach — new raw sensor attributes.** Keeps the driver boundary
intact and follows Zephyr convention (the IIO wrapper already goes through
`sensor_attr_get`). Add to `enum sensor_attribute_admt4000`:

```c
SENSOR_ATTR_ADMT4000_RAW_TURNS,     /* quarter-turn count, signed  */
SENSOR_ATTR_ADMT4000_RAW_ANGLE,     /* 12-bit, FIELD_GET(ANGLE_MASK) */
SENSOR_ATTR_ADMT4000_RAW_TEMP,      /* 12-bit ADC code             */
SENSOR_ATTR_ADMT4000_RAW_COS,       /* 14-bit signed CORDIC        */
SENSOR_ATTR_ADMT4000_RAW_SIN,       /* 14-bit signed CORDIC        */
SENSOR_ATTR_ADMT4000_RAW_RADIUS,    /* 15-bit                      */
```

These read from the already-cached `struct admt4000_data` fields
(`quarter_turns`, `angle[1]`, `temp`, `cos_val`, `sin_val`) that
`admt4000_sample_fetch_helper()` populates — no extra SPI traffic.

> **Note on `rot`:** no-OS raw `99` with scale `0.25` is the *quarter-turn*
> count, i.e. 24.75 turns. Zephyr's `data->quarter_turns` is exactly this, so
> the mapping is direct.

**Open issue:** the wrapper currently fetches on read of channel index 0 only.
That interacts badly with the stale-cache bug (§6) and must be resolved in the
same change.

### 3.4 Scales and offset

| Channel | Scale string | Derivation |
|---|---|---|
| `rot` | `0.250000000` | quarter turn |
| `angl` | `0.087890625` | 360 / 4096 |
| `temp` | `63.856960408` | 1e6 / 15660 (m°C per LSB) |
| `voltage` | `0.000924000` | mV/V per LSB |
| `count1`/`count2` | none | raw CORDIC counts |

**Discrepancy found — needs a decision.** no-OS reports `temp` offset as
`1168`, but its own conversion in `admt4000.c:1105` is:

```c
*temp = ((temp_raw + ADMT4000_TEMP_OFFSET) * MILLI / ADMT4000_TEMP_SCALE_MILLIS);
```

with `ADMT4000_TEMP_OFFSET = -1168` (admt4000.h:206). The IIO convention is
`processed = (raw + offset) * scale`, so the offset consistent with the
conversion is **-1168**, not `+1168`. no-OS emits `val = -ADMT4000_TEMP_OFFSET`,
which flips the sign.

Check: raw `1914`
- with `-1168`: `(1914 - 1168) * 63.857 = 47637` m°C = **47.6 °C** — plausible
- with `+1168`: `(1914 + 1168) * 63.857 = 196805` m°C = **196.8 °C** — not plausible

Options: (a) emit `-1168` so `processed = (raw + offset) * scale` is correct and
deviate from no-OS by a sign, or (b) replicate `+1168` for byte-exact parity and
document it. **Recommend (a)** and raise the no-OS sign as a separate bug.

### 3.5 Attribute filenames

no-OS shows `attr 0: raw (in_rot_raw) value: 99`. The parenthesised string is
the attribute *filename*. `iio_channel_add_attr()` takes it as its 4th argument;
Zephyr currently passes `NULL`.

Pass `in_<id>_<attr>` — `in_rot_raw`, `in_rot_scale`, `in_angl_raw`,
`in_temp_offset`, `in_count1_raw`, `in_voltage_scale`, etc. These need to be
static strings with lifetime beyond the call; a per-channel table of
precomputed names is simplest.

---

## 4. Tier 2 — missing attributes

### 4.1 `faults`

no-OS reads the fault register and prints a bitmask. Zephyr has the register
definitions already — `ADMT4000_AGP_REG_FAULT` (0x06), `ADMT4000_ALL_FAULTS`,
`ADMT4000_AGP_INDIV_FAULT(x)`, and `enum` members `ADMT4000_FAULT_VDD_UV`,
`ADMT4000_FAULT_VDD_OV`, `ADMT4000_FAULT_VDRIVE_UV`, `ADMT4000_FAULT_VDRIVE_OV`
— in [admt4000.h](../../../../include/zephyr/drivers/sensor/admt4000.h).

Add `SENSOR_ATTR_ADMT4000_FAULTS` (read-only) returning the raw fault word.

### 4.2 `gpio0_raw` … `gpio5_raw`

Read/write the ADMT4000's own GPIO pins via its registers — **not** MCU pins.
The driver already manipulates these through `admt4000_io_en()` (line 874) and
`admt4000_gpio_func()` (line 904), and tracks `data->gpios[]` with
`is_alt_pin`. `ADMT4000_MAX_GPIO_INDEX` is 5 and `ADMT4000_GPIO_LOGIC(x)` /
`ADMT4000_GPIO_FUNC(x)` macros exist.

Add a single indexed attribute pair rather than six:
`SENSOR_ATTR_ADMT4000_GPIO_RAW` with the GPIO number carried in
`sensor_value.val2`, then expose six IIO attribute names that map onto it.

> Sanity check against the reference output: `gpio3_raw value: 1` — GPIO3 is
> ACALC, which the Zephyr driver already configures to alt-function at init
> (`admt4000_gpio_func(dev, 3, true)`, line 1654) when `acalc-gpios` is present.

### 4.3 `reinitialize`

Write-only; re-runs the device init sequence. Maps to a new
`SENSOR_ATTR_ADMT4000_REINITIALIZE` that calls the existing init path.
Reads must return `-ENOENT` to match no-OS (`ERROR: No such file or directory (-2)`).

### 4.4 Debug attributes

**This is supported** — `IIO_ATTR_TYPE_DEBUG` exists in
[iio-backend.h](../../../../modules/lib/libiio/include/iio/iio-backend.h) (line 71),
`iio_device_add_attr()` accepts it, and the IIOD responder's `get_attr()`
already dispatches `IIOD_OP_READ_DBG_ATTR` to `iio_device_get_debug_attr()`.
Nothing in the Zephyr backend blocks it; the driver simply never registers any.

Work:
1. Register `direct_reg_access` and `reset` with `IIO_ATTR_TYPE_DEBUG`.
2. Handle `IIO_ATTR_TYPE_DEBUG` in `admt4000_iio_read_attr()` /
   `admt4000_iio_write_attr()` — currently falls through to `-EINVAL`.
3. Add `SENSOR_ATTR_ADMT4000_REG_ADDR` + `SENSOR_ATTR_ADMT4000_REG_VAL` backed
   by the existing static `admt4000_read()` (line 150) / `admt4000_write()`
   (line 202).

`direct_reg_access` semantics: write `<addr>` then read to get the value, or
write `<addr> <value>` to set. Mirrors the no-OS/IIO convention.

---

## 5. Tier 3 — `admt_evb` device

no-OS exposes a second IIO device with three boolean attributes
(from `projects/admt4000/src/common/iio_admt_evb.c`):

| Attribute | no-OS enum | Pin | SDP-K1 |
|---|---|---|---|
| `pulse_shutdown` | `ADMT_EVB_SHDN` | A1 | `gpioa 4` |
| `power_enable` | `ADMT_EVB_V_EN` | A3 | `gpioc 1` |
| `coil_reset` | `ADMT_COIL_RS` | D4 | `gpiog 9` |

Zephyr currently drives `V_EN` and `SHDN_N` as **`gpio-hog`s** in the overlay
(set high at `CONFIG_GPIO_HOGS_INIT_PRIORITY` 41) and `COIL_RS` via the sensor
driver's `coil-rs-gpios`.

Work:
1. New binding `adi,admt4000-evb.yaml` with `v-en-gpios`, `shdn-n-gpios`,
   `coil-rs-gpios`.
2. New `iio_admt_evb.c` implementing the `iio_device` API with three
   `IIO_ATTR_TYPE_DEVICE` boolean attributes.
3. **Remove the `gpio-hog` entries** from
   [admt4000.overlay](../../../../modules/lib/libiio/zephyr/samples/iiod/admt4000.overlay)
   and let the new driver own those pins.
4. Device ordering: the EVB driver must init **before** the ADMT4000 sensor,
   or the rails are off during sensor probe and every register read returns
   `0x00` (this exact failure cost a full session on 2026-10-21 — see the
   README troubleshooting section). Set its init priority below
   `SENSOR_INIT_PRIORITY` (90).

**Risk:** this is the highest-risk tier. It moves the power rails from a
mechanism that is known-working into a new driver with an ordering dependency.
Recommend implementing and verifying Tiers 1–2 first, then doing Tier 3 as a
separate flash-and-verify cycle with the rail state confirmed on hardware
before trusting any sensor reading.

Also note: `coil_reset` would then exist on **both** devices (the sensor driver
already has `SENSOR_ATTR_ADMT4000_COIL_RESET`). Decide whether to drop it from
the `admt4000` device for strict parity or leave both.

---

## 6. Prerequisite bug — stale sample cache

Independent of parity, and confirmed on hardware 2026-10-22:

```
temp raw #1,2,3 -> 36887250, 36887250, 36887250   (frozen)
rot raw         -> 24250000                        (invalidates cache)
temp raw #4     -> 37132350                        (moved)
```

`admt4000_iio_read_channel_raw()` only clears `fetched` when `index == 0`, so a
client polling a single channel gets one reading forever, with no error. Tier 1
rewrites this function anyway — fold in a **time-bounded cache** (re-fetch if
older than ~100 ms) so intra-round coherence is kept without the ordering
dependency.

---

## 7. File-by-file summary

| File | Tier | Change |
|---|---|---|
| `include/zephyr/drivers/sensor/admt4000.h` | 1,2 | +6 raw attrs, +faults, +gpio_raw, +reinitialize, +reg addr/val |
| `drivers/sensor/adi/admt4000/admt4000.c` | 1,2 | `attr_get`/`attr_set` cases for the above |
| `drivers/sensor/adi/admt4000/iio_admt4000.c` | 1,2 | ids, per-channel formats, raw via new attrs, scales/offset, filenames, debug attr handling, cache fix |
| `drivers/sensor/adi/admt4000/iio_admt_evb.c` | 3 | **new** |
| `dts/bindings/sensor/adi,admt4000-evb.yaml` | 3 | **new** |
| `modules/lib/libiio/zephyr/samples/iiod/admt4000.overlay` | 3 | drop hogs, add evb node |
| `drivers/sensor/adi/admt4000/Kconfig` | 3 | `CONFIG_ADMT4000_EVB_IIO` |
| `drivers/sensor/adi/admt4000/README.rst` | all | rewrite channel/attr tables; document raw-units change; note trigger+buffer unsupported |

---

## 8. Verification

Per tier, on SDP-K1 at `serial:COM22,230400,8n1n`:

1. `iio_info` — diff the channel/attribute listing against the no-OS reference
   capture. Expect exact match except the documented trigger/buffer lines and
   the temp-offset sign decision (§3.4).
2. `iio_attr -c admt4000 angl raw` → expect a 0–4095 integer, **not** micro-degrees.
3. Cross-check `raw * scale` against the pre-change SI value:
   `3323 * 0.087890625 = 292.06°`.
4. Poll a varying channel (`temp`) repeatedly to confirm the stale-cache fix —
   use temperature, **not** a stationary angle, which cannot detect the failure.
5. Tier 3 only: confirm rails high and sensor probing cleanly (`res=0`) before
   trusting data.

---

## 9. Open decisions

1. **Temp offset sign** (§3.4) — correct `-1168` vs no-OS-identical `+1168`.
2. **`coil_reset` placement** (§5) — sensor device, EVB device, or both.
3. **`eck_type` / `eck_type_available`** — Zephyr has these, no-OS does not.
   Keep (superset) or drop for strict parity? Recommend keep.
4. **Channel rename blast radius** — `angl0` → `angl` breaks existing scripts
   and the current README examples.
