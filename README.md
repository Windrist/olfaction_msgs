# olfaction_msgs

Reference documentation for the `olfaction_msgs` ROS 2 package — standard
message definitions for robotic olfaction sensors (gas detectors, anemometers,
spectroscopic sensors).

- **Build type**: `ament_cmake` (IDL only, no nodes) · **License**: GPL-3.0
- **Upstream**: [MAPIRlab/olfaction_msgs](https://github.com/MAPIRlab/olfaction_msgs) · **This fork**: [Windrist/olfaction_msgs](https://github.com/Windrist/olfaction_msgs), branch `ros2`
- **Generates**: 4 messages — `Anemometer`, `GasSensor`, `GasSensorArray`, `TDLAS`
- Vendored as a **git submodule** into the GSExploration workspace

> **This is a submodule.** Commits here go to the fork, not to the
> GSExploration repository. See [§8](#8-fork-status-and-submodule-workflow)
> before editing.

---

## Contents

- [1. Executive Summary](#1-executive-summary)
- [2. Usage in This Workspace](#2-usage-in-this-workspace)
- [3. Anemometer — The Wind Contract](#3-anemometer--the-wind-contract)
- [4. The Gas Messages](#4-the-gas-messages)
- [5. Consumers and QoS](#5-consumers-and-qos)
- [6. Data Sources](#6-data-sources)
- [7. Building](#7-building)
- [8. Fork Status and Submodule Workflow](#8-fork-status-and-submodule-workflow)
- [9. Known Issues](#9-known-issues)
- [10. References](#10-references)

---

## 1. Executive Summary

This package defines four sensor message types. **GSExploration uses exactly
one of them.**

| Message | Fields | Used in GSExploration |
|---|---:|---|
| [`Anemometer.msg`](msg/Anemometer.msg) | 4 | **Yes** — 7 subscribers across 6 packages |
| [`GasSensor.msg`](msg/GasSensor.msg) | 8 + 33 constants | No — zero references |
| [`GasSensorArray.msg`](msg/GasSensorArray.msg) | 2 | No — zero references |
| [`TDLAS.msg`](msg/TDLAS.msg) | 7 | No — zero references |

The three gas messages are inherited from upstream and retained for
compatibility with the wider MAPIRlab olfaction ecosystem. They are **not**
part of any GSExploration data path — gas concentration travels as
`geometry_msgs/Vector3Stamped` on `/gas_data` instead ([§4.4](#44-why-gsexploration-does-not-use-the-gas-messages)).

For a GSExploration maintainer, this package is effectively **one message**:
`Anemometer`, and specifically its `wind_direction` field, whose convention is
the single most error-prone semantic in the workspace ([§3](#3-anemometer--the-wind-contract)).

---

## 2. Usage in This Workspace

```
                    ┌──────────────────────────────┐
   simulated_       │  Anemometer on /wind_data    │
   anemometer  ────→│  (external: Libraries_ws)    │
   (GADEN)          └──────────────┬───────────────┘
                                   │
   fake_sensor_node ───────────────┤   main_decision_debug/src/fake_sensor_node.cpp:227
   (replay/debug)                  │
                                   │
        ┌──────────────────────────┼──────────────────────────┐
        ▼                          ▼                          ▼
   sampling_node            gsl_local_search           gsl_streamline
   sampling_node.cpp:227    algorithm_base.cpp:39      :355
        │                   particle_filter_             │
        │                     standalone_node.cpp:54     │
        ▼                          ▼                     ▼
   Observation.wind        flow dir → map frame     flow dir → map frame
   (map frame u,v)

        ┌──────────────────┬──────────────────┬──────────────────┐
        ▼                  ▼                  ▼
   gdm_node           gmrf_wind_mapping   sensor_visualizer
   gdm_node.cpp:61    gmrf_node.cpp:53    sensor_visualizer.py:49
   wind-weighted      GMRF wind field     RViz arrow
   DM+V/W kernels     reconstruction
   [use_wind=false —
    off by default]
```

Seven subscribers, one message type, one topic (`/wind_data` — most
subscribers parameterize it; two default to `/anemometer` and depend on a
launch override).

**Three of the four `Anemometer` fields are read by live nodes**: `header`,
`wind_speed`, `wind_direction`. `sensor_label` is read only by the offline bag
validator — which still *requires* it
([§9.1](#91-sensor_label-is-read-only-offline--but-it-is-validated)).

---

## 3. Anemometer — The Wind Contract

```
std_msgs/Header header      # timestamp, frame_id
string sensor_label         # identifier
float32 wind_speed          # m/s
float32 wind_direction      # rad; clockwise UPWIND bearing in sensor frame
                            # (0 = North, pi/2 = East)
```

### 3.1 The convention

`wind_direction` is the single most misread field in the workspace. Three
properties must hold together, and getting any one wrong flips the wind by 180°
or mirrors it:

| Property | Value | Consequence if assumed otherwise |
|---|---|---|
| **Sense** | **UPWIND** — where wind comes FROM | Robot searches *downwind* of the source |
| **Handedness** | **Clockwise** (compass bearing) | Wind mirrored about the N–S axis |
| **Zero** | **North**, `π/2` = East | 90° rotation |
| **Frame** | **Sensor frame** (`header.frame_id`) | Wrong under any robot rotation |

This is a meteorological convention, not a mathematical one. ROS yaw is
counter-clockwise from +X (East); this field is clockwise from North. The two
differ by both a reflection and a rotation.

### 3.2 The conversion

Every consumer converts to a mathematical yaw before transforming to the map
frame. The full upwind-bearing → map-frame-flow conversion is:

```
flow_direction = 1.5π − wind_direction        (then TF into map frame)
```

Derivation: `0.5π − bearing` converts a clockwise-from-North bearing to a
counter-clockwise-from-East yaw (still upwind); adding `π` reverses it to flow.
`0.5π + π = 1.5π`.

Two implementations, both correct, applying the identity differently:

| Site | Expression | Note |
|---|---|---|
| `gsl_local_search/src/algorithm_base.cpp:466-468` | `1.5π − upwind` in one step | Canonical |
| `gsl_streamline/src/gsl_streamline_server.cpp:544-545` | `1.5π − upwind` in one step | Mirrors the above verbatim |
| `gas_distribution_mapping/src/gdm_node.cpp:422` | `0.5π − wind_direction`, TF, then flow | Split across steps |
| `GMRF-wind/.../gmrf_node.cpp:199` + `:203` | `0.5π − wind_direction`, TF, then `+ π` | Split across steps |

> **Do not "fix" the `0.5π` sites to match the `1.5π` sites.** They are not
> inconsistent — GDM and GMRF add the `π` reversal after the TF rather than
> before it. Both reach the same map-frame flow direction. Changing one in
> isolation silently reverses the wind for that consumer.

The counterpart of this contract lives in
`main_decision_msgs/msg/Observation.msg`, whose `wind` field is already
**flow direction in the map frame** as a `Vector3` (`x=u` east+, `y=v` north+).
The conversion happens exactly once per consumer, at the subscription boundary.

### 3.3 Publishing correctly

```python
from olfaction_msgs.msg import Anemometer
import math

msg = Anemometer()
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = 'anemometer'   # required — consumers TF from this frame
msg.sensor_label = 'anemometer_0'    # optional; nothing reads it
msg.wind_speed = 1.5                 # m/s, must be >= 0 and finite
msg.wind_direction = math.pi / 2     # wind blowing FROM the East
```

`header.frame_id` must be set and must exist in the TF tree. Every consumer
transforms from it; an empty frame is rejected outright at
`particle_filter_standalone_node.cpp:84`.

**Validity.** Consumers defend against bad values, and so should producers:

| Guard | Site |
|---|---|
| `frame_id` non-empty, `wind_speed` finite and `>= 0`, `wind_direction` finite | `particle_filter_standalone_node.cpp:84-85` |
| `wind_speed > 0.01` before computing a direction | `gdm_node.cpp:408` |
| `wind_speed != 0.0` before the TF | `gmrf_node.cpp:187` |

The speed gates exist because direction is meaningless at zero speed — a
still-air reading carries an arbitrary bearing that would otherwise enter the
field estimate as a real observation.

---

## 4. The Gas Messages

None of the three are used by GSExploration. Documented here for completeness
and for downstream users of the upstream ecosystem.

### 4.1 GasSensor.msg

A generic gas reading with provenance and calibration metadata: `technology`,
`manufacturer`, `mpn`, then `raw`, `raw_units`, `raw_air`, `calib_a`,
`calib_b`.

`raw_air` is the clean-air baseline; `calib_a`/`calib_b` are sensor-dependent
constants whose meaning the message deliberately does not fix — MOX
resistance-ratio curves and electrochemical linear fits need different
parameters, so the type stays generic.

The file defines **33 `uint8` constants** in four groups. None is referenced
anywhere in the GSExploration workspace.

**Technology** (`TECH_*`): `UNKNOWN=0`, `MOX=1`, `AEC=2`, `EQ=50`, `PID=51`,
`SAW=52`, `TDLAS=53`, `TEMP=100`, `HUMIDITY=101`, `NOT_VALID=255`

**Manufacturer** (`MANU_*`): `UNKNOWN=0`, `FIGARO=1`, `ALPHASENSE=2`, `SGX=3`,
`RAE=50`, `HANWEI=51`, `NOT_VALID=255`

**Part number** (`MPN_*`): `UNKNOWN=0`, `TGS2620=50`, `TGS2600=51`,
`TGS2611=52`, `TGS2610=53`, `TGS2612=54`, `MINIRAELITE=70`, `NOT_VALID=255`

**Units** (`UNITS_*`): `UNKNOWN=0`, `VOLT=1`, `AMP=2`, `PPM=3`, `PPB=4`,
`OHM=5`, `PPMXM=6`, `CENTIGRADE=100`, `RELATIVEHUMIDITY=101`, `NOT_VALID=255`

> The numbering is banded, not sequential: `1–49` solid-state,
> `50–99` instrument-class, `100+` environmental, `255` invalid. New values
> should respect the band or the sentinel loses meaning.
>
> `TECH_TEMP` and `TECH_HUMIDITY` reflect the type's dual role — the same
> message carries environmental channels of an e-nose, which is why
> `GasSensorArray` is described upstream as "gas, temp, RH".

### 4.2 GasSensorArray.msg

`header` plus `GasSensor[] sensors` — the common format for electronic noses,
where one physical device exposes many heterogeneous channels.

### 4.3 TDLAS.msg

Tunable Diode Laser Absorption Spectroscopy — a **remote** sensor, unlike the
others. It measures path-integrated concentration, hence the `ppmxm` units
(ppm × metre): a column density along the beam, not a point sample.

```
uint8     average_ppmxm                 # ppm x meter
float32   average_reflection_strength   # no units given
float32   average_absorption_strength   # no units given
uint8[]   ppmxm                         # 5 consecutive readings
float32[] reflection_strength
float32[] absorption_strength
```

Each message carries one averaged value plus the 5 raw readings it came from.
The reflection/absorption strengths are unitless quality indicators — a high
`ppmxm` with low reflection strength is an unreliable reading, not a detection.

> `average_ppmxm` and `ppmxm[]` are `uint8`, capping at 255 ppm·m — an
> upstream design limit, not a bug, but it makes the type unsuitable for
> high-concentration work.

### 4.4 Why GSExploration does not use the gas messages

Gas concentration in this workspace travels as **`geometry_msgs/Vector3Stamped`**
on `/gas_data`, not as `GasSensor`
(`main_decision_debug/src/fake_sensor_node.cpp:226`). Wind uses `Anemometer`
from this package; gas does not.

The workspace needs a scalar concentration and a timestamp. `GasSensor`'s
value is its *metadata* — technology, calibration, units — which matters when
fusing heterogeneous real sensors, and which a GADEN simulation does not have.
The simulated source emits a single calibrated scalar, so the metadata fields
would all be `UNKNOWN`.

This is a defensible simplification with a real cost: **the gas path carries no
units**. A `Vector3Stamped` cannot say whether its value is ppm or volts. The
workspace resolves this by convention — `th_gas_present` is documented as
1.0 V — but nothing in the type system enforces it. Migrating the gas path to
`GasSensor` would make units explicit and is the obvious upgrade if real
multi-sensor hardware is ever added.

---

## 5. Consumers and QoS

Every subscriber uses **`SensorDataQoS`** (best-effort, volatile, small depth).
A reliable publisher is compatible with a best-effort subscriber, but not the
reverse — a publisher must not be *more* restrictive.

| Consumer | Site | Topic source | QoS |
|---|---|---|---|
| `sampling_node` | `main_decision/src/sampling_node.cpp:227` | param `topics.wind_topic`, default `wind_data` | `sensor_qos` |
| `gsl_local_search` (`AlgorithmBase`) | `gsl_local_search/src/algorithm_base.cpp:39` | param `anemometer_topic`, default `wind_data` | `SensorDataQoS()` |
| `gsl_local_search` (standalone PF) | `gsl_local_search/src/particle_filter_standalone_node.cpp:54` | hardcoded `wind_data` | `SensorDataQoS()` |
| `gsl_streamline` | `gsl_streamline/src/gsl_streamline_server.cpp:355` | `params_.wind_topic` | `sensor_qos` |
| `gas_distribution_mapping` | `gas_distribution_mapping/src/gdm_node.cpp:61` | `anemometer_topic` param, default `/anemometer`; yaml sets `/wind_data` | `sensor_qos` + explicit `best_effort` |
| `GMRF-wind` | `GMRF-wind/gmrf_wind_mapping/src/gmrf_node.cpp:53` | `sensor_topic` param, default `/anemometer`; launch overrides to `/wind_data` | `SensorDataQoS()` |
| `main_decision_viz` (`sensor_visualizer`) | `main_decision_viz/main_decision_viz/sensor_visualizer.py:49` | hardcoded `/wind_data` | `BEST_EFFORT`, volatile, depth 10 |

> **`gas_distribution_mapping` does not subscribe in the shipped config.** The
> subscription is gated on `use_wind`, which is `false` in
> `gdm_params.yaml:72` — the node runs plain Kernel DM+V without the wind-weighted
> W term. Set it to `true` to activate this consumer.
>
> Two subscribers default their topic parameter to `/anemometer`, not
> `/wind_data`, and rely on launch/YAML to override it. Running either node
> standalone without its config leaves it subscribed to a topic nothing
> publishes — a silent no-data condition, not an error.

Best-effort is deliberate: this is a high-rate sensor stream over WiFi on the
real robot, where a dropped sample is preferable to head-of-line blocking. The
estimators are all incremental and tolerate gaps.

`/wind_data` is recorded into campaign bags
(`main_decision/main_decision_scripts/trial_recorder.py:17`).

---

## 6. Data Sources

`Anemometer` messages come from one of two producers, never both:

| Producer | Package | Status |
|---|---|---|
| `fake_sensor_node` | `main_decision_debug` | **The actual simulation producer.** Calls `gaden_player` services and republishes as `Anemometer` on `wind_topic`, default `/wind_data` (`fake_sensor_node.cpp:75,227`). Launched at `demo_sim.launch.py:762-764` |
| `simulated_anemometer` | **External** — `~/Workspace/Libraries_ws/` (GADEN) | **Never launched from this tree.** Publishes to `<node_fqn>/WindSensor_reading`, depth 20 |
| Hardware driver | — | Real robot; `demo_real.launch.py:424` wires `sensor_topic` to `<namespace>/wind_data` |

> The GADEN `simulated_anemometer` / `simulated_gas_sensor` nodes are the
> canonical upstream producers of these messages, but GSExploration does not
> use them. `fake_sensor_node` talks to `gaden_player` over **services**
> (`fake_sensor_node.cpp:200-221`) and synthesizes the sensor topics itself —
> which is what makes replayable per-trial sensor noise possible
> (`fake_sensor_params.yaml:13-14` sets `wind_speed_noise_std` and
> `wind_direction_noise_std`).
>
> Consequence: the only `GasSensor` publishers that exist anywhere are in
> GADEN, and they are never started here — reinforcing §4.4.

---

## 7. Building

Built as part of the GSExploration workspace:

```bash
cd ~/Workspace/Bio_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select olfaction_msgs --symlink-install
source install/setup.bash
```

This package builds **first** in the dependency chain —
`olfaction_msgs` → `main_decision_msgs` → `gsl_streamline` → `main_decision` —
so regenerating it rebuilds most of the workspace.

Inspect a generated type:

```bash
ros2 interface show olfaction_msgs/msg/Anemometer
ros2 topic echo /wind_data --once
```

**Dependencies**: `std_msgs`, `rosidl_default_generators` (build),
`rosidl_default_runtime` (exec). Nothing else — the package is intentionally
standalone so it can be vendored into unrelated projects.

Dependent packages in this workspace declaring `olfaction_msgs`:
`main_decision`, `gas_distribution_mapping`, `gsl_local_search`,
`gsl_streamline`, `main_decision_debug`, `gmrf_wind_mapping`.

---

## 8. Fork Status and Submodule Workflow

This directory is a git submodule pinned at `77c4c22` on branch `ros2`.

| Remote | URL |
|---|---|
| `origin` | `git@github.com:Windrist/olfaction_msgs.git` |
| `upstream` | `git@github.com:MAPIRlab/olfaction_msgs.git` |

### What this fork changes

**One line**, commit `77c4c22`:

```diff
-float32 wind_direction	#rad
+float32 wind_direction	# rad; clockwise UPWIND bearing in sensor frame (0 = North, pi/2 = East)
```

That is the entire functional divergence from upstream — a comment. No field,
type, or constant differs. The fork exists to **pin the wind convention in the
IDL**, where a reader encounters it, rather than leaving it to be rediscovered
from consumer code. Given that six consumers each apply a rotation to this
field, the comment is doing real work.

Because the divergence is comment-only, generated code is **binary-compatible
with upstream**. A node built against MAPIRlab's version interoperates with one
built against this fork.

### Working on this package

```bash
# Changes here belong to the fork, not to GSExploration
cd ~/Workspace/Bio_ws/src/olfaction_msgs
git checkout ros2          # submodules default to detached HEAD
# ... edit ...
git commit -am "..."
git push origin ros2

# Then record the new pointer in the parent repo
cd ~/Workspace/Bio_ws/src
git add olfaction_msgs
git commit -m "chore: bump olfaction_msgs"
```

Fresh clones need `git submodule update --init --recursive`.

> Keep the divergence minimal. Every added local change is one more thing to
> reconcile when pulling upstream, and the value of using a *standard* olfaction
> message set drops as the fork drifts. Prefer contributing genuinely useful
> changes upstream over accumulating them here.

---

## 9. Known Issues

### 9.1 `sensor_label` is read only offline — but it is validated

No **live** node reads `Anemometer.sensor_label`; multi-anemometer setups are
disambiguated by `header.frame_id`, which the TF path requires anyway.

It is not dead, however. `post_evaluation/final_test/bag_validation.py:571-572`
asserts the field is a `str` on every `/wind_data` message, so a producer that
leaves it empty **fails bag validation** even though every live consumer would
run fine. `fake_sensor_node.cpp:331` sets it to `"fake_anemometer"`.

Populate it. The cost is one string assignment; the failure mode is a campaign
bag rejected after the trial has already run.

### 9.2 A dead `GasSensor` include

`gsl_local_search/include/gsl_local_search/algorithm_base.hpp:24` includes
`olfaction_msgs/msg/gas_sensor.hpp`, but no `msg::GasSensor` is ever
instantiated — that package's own README already flags it
(`gsl_local_search/README.md:874`). It is the only reference to the type in the
workspace, and removing it would drop a build dependency edge.

### 9.3 `TDLAS.ppmxm` is `uint8`

Both `average_ppmxm` and the `ppmxm[]` array are `uint8`, capping readings at
255 ppm·m. Upstream design limit. Unused here, but it would need addressing
before any TDLAS deployment.

### 9.4 `package.xml` contains a stray comment

`package.xml:15` has `# TO ADD MSG/SRV` — a shell-style comment inside XML,
where it is character data rather than a comment. It parses today because it
sits between elements, but it is not valid XML commenting and should be
`<!-- TO ADD MSG/SRV -->`. Inherited from upstream.

### 9.5 Version is `0.0.0`

`package.xml:5` declares version `0.0.0`, so downstream packages cannot express
a meaningful version constraint. Inherited from upstream.

### 9.6 Fixed in this revision

The previous README linked `../documents/knowledge-overview.md`, which does not
exist in this repository or the parent workspace, and gave the build path as
`~/Bio_ws` (actual: `~/Workspace/Bio_ws`). It also documented `GasSensor`
constants without noting that nothing in GSExploration references them, and
listed `main_decision`/`gas_distribution_mapping`/`gsl_local_search`/`GMRF-wind`
as integrations without stating that all four consume only `Anemometer`.

---

## 10. References

- [MAPIRlab/olfaction_msgs](https://github.com/MAPIRlab/olfaction_msgs) — upstream repository
- [GADEN](https://github.com/MAPIRlab/gaden) — gas dispersion simulator; source of `simulated_anemometer` and `simulated_gas_sensor`
- `../CLAUDE.md` — workspace guide; wind convention is gotcha #1
- `../main_decision_msgs/README.md` — the workspace's own IDL package; `Observation.msg` is the map-frame counterpart to `Anemometer`
- Monroy, Lilienthal et al., MAPIR Lab, Universidad de Málaga — originating research group

## License

GPL-3.0 — see [`LICENSE`](LICENSE). Note this differs from the GSExploration
workspace (MIT); the submodule keeps its upstream licence.
