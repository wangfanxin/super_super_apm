# `sim/legacy/` —— Phase 1b 的回滚路径（正常飞行不使用）

Phase 1b 之后，**飞行路径的真值来自 `sim/src/sim_state_recv.py --ros`**（自建 MAVLink 客户端订阅 SIM_STATE，
msg 108），不再需要固件补丁发出的 truth tap。这里保留旧的 tap 链路，用于：

* 回滚：`TRUTH_SRC=tap`（要求 patched SITL 二进制）；或 `LEGACY_TAP=1` 与 SIM_STATE 并录一趟做双真值对比；
* 取证：`truth_rx.py --selftest`、以及任何需要 1.2 kHz 真值的离线重放。

| 文件 | 作用 | 使用方式 |
|---|---|---|
| `apm-0001-sitl-truth-tap.patch` | ArduPilot SITL 的 tap 补丁（`LIO_TRUTH_TAP` 每模型步一行 TAP） | `cd .build/ardupilot && patch -p1 < .../apm-0001-sitl-truth-tap.patch && ./waf copter` |
| `truth_rx.py` | 接收 tap UDP（14560），校验并分发 /sim/truth/odom、/clock、/sim/rtf、fanout（14561），写 `tap.csv` | `python3 sim/legacy/truth_rx.py --ros --moving --port 14560 --fanout 14561 --fanout-decimate 6 --csv OUT/tap.csv` |

**默认二进制是 UNPATCHED SITL**（`.build/ardupilot/build/sitl/bin/arducopter`）；打补丁的版本在构建时被
备份为同目录的 `arducopter.tap-patched`，`scripts/run-mission.sh` 只在 `TRUTH_SRC=tap` 或 `LEGACY_TAP=1`
时选它。重新打补丁/重编会覆盖默认二进制——`setup/build.sh` 与 `docs/PHASE1-TRUTH.md` 记录了步骤。

删除条件：当 Phase 2 的 bag 基座被验收（真值在 bag 里、无需 tap）、且不再需要双真值对比时，本目录可以整目录删除。
