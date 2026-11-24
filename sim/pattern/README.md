# harness/sim —— 传感器与世界的仿真数据

这个目录放**数据**；代码在同级的 `harness/src/sim/`（`lidar_imu.py`、`raycast.py`、`make_world.py`）。
飞行链需要其中**一个文件**，而它没有、也不应该进 git：

## `mid360_pattern.csv`（≈16 MB，供应商数据）

* **是什么**：真实 Livox MID-360 的逐点射线图案（方位角 / 天顶角 / 点序号），800 001 行 = 1 秒 @ 200 kHz。
  格式与来源写死在 `harness/src/sim/raycast.py:load_pattern()` 的注释里：
  表头 `Time/s,Azimuth/deg,Zenith/deg`，第一列实际是**点序号**（步长 1），加载器会自动识别并除以点率。
* **为什么要它**：合成图案（`--allow-synth`）的盲区与重复性都和真机不同，**只能做冒烟测试，不能出测量数字**。
  `lidar_imu.py` 在找不到真图案时会**拒绝启动**并打印所有查找路径。
* **从哪来**：`https://github.com/Livox-SDK/livox_laser_simulation` 的 `scan_mode/mid360.csv`。
* **放在哪**（按优先级）：
  1. 环境变量 `MID360_PATTERN=/path/to/mid360_pattern.csv`
  2. 本目录：`harness/sim/mid360_pattern.csv`   ← 推荐
  3. `~/lio_avoid/sim/mid360_pattern.csv`（旧布局）
* **一条命令获取**：

  ```bash
  curl -fsSL -o harness/sim/mid360_pattern.csv \
    https://raw.githubusercontent.com/Livox-SDK/livox_laser_simulation/main/scan_mode/mid360.csv
  head -1 harness/sim/mid360_pattern.csv     # 期望：Time/s,Azimuth/deg,Zenith/deg
  wc -l  harness/sim/mid360_pattern.csv      # 期望：800001
  ```

* **校验**：`bash scripts/check_pattern.sh` 会打印默认查找路径、加载 2 万条射线并给出方位/仰角范围
  （真图案：方位 0–360°、仰角约 −7°…+52°）。低于这个范围或行数不等于 800001，说明拿错了文件。

本目录其余内容（`scan_mode/`、其它 CSV）同理由 `.gitignore` 排除；世界文件在 `worlds/`，不在这里。
