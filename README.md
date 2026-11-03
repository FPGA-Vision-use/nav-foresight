# nav-foresight

**Edge traversability prediction and yielding for delivery robots on KV260**

在 AMD Kria KV260（Zynq UltraScale+ MPSoC, XCK26）上实现的通行预判加速器：
每一帧为移动机器人评估 43 条候选走法各自的安全程度，一次交付 215 个整数（单位毫米）给决策层。

---

## 关键结果（全部为板级实测）

| 项 | 数值 |
|---|---|
| 单帧 PL 耗时 | **15.639 ms**（63.9 fps） |
| 端到端（含 PS 主循环） | **20.6 ms → 48.6 fps** |
| 帧间抖动 | **< 0.06 ms**（中位 15.639 / 最大 15.699） |
| 板级 vs 行为级模型 | **0/215、0/17200 点不一致，最大偏差 0 mm** |
| 453 帧真实回放 | **停/行决策 453/453 一致**；路径最大偏差 1 mm |
| 安全口径 | 板子从不比浮点参考**更乐观**超过 +9.8 mm（偏"多停"） |
| 资源（XCK26） | LUT 46.0% / FF 28.3% / DSP 179 / BRAM 4 |
| 时序 | WNS **+6.015 ns** @ 50 MHz 约束，0 失败端点 |

## 系统结构

```
PS (ARM Cortex-A53)                     PL (FPGA @ 50 MHz)
├── 感知：车/人定位              ┌──► rollout  : 43 条候选 × 400 步 → 17200 字
├── 状态估计：位姿·速度·曲率      │              │
├── 决策：停/行·去抖·硬停   ──AXI-Lite(32b)──►  ├──► clearance: 5 类净空 × 400 点 → 215 整数
└── 显示：三色通行图              │              │
                                  └── DDR 共享缓冲（rollout 写 / clearance 读，不经 PS）
```

纯算术下沉到可编程逻辑，分支与状态机留在处理系统。

## 目录

```
hls/         两个 HLS 算子的源码与综合脚本
bd/          集成 Block Design 脚本（4 个 AXI-Lite 从口 + 显式位宽转换）
hw/          板端驱动与验收脚本（/dev/mem + dma_heap），含纯软件基线 bench_sw.cpp
replay/      PC 侧分析脚本（453 帧误差统计、停/行决策对比、演示渲染）
bitstream/   集成比特流（Vivado 2026.1 重建版）与 .hwh
data/        453 帧浮点参考、板子输出、PS 主循环日志
docs/        技术文档、协作过程决策日志、AMD ROSS 部署实录
docs/codex-export/  31 段人机协作原始记录
skills/      从本工程提炼的可复用验收技能包
```

## 环境

- Vivado / Vitis HLS **2026.1**（2025.2 亦可）
- 目标器件 xck26（Kria KV260）
- 板端 Ubuntu 22.04，Python 3.10+（`paramiko`、`numpy`、`opencv-python`）
- 不依赖 PYNQ Overlay

## 复现

```bash
# 1) 两个 IP
cd hls
vitis_hls -f hls_v13.tcl
vitis_hls -f rollout/hls_roll17.tcl

# 2) 集成比特流（约 25 分钟）
cd ../bd
vivado -mode batch -source build_bd_int6.tcl -tclargs impl

# 3) 上板验收（先设环境变量 BOARD_HOST / BOARD_PWD）
cd ../hw
python host.py intpush && python host.py intload
python host.py chain chain ../data/chain_ref.txt      # 期望 0/215
cd ../replay && python analyze453.py                  # 453 帧误差统计
```

### 纯软件基线（可选）

```bash
g++ -O2 -std=c++14 -I <vitis_include> hw/bench_sw.cpp hls/clearance.cpp hls/rollout/rollout.cpp -o bench
./bench 453
```

## 三个必须知道的坑

1. **PL 配置易失**，每次重启板子都要重新加载比特流。`fpga_manager` 显示 `operating` 只代表 base overlay。
2. **不要用 `host.py verify` 验集成版**——它加载的是单核 clearance 版。
3. **不要用"读回 CTRL=0x4"判断核是否启动**——`ap_done` 是单周期脉冲，只认 `ap_idle` 的 0→1。

完整 13 个坑见 `docs/` 下的技术文档 §7。

## 大模型协作

本工程的代码、调试与文档起草由大模型承担主要工作量；方向、约束与验收标准由人确定，错误由人发现并纠正。
原始交互记录在 `docs/codex-export/`（31 段会话），可逐条核对。

## 许可证

MIT，见 [LICENSE](LICENSE)。
