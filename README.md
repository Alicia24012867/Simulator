# SPICE-like Circuit Simulator

这是一个基于 C++17 和 Eigen 的类 SPICE 电路仿真器。目前支持 DC operating point、基础 transient analysis 最小闭环，以及实验性的 pseudo-transient analysis (PTA) 路径，使用稀疏 MNA、SparseLU、Newton 迭代、步长限制和 source stepping 完成求解。瞬态分析首个积分步采用 Backward Euler，后续在步长增长不超过前一步两倍时采用可变步长 BDF2。这里的 TRAN 是可运行的 MVP，不等同于完整 SPICE 瞬态引擎。

本项目实现的是明确受限的 SPICE 子集，并非完整 ngspice 替代品。I/O 层遵循常见 SPICE netlist、`.print` listing 和 ASCII rawfile 约定，未支持的控制卡或输出表达式会直接报错，不会静默忽略。

## 已支持功能

### 器件

| 前缀 | 器件 | OP | TRAN |
| --- | --- | --- | --- |
| `R` | Resistor | 电导 stamp | 电导 stamp |
| `C` | Capacitor | 开路 | Backward Euler / BDF2 companion model |
| `L` | Inductor | 0 V 支路 | Backward Euler / BDF2 branch equation |
| `V` | Independent voltage source | MNA branch | 当前仅支持固定 DC 值 |
| `I` | Independent current source | RHS stamp | 当前仅支持固定 DC 值 |
| `D` | Diode | 指数模型与 Newton 线性化 | 静态非线性模型 |
| `Q` | NPN / PNP BJT | 简化 Ebers-Moll 风格模型 | 静态非线性模型 |
| `M` | NMOS / PMOS | 简化 Level-1 平方律模型 | 静态非线性模型 |

`.model` 支持 `D`、`NPN`、`PNP`、`NMOS`、`PMOS`、`NCH`、`PCH`。当前求解方程使用的模型参数包括：

- Diode：`IS`, `N`, `VT`, `GMIN`
- BJT：`IS`, `BF` / `BETA`, `BR`, `NF`, `NR`, `VT`, `GMIN`
- MOSFET：`LEVEL=1`, `VTO` / `VT0`, `KP` / `K`, `LAMBDA` / `LAM`, `GMIN`
- 实例参数：`AREA`, `W`, `L`

读取器还接受 Diode 的 `RS`、BJT 的 `RBE` / `RCE` 和 MOSFET 的 `RDS`，但这些参数尚未写入器件 stamp，因此暂不改变求解结果。未知参数、非数值参数、非物理的负值以及非 `LEVEL=1` 的 MOSFET model 会在读取阶段明确报错，避免网表被静默降级求解。

### Netlist 读取规则

- 第一物理行作为 circuit title；`.title ...` 可以覆盖它。
- 元件名、模型名、节点名和控制卡大小写不敏感。
- `0` 是 ground；额外接受 `gnd` 作为别名。
- 整行注释使用 `*`；行尾注释支持 `;`、`$` 和 `//`。
- `#` 不是注释符，因为 SPICE branch vector 会使用 `v1#branch` 形式。
- `+` 开头的逻辑续行会拼接到上一条语句；孤立续行会报告文件名和行号。
- `.end` 必须存在、不接受参数，并且必须是最后一个非注释语句。
- 支持数值后缀 `a`, `f`, `p`, `n`, `u`, `m`, `mil`, `k`, `meg`, `g`, `t`；其中 `m` 表示 milli，mega 必须写成 `meg`。
- 独立源接受 `5`、`DC 5`、`DC=5`、`DC= 5` 和 `DC = 5`；当前不接受任意 `key=value` 代替 DC 值。
- 数值 token 会完整校验，`1..2`、`1k=2` 等畸形写法不会只读取前缀后继续运行。
- 网表必须至少包含一个元件和一个非 ground 节点，避免把零维 MNA 系统送入求解器。
- 当前支持的控制卡为 `.title`、`.model`、`.op`、`.tran`、`.print` 和 `.end`。其他 dot command 会明确报错。

### 分析与 `.print`

支持：

```spice
.op
.tran TSTEP TSTOP [TSTART [TMAX]] [UIC]
.print op v(node) v(node1,node2) i(device)
.print tran v(node) v(node1,node2) i(device)
```

- `v(node1,node2)` 在输出层计算差分电压，不改变 MNA 方程。
- transient listing 自动把 `time` 放在第一列；`.print tran time ...` 也可以接受。
- 多条相同分析类型的 `.print` 会按出现顺序合并并去重。
- 没有 `.print` 时，默认输出所有非 ground 节点电压和所有 branch unknown 电流。
- `i(device)` 当前只适用于具有 branch unknown 的器件，即独立电压源和电感；请求其他器件电流会得到明确错误。
- 没有分析卡时默认执行 `.op`。同时存在 `.op` 与 `.tran` 时依次输出两个分析块。
- `.tran` 未指定 `UIC` 时先求 operating point；指定 `UIC` 时当前使用全零 MNA 初值。尚未支持器件 `IC=`。
- `TSTEP` 控制输出间隔，`TSTART` 控制开始保存的时间，`TMAX` 限制内部积分步长。当前未指定 `TMAX` 时内部最大步长使用 `TSTEP`；输出时间点也会强制成为积分点，因此与 ngspice 的默认自适应步长策略不同。
- 每次瞬态分析的首步使用 Backward Euler 的 step-doubling 误差估计；之后在新步长不大于前一步两倍时使用可变步长 BDF2，并以预测—校正差估计误差。超过误差预算或 Newton 未收敛的步会回滚并缩小后重试；内部积分点仍不会越过输出时间点。

### 实验性 PTA

命令行可选择 OP 求解路径：

```sh
./spice --pta disabled input.cir  # 默认：Newton + source stepping
./spice --pta force input.cir     # 只使用 PTA
./spice --pta fallback input.cir  # 常规 OP 失败后尝试 PTA

# 使用 SPICE 数值后缀覆盖 PTA 配置；可重复指定
./spice --pta force \
  --pta-option initial-step=1n \
  --pta-option maximum-steps=20000 \
  --pta-option successful-step-scale=1.5 \
  --pta-option medium-oscillation-ratio=0.5 \
  --pta-option heavy-oscillation-ratio=1.0 \
  --pta-option include-diodes=true \
  input.cir
```

`--pta-option name=value` 只可与 `--pta force` 或 `--pta fallback` 一起使用。数值选项接受与网表相同的 SPICE 后缀；布尔选项 `include-mos-bulk` 与 `include-diodes` 接受 `true` / `false` 或 `1` / `0`。可用名称为：

- 时间与收敛：`initial-step`、`minimum-step`、`maximum-step`、`maximum-steps`、`derivative-tolerance`、`dc-residual-tolerance`
- 伪元件：`initial-node-capacitance`、`minimum-node-capacitance`、`maximum-node-capacitance`、`current-source-capacitance`、`voltage-source-inductance`
- 自适应规则：`failed-step-scale`、`successful-step-scale`、`capacitance-grow-scale`、`small-oscillation-scale`、`medium-oscillation-scale`、`heavy-oscillation-scale`、`medium-oscillation-ratio`、`heavy-oscillation-ratio`
- 放置开关：`include-mos-bulk`、`include-diodes`

同一 PTA 选项不可重复指定；所有覆盖值会在建模前统一执行配置校验，非法范围或相互矛盾的边界会以命令行错误退出。

PTA 在 MNA pattern 固化前加入人工伪元件：独立电压源 branch 上的伪电感、独立电流源两端的伪电容，以及晶体管节点到地的伪电容。其伪时间迭代复用现有的 Backward Euler / 受步长比限制的 BDF2 `TransientIntegrator`；每一步以原始 OP 方程残差和 BDF 导数范数共同判定稳态。Newton 失败时先缩小伪时间步长；在最小步长仍失败时，增大所有节点伪电容并重启积分历史。每个成功步后，伪时间步会按 `successful-step-scale` 增长，同时仍受最大步长和 BDF2 步长比限制；节点电压变化反向时会按相邻步变化幅度比降低该节点伪电容，两个 ratio 参数分别划分小/中及中/重振荡。

该功能仍处于实验阶段。自适应规则具有单元测试，并有一条端到端夹具覆盖“最小步长失败 → 全局增容 → 重启 → 恢复收敛”路径及后续节点降容。当前 Force OP 回归覆盖的 18 个网表、76 个输出值均可通过参考对比。默认 `derivativeTolerance=1e-8` 仍是固定绝对阈值；对不同伪时间步和未知量尺度的鲁棒性尚需更广泛的基准验证。因此 `force` 与 `fallback` 可用于回归和实验，但暂不视为生产求解保证。

## 输出格式

### SPICE listing

默认写到 stdout；使用位置参数或 `-o` 写入文件。`.print` 决定 listing 的变量和顺序：

```text
Circuit: Level 1 - RC step with UIC

Transient Analysis
No. of Data Rows : 11

----------------------------------------------------------------------------------------
Index   time                v(in)               v(out)              v1#branch
----------------------------------------------------------------------------------------
       0    0.0000000000e+00    0.0000000000e+00    0.0000000000e+00    0.0000000000e+00
       1    1.0000000000e-04    5.0000000000e+00    4.5454545455e-01   -4.5454545455e-03
```

branch current 在 listing 中使用 ngspice 常见的 `device#branch` 列名。

### ASCII rawfile

使用 `-r` 额外写出 SPICE ASCII rawfile。rawfile 不受 `.print` 过滤，包含全部节点电压和全部 branch unknown 电流；瞬态的第一个 variable 固定为 `time`。

```text
Title: Level 1 - RC step with UIC
Date: Tue Jul 14 12:00:00 2026
Plotname: Transient Analysis
Flags: real
No. Variables: 4
No. Points: 11
Variables:
	0	time	time
	1	v(in)	voltage
	2	v(out)	voltage
	3	i(v1)	current
Values:
 0	0.000000000000000e+00
	0.000000000000000e+00
	0.000000000000000e+00
	0.000000000000000e+00
```

输出数值使用 classic locale、科学计数法，并拒绝写出 NaN/Inf。程序先在内存中生成结果，再把所有文件输出写入目标目录中的临时文件；全部暂存成功后才备份并替换旧文件，正常写入错误会触发回滚。解析、构建、求解或暂存失败都不会提前截断旧结果。input、listing 和 rawfile 不能通过规范路径、符号链接或硬链接指向同一文件。

## 构建与运行

依赖：

- C++17 编译器
- Eigen3
- Python 3（测试脚本）
- Make
- ngspice 46（仅在重新生成独立参考结果时需要；普通构建与 `make test` 不需要）

macOS 可安装 Eigen：

```sh
brew install eigen
```

构建：

```sh
make
```

运行并输出 listing：

```sh
./spice tests/cases/op/level1_01_resistive_bridge_mesh.cir
./spice tests/cases/op/level1_01_resistive_bridge_mesh.cir result.out
./spice -b -o result.out tests/cases/op/level1_01_resistive_bridge_mesh.cir
```

同时生成 listing 与 rawfile：

```sh
./spice -b -o result.out -r result.raw tests/cases/tran/level1_01_rc_step_ladder.cir
```

查看命令行帮助：

```sh
./spice --help
```

## 自动测试

测试与参考结果按分析类型分离：

```text
tests/
  cases/
    op/          18 个 operating-point netlist
    tran/        18 个 transient netlist
  references/
    op/          18 个由 ngspice 独立生成的 OP listing reference
    tran/        18 个由 ngspice 独立生成并重采样的 TRAN listing reference
  output/
    op/          测试产生的 .out / .raw / .err
    tran/        测试产生的 .out / .raw / .err
```

运行全部 36 个用例：

```sh
make test
```

`make test-op` 与 `make test-tran` 会在每个网表执行后输出一条
`TIME <analysis> <case> <milliseconds> PASS/FAIL`，并输出该分析组的总墙钟时间。单例时间覆盖 simulator 子进程启动、解析、建模、求解以及 `.out` / `.raw` 写出；rawfile 校验和 ngspice 对照时间不包含在其中，便于 PTA 前后比较求解端到端开销。

PTA Force OP 回归与参考精度比较可一并运行：

```sh
make pta-force-standard
```

当前该套件覆盖 18 个 OP 网表和 76 个输出值。它是 PTA 的基础回归门槛，不替代更大规模的困难非线性电路基准。

也可以分别运行或只比较已有结果：

```sh
make test-io
make test-cases
make test-op
make test-tran
make compare
make compare-op
make compare-tran
```

`make test` 会完成以下检查：

1. 构建 simulator。
2. 使用 `tests/scripts/test_io.py` 检查 SPICE 注释、续行、大小写、严格数值/model/实例参数、`.end`、混合 OP/TRAN 输出、事务式文件替换、硬链接保护和 CLI。
3. 对每个 netlist 同时生成 listing、ASCII rawfile 和 stderr 文件。
4. 使用 `tests/scripts/validate_raw.py` 校验 rawfile header、变量数量、点数、有限数值和瞬态时间单调性，并把 raw 数据与同次 listing 逐点、逐变量交叉核对。
5. 使用 `tests/scripts/compare_spice.py` 解析标准与实际 `Index` 表格，并按绝对误差加相对误差比较。

`make test-cases` 是独立的网表复杂度审计：检查两组各 18 个网表的命名、数量、分析类型、物理行数、有效语句以及最大案例规模；它不属于 `make test` 的常规回归流程。

默认容差：

```make
OP_ABS_TOL    ?= 5e-4
OP_REL_TOL    ?= 1e-4
TRAN_ABS_TOL  ?= 1e-4
TRAN_REL_TOL  ?= 1e-3
TIME_ABS_TOL  ?= 1e-15
```

判定公式：

```text
|actual - expected| <= absolute_tolerance + relative_tolerance * |expected|
```

详细显示每一个比较值：

```sh
make test OP_COMPARE_FLAGS=--verbose TRAN_COMPARE_FLAGS=--verbose
```

OP 与 TRAN 各有 18 个由上游开源电路库拓扑改编的压力用例，Level 分布均为 `4/4/5/5`：Level 1 为 10-20 行，Level 2 为 20-40 行，Level 3 为 40-100 行，Level 4 大于 100 行；两组最大案例均为 340 个物理行。案例不是上游文件的原样复制，而是展平并约束到当前解析器所支持的 primitive-only 子集。详细来源、适配规则和逐级行数见 [`tests/cases/SOURCES.md`](tests/cases/SOURCES.md)。

测试用例与参考值可分别复现：

```sh
python3 -B tests/scripts/generate_complex_cases.py
make test-cases
make generate-standards  # 需要 ngspice
```

`tests/references/` 不使用本项目求解结果自我生成。OP 参考值直接来自 ngspice 46；生成 TRAN 参考时，脚本会向临时网表注入固定的高精度 ngspice 设置：`reltol=1e-8`、`vntol=1e-10`、`abstol=1e-12`、`trtol=1`，并将内部最大步长限制为 `TSTEP / 2000`。随后将 ngspice 结果线性重采样到网表要求的输出时间网格，原始测试网表不会被修改。对于 `UIC` 网表，仅显式 `t=0` 行按本项目当前的全零采样约定处理，所有 `t>0` 数据均来自 ngspice。

## 目录结构

```text
include/
  analysis/      分析计划、瞬态配置、stamp 上下文与积分器
  circuit/       Circuit 求解编排与 NodeMap 拓扑接口
  devices/       器件定义及 OP / TRAN stamp
  io/            SPICE listing / rawfile 输出接口
  math/          Eigen 稀疏 MNA、Newton 步长控制与数值限制工具
  models/        .model 参数存储
  netlist/       网表 Parser 接口
  utils/         跨模块字符串与 SPICE 数值工具
src/
  circuit/       Circuit 与 NodeMap 实现
  io/            listing / rawfile 输出实现
  netlist/       网表解析实现
  main.cpp       命令行入口与事务式输出流程
tests/
  cases/         OP / TRAN netlist 与 SOURCES.md
  references/    ngspice 独立参考 listing
  scripts/       用例生成、参考生成与回归校验脚本
  output/        测试生成的 listing / rawfile / stderr（不纳入版本控制）
```

项目内头文件统一相对于 `include/` 引用。`include/analysis`、`include/devices`、`include/math` 和 `include/models` 当前主要是 header-only 模块；存在独立实现文件的模块则在 `src/` 中使用对应职责目录。默认构建使用 `-O3`；调试时可用 `make OPT_FLAGS=-O0` 覆盖。

## 当前限制

- 不支持 `PULSE`、`SIN`、`PWL` 等时变独立源，因此瞬态阶跃测试使用 `UIC` 和固定 DC 源构造 t=0 激励。
- 瞬态使用首步 Backward Euler 与受步长比限制的可变步长 BDF2；具备基于预测—校正差/step-doubling 的误差控制和步长拒绝重试，但尚未实现严格 LTE 估计、事件断点对齐或高阶积分公式。
- `UIC` 当前把完整 MNA 解向量初始化为零；尚未支持器件 `IC=`、`.ic` 与一致初值求解。
- 瞬态 Newton 失败会缩小时间步并在上一个已接受状态重试；非线性收敛判据本身仍未拆分电压/电流的相对与绝对容差。
- PTA 已具备伪元件 stamp、BE/BDF2 伪时间推进、失败缩步、最小步长后的全局增容，以及成功步后的逐节点振荡降容；默认导数阈值仍需按伪时间步和未知量尺度归一化后再作为可靠收敛判据。
- 不支持 `.include`、`.lib`、`.param`、`.options`、`.temp`、`.nodeset`、`.ic`、`.subckt`、`.save`。
- 不支持受控源 `E/F/G/H`、行为源、AC/noise 分析。
- 二极管、BJT 和 MOSFET 是简化模型；`RS`、`RBE`、`RCE`、`RDS` 虽可解析但尚未参与 stamp，且瞬态中没有结电容等器件内部动态。
- 电阻、电容、二极管、BJT、MOSFET 的器件电流尚不能通过 `.print i(...)` 输出。

## SPICE 格式参考

I/O 兼容规则主要参考：

- [ngspice User's Manual](https://ngspice.sourceforge.io/docs/ngspice-manual.pdf)
- [ngspice ASCII rawfile writer source](https://sourceforge.net/p/ngspice/ngspice/ci/master/tree/src/frontend/rawfile.c)

本项目对未实现的语法保持显式报错，并在本文档中标明与 ngspice 的差异。
