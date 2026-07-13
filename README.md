# SPICE-like Circuit Simulator

这是一个基于 C++17 和 Eigen 的类 SPICE 电路仿真器。目前支持 DC operating point 与基础 transient analysis，使用稀疏 MNA、SparseLU、Newton 迭代和 source stepping 完成求解；电容与电感的瞬态离散采用 Backward Euler。

本项目实现的是明确受限的 SPICE 子集，并非完整 ngspice 替代品。I/O 层遵循常见 SPICE netlist、`.print` listing 和 ASCII rawfile 约定，未支持的控制卡或输出表达式会直接报错，不会静默忽略。

## 已支持功能

### 器件

| 前缀 | 器件 | OP | TRAN |
| --- | --- | --- | --- |
| `R` | Resistor | 电导 stamp | 电导 stamp |
| `C` | Capacitor | 开路 | Backward Euler companion model |
| `L` | Inductor | 0 V 支路 | Backward Euler branch equation |
| `V` | Independent voltage source | MNA branch | 当前仅支持固定 DC 值 |
| `I` | Independent current source | RHS stamp | 当前仅支持固定 DC 值 |
| `D` | Diode | 指数模型与 Newton 线性化 | 静态非线性模型 |
| `Q` | NPN / PNP BJT | 简化 Ebers-Moll 风格模型 | 静态非线性模型 |
| `M` | NMOS / PMOS | 简化 Level-1 平方律模型 | 静态非线性模型 |

`.model` 支持 `D`、`NPN`、`PNP`、`NMOS`、`PMOS`、`NCH`、`PCH`。当前使用的主要参数包括：

- Diode：`IS`, `N`, `VT`, `RS`, `GMIN`
- BJT：`IS`, `BF` / `BETA`, `BR`, `NF`, `NR`, `VT`, `GMIN`, `RBE`, `RCE`
- MOSFET：`LEVEL=1`, `VTO` / `VT0`, `KP` / `K`, `LAMBDA` / `LAM`, `GMIN`, `RDS`
- 实例参数：`AREA`, `W`, `L`

上述是当前求解模型真正使用的参数集合。未知参数、非数值参数、非物理的负值以及非 `LEVEL=1` 的 MOSFET model 会在读取阶段明确报错，避免网表被静默降级求解。

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
- `TSTEP` 控制输出间隔，`TSTART` 控制开始保存的时间，`TMAX` 限制内部积分步长。当前未指定 `TMAX` 时内部步长使用 `TSTEP`，这与 ngspice 的默认步长选择策略不同。

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
./spice testcase/op/level1_01_resistor_divider.cir
./spice testcase/op/level1_01_resistor_divider.cir result.out
./spice -b -o result.out testcase/op/level1_01_resistor_divider.cir
```

同时生成 listing 与 rawfile：

```sh
./spice -b -o result.out -r result.raw testcase/tran/level1_01_rc_step_uic.cir
```

查看命令行帮助：

```sh
./spice --help
```

## 自动测试

测试与参考结果按分析类型分离：

```text
testcase/
  op/       18 个 operating-point netlist
  tran/     18 个 transient netlist
standard/
  op/       18 个 OP listing reference
  tran/     18 个 TRAN listing reference
actual/
  op/       测试产生的 .out / .raw / .err
  tran/     测试产生的 .out / .raw / .err
```

运行全部 36 个用例：

```sh
make test
```

也可以分别运行或只比较已有结果：

```sh
make test-io
make test-op
make test-tran
make compare
make compare-op
make compare-tran
```

`make test` 会完成以下检查：

1. 构建 simulator。
2. 使用 `scripts/test_io.py` 检查 SPICE 注释、续行、大小写、严格数值/model/实例参数、`.end`、混合 OP/TRAN 输出、事务式文件替换、硬链接保护和 CLI。
3. 对每个 netlist 同时生成 listing、ASCII rawfile 和 stderr 文件。
4. 使用 `scripts/validate_raw.py` 校验 rawfile header、变量数量、点数、有限数值和瞬态时间单调性，并把 raw 数据与同次 listing 逐点、逐变量交叉核对。
5. 使用 `scripts/compare_spice.py` 解析标准与实际 `Index` 表格，并按绝对误差加相对误差比较。

默认容差：

```make
OP_ABS_TOL    ?= 1e-3
OP_REL_TOL    ?= 2e-3
TRAN_ABS_TOL  ?= 1e-7
TRAN_REL_TOL  ?= 1e-4
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

原有 14 个 OP 用例及其 ngspice 风格参考输出保持不变，并新增 4 个高复杂度 OP 压力用例：BJT 差分对、CMOS NAND/二极管钳位、互补 BJT 桥和 CMOS/BJT/MOS/二极管多级网络。TRAN 的 18 个用例从 RC/RL 基础响应逐步扩展到 RLC ladder、双源储能桥、二极管、NMOS、BJT 和 CMOS/RLC 混合网络。

## 目录结构

```text
include/
  core/          Circuit、Parser、NodeMap 和分析配置
  devices/       器件与 stamp
  io/            SPICE listing / rawfile writer
  math/          Eigen 稀疏 MNA 封装
  models/        .model 参数存储
src/
  core/          核心实现
  io/            输出格式实现
scripts/         listing 比较器与 rawfile 校验器
testcase/op/     OP netlist
testcase/tran/   TRAN netlist
standard/op/     OP reference listing
standard/tran/   TRAN reference listing
```

## 当前限制

- 不支持 `PULSE`、`SIN`、`PWL` 等时变独立源，因此瞬态阶跃测试使用 `UIC` 和固定 DC 源构造 t=0 激励。
- 不支持自适应步长、LTE 控制和高阶积分方法。
- 不支持 `.include`、`.lib`、`.param`、`.options`、`.temp`、`.nodeset`、`.ic`、`.subckt`、`.save`。
- 不支持受控源 `E/F/G/H`、行为源、AC/noise 分析。
- 二极管、BJT 和 MOSFET 是简化模型；瞬态中没有结电容等器件内部动态。
- 电阻、电容、二极管、BJT、MOSFET 的器件电流尚不能通过 `.print i(...)` 输出。

## SPICE 格式参考

I/O 兼容规则主要参考：

- [ngspice User's Manual](https://ngspice.sourceforge.io/docs/ngspice-manual.pdf)
- [ngspice ASCII rawfile writer source](https://sourceforge.net/p/ngspice/ngspice/ci/master/tree/src/frontend/rawfile.c)

本项目对未实现的语法保持显式报错，并在本文档中标明与 ngspice 的差异。
