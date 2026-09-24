# SVPWM 生成模块（两电平 / 三电平）

面向 TMS320F28379D（C2000）的两套空间矢量脉宽调制实现：**两电平 VSI** 与**三电平中点钳位（NPC）**。均为纯 C 浮点实现，输入 αβ 轴参考电压矢量，输出 PWM 比较值。

## 目录结构

```text
svpwm_code/
├── README.md
├── svpwm_code_Twolevel/
│   ├── Svpwm_gen_two_levlel.c      # 两电平算法实现
│   └── Svpwm_gen_two_levlel.h      # 数据结构与接口声明
└── svpwm_code_Threelevel/
    ├── Svpwm_gen_three_levlel.c    # 三电平算法实现
    └── Svpwm_gen_three_levlel.h    # 数据结构、常数与接口声明
```

## 模块对比

| 项目 | 两电平 | 三电平 |
| --- | --- | --- |
| 入口函数 | `SVPWM_Calc` | `Svpwm_gen_Update` |
| 拓扑 | 2 电平 VSI | 3 电平 NPC |
| 输出通道 | 3 路 | 6 路（上管 T1 / 下管 T4 各 3） |
| 输出类型 | `float` 时间值 | `uint16_t` 计数值 |
| 输入防护 | 无 | `NULL` / `Udc <= 0` 检查 |
| 线性区边界 | `t1 + t2 > Tpwm` 时等比缩放 | `k > 1` 时硬钳位 |
| 用到的超越函数 | 无（纯四则运算） | `atan2f` / `sinf` / `hypotf` |
| 依赖头文件 | `<math.h>` | `<math.h>` + `<stdint.h>` |

> 三电平版在中断里调用 `atan2f`/`sinf`/`hypotf`，相比两电平版的纯算术实现，单次执行开销明显更大。若 ISR 时间紧张，可考虑改用查表或两电平版。

## 一、两电平 SVPWM

### 快速开始

```c
#include "Svpwm_gen_two_levlel.h"

Svpwm_cal svpwm;

void Init(void)
{
    svpwm.Udc  = 100.0f;   /* 母线电压 (V) */
    svpwm.Tpwm = 5000.0f;  /* PWM 周期值，单位与输出 Tcmp 一致，通常直接传 ARR */
}

/* 在每个开关周期中断里调用 */
__interrupt void SvpwmIsr(void)
{
    svpwm.Valpha = ...;    /* 由电流环 / FOC 给出 */
    svpwm.Vbeta  = ...;

    SVPWM_Calc(&svpwm);

    EPwm1Regs.CMPA.bit.CMPA = (uint16_t)svpwm.Tcmp1;
    EPwm2Regs.CMPA.bit.CMPA = (uint16_t)svpwm.Tcmp2;
    EPwm3Regs.CMPA.bit.CMPA = (uint16_t)svpwm.Tcmp3;
}
```

### API

#### `Svpwm_cal`

| 字段 | 方向 | 说明 |
| --- | --- | --- |
| `Valpha` | 输入 | α 轴参考电压 (V) |
| `Vbeta` | 输入 | β 轴参考电压 (V) |
| `Udc` | 输入 | 直流母线电压 (V)，**不可为 0** |
| `Tpwm` | 输入 | PWM 周期值，单位与输出一致 |
| `Tcmp1` `Tcmp2` `Tcmp3` | 输出 | A/B/C 三相比较时间，单位同 `Tpwm` |

#### `void SVPWM_Calc(Svpwm_cal *p)`

原地更新 `Tcmp1/2/3`。无返回值。

- **不做任何输入检查**：`p == NULL` 或 `Udc == 0` 会直接段错误或产生 `inf`/`NaN`，调用方必须自行保证。
- 输出是**时间值而非寄存器计数值**，若 `Tpwm` 传的不是 ARR，上层需自行按 `Tcmp / Tpwm * ARR` 换算。
- 不使用静态变量，函数可重入。

### 算法流程

1. **扇区判断**：三个符号判据按 1 / 2 / 4 权重累加得 `sector`。
   - `Vbeta > 0` → `+1`
   - `√3·Valpha - Vbeta > 0` → `+2`
   - `-√3·Valpha - Vbeta > 0` → `+4`

   注意此处**没有 `atan2`**，也不需要角度折算，比三电平版省一个超越函数。
2. **XYZ 计算**：

   ```text
   x = √3 · Vbeta · Tpwm / Udc
   y = (1.5 · Valpha + √3/2 · Vbeta) · Tpwm / Udc
   z = (-1.5 · Valpha + √3/2 · Vbeta) · Tpwm / Udc
   ```

3. **查表得作用时间**：按 `sector` 从 `x`/`y`/`z` 中选出相邻基本矢量的作用时间 `t1`、`t2`。
4. **过调制处理**：若 `t1 + t2 > Tpwm`，令 `scale = Tpwm / (t1 + t2)` 后对 `t1`、`t2` **等比缩放**。该边界对应内切圆 `Vref = Udc/√3`，缩放保留了矢量角度、牺牲幅值精度。
5. **七段式映射**：`Ta = (Tpwm - t1 - t2) / 4`，`Tb = Ta + t1/2`，`Tc = Tb + t2/2`。
6. **扇区分配**：按 `sector` 把 `Ta`/`Tb`/`Tc` 分派到 `Tcmp1/2/3`。

### 已知限制

- **零矢量输入时输出不更新**：`Valpha = Vbeta = 0` 时三个判据全不成立，`sector` 保持初值 `0`，而第二个 `switch` 只有 `case 1~6`、**没有 `default` 分支**，因此 `Tcmp1/2/3` 不会被赋值，会保持上一次的结果；若这是首次调用，则是未初始化的不确定值。建议补一个 `default` 分支输出 `Tpwm/2`（三相占空比 50%）。
- 无 `NULL` 与 `Udc <= 0` 防护，除零会传播 `inf`/`NaN` 到比较寄存器。
- 过调制只做了等比缩放，未实现最小相位误差等更优策略。
- 头文件中残留 FOC 相关的注释与结构体（`User_IN`、`Voltage_DQ_DEF`、`Current_ABC_DEF` 及被注释的 `FOC_Init`/`FOC_Run`），文件头注释仍写着 `foc_functions.h`，可清理。

## 二、三电平 SVPWM

### 快速开始

```c
#include "Svpwm_gen_three_levlel.h"

Svpwm_threelevel_cal svpwm;

void Init(void)
{
    svpwm.Udc = 800.0f;        /* 母线电压 (V) */
    svpwm.Ts  = 1.0f / 10000;  /* 开关周期 (s)，10 kHz */
    svpwm.sw_period = 3750.0f; /* ePWM 周期计数峰值 ARR，见下文 */
}

/* 在每个开关周期中断里调用（例如 ePWM 的 ZERO 中断，或固定频率的 ISR） */
__interrupt void SvpwmIsr(void)
{
    svpwm.Valpha_ref = ...;    /* 由外环/电流环给出 */
    svpwm.Vbeta_ref  = ...;

    Svpwm_gen_Update(&svpwm);

    EPwm1Regs.CMPA.bit.CMPA = svpwm.Cmpa;  /* A 相上管 T1 */
    EPwm2Regs.CMPA.bit.CMPA = svpwm.Cmpb;  /* B 相上管 T1 */
    EPwm3Regs.CMPA.bit.CMPA = svpwm.Cmpc;  /* C 相上管 T1 */
    EPwm1Regs.CMPB.bit.CMPB = svpwm.Cpmd;  /* A 相下管 T4 */
    EPwm2Regs.CMPB.bit.CMPB = svpwm.Cpme;  /* B 相下管 T4 */
    EPwm3Regs.CMPB.bit.CMPB = svpwm.Cpmf;  /* C 相下管 T4 */
}
```

### API

#### `Svpwm_threelevel_cal`

| 字段 | 方向 | 说明 |
| --- | --- | --- |
| `Valpha_ref` | 输入 | α 轴参考电压 (V) |
| `Vbeta_ref` | 输入 | β 轴参考电压 (V) |
| `Udc` | 输入 | 直流母线电压 (V)，须 `> 0` |
| `Ts` | 输入 | 开关周期 (s) |
| `sw_period` | 输入 | ePWM 周期计数峰值 ARR（中央对齐模式下为**半周期**计数值） |
| `Cmpa` `Cmpb` `Cmpc` | 输出 | A/B/C 相**上管** T1 比较值 |
| `Cpmd` `Cpme` `Cpmf` | 输出 | A/B/C 相**下管** T4 比较值 |

#### `void Svpwm_gen_Update(Svpwm_threelevel_cal* svpwm_gen)`

原地更新结构体中的六个比较值。无返回值。

- 指针为 `NULL` 或 `Udc <= 0` 时直接返回，**不修改**输出字段——调用方需自行保证输出初值安全（建议初始化时先写零矢量）。
- 计算过程中不使用静态变量，函数是**可重入**的，但同一结构体不可被并发调用。

### 算法流程

1. **扇区判断**：构造 `u1 = -Vbeta`、`u2 = Vbeta - √3·Valpha`、`u3 = Vbeta + √3·Valpha`，按符号得 `flag = 4a + 2b + c`，查表映射到 6 个大扇区（各 60°）。`Vref = 0` 时 `flag = 0`，按扇区 1 处理并自然输出零矢量。
2. **角度折算**：`theta_1 = (atan2(Vbeta, Valpha) - (sector-1)·60°) mod 60°`，把矢量折到扇区 1 的 0°~60° 内。
3. **调制比**：`k = √3·|Vref| / Udc`。线性调制区边界为 `k = 1`（对应内切圆 `Vref = Udc/√3`），超出后**硬钳位**到 1，此时伏秒平衡无解，避免出现负作用时间。
4. **小区域判断**：以扇区 1 的几何（V1=POO、V2=PPO、V7=PON、PNN、PPN）划出 6 个小三角形区域，用分界线 `ub = -√3·ua + √3/3·Udc`、`ub = √3·ua - √3/3·Udc`、`ub = √3/6·Udc` 判别。
5. **作用时间**：按 `region` 分 6 种情况计算 `T1 / T2 / T3`（相邻小矢量时间、中矢量时间、零矢量时间）。归一化系数 `2·k·Ts` 源自三电平小矢量在 αβ 坐标系中幅值为 `Udc/3`。
6. **数值兜底**：`T1/T2/T3` 逐项非负钳位；若总和 `> Ts` 则等比缩放到 `Ts`；最后逐项钳到 `[0, Ts]`。这一步只用于修正浮点误差，**不应作为常规过调制手段**。
7. **比较值映射**：按七段式对称排列换算成三个切换时刻 `c0/c1/c2`，再依 `sector` 分派到六路比较寄存器。

### ePWM 硬件配置约定

`SVPWM_PWM_INVERTED` 宏（默认 `0`）用于匹配 Action Qualifier 的极性：

| 取值 | 含义 |
| --- | --- |
| `0`（默认） | 上管 T1 在 `counter >= Cmp_up` 时导通（P 态居载波峰顶）；下管 T4 在 `counter <= Cmp_dn` 时导通（N 态居载波谷底） |
| `1` | 比较值越大脉宽越窄的相反配置 |

所需 ePWM 设置：

- **中央对齐**（up-down）计数模式
- 计数器范围 `0 → ARR → 0`
- `ARR = sw_period`，即半周期的计数峰值

### 已知限制

- **外区矢量序列未完成**：`Ta/Tb/Tc` 的 `0.25 / 0.5` 系数只在“内三角 region 1/2 + ePWM 半周期计数 ARR + 中央对齐 + 比较值越大脉宽越宽”的约定下成立。region 3~6（中三角、外三角）需要按各自矢量序列另行排布，当前实现直接套用了同一组系数，**输出正确性未经验证**。
- 线性调制区限制在 `k = 1`（内切圆），超出后是硬钳位而非等比缩放，矢量角度会失真。
- 中性点电位平衡：算法本身不含中点平衡控制，需在上层加入冗余小矢量调节。
- 未涉及死区补偿与开关损耗优化。

## 共用注意事项

- **头文件宏重复定义**：两个头文件都定义了 `pi`、`sqrt3`、`sqrt2`、`sqrt3_2`、`sqrt3_3`、`rad_to_deg`、`deg_to_rad`。目前两处取值完全一致，C 标准允许这种相同定义的重定义；但一旦其中一处被改动（例如统一改成 `float` 常量），同时包含两个头文件的源文件就会编译报错。建议下沉到一个公共头文件。
- **常量命名**：`sqrt3_2` 表示 `√3/2`，`sqrt3_3` 表示 `1/√3`（而非 `√3/3`），命名容易误读，引用时以头文件注释为准。

## 数值常数

头文件中除 `pi`、`sqrt3` 等通用常数外，还预定义了扇区/区域判据用到的常量：

| 宏 | 值 | 用途 |
| --- | --- | --- |
| `sqrt3_2` | √3/2 | — |
| `sqrt3_3` | 1/√3 | 小矢量区域分界线常数 |
| `sqrt3_4` | √3/4 | — |
| `sqrt3_6` | √3/6 | 中矢量 V7 的 β 坐标 |
| `pi_3` `pi_6` | π/3, π/6 | 扇区角度 |
| `CLAMP01(x)` | — | 钳位到 `[0, 1]` |

后四项仅三电平头文件包含。

## 当前问题

当前代码还没验证，欢迎提交 PR 修复问题。

### 三电平

1. `SVPWM_PWM_INVERTED` 宏已定义但**未被任何代码引用**，改它不会改变输出极性。
2. region 3~6 的比较值排布未经验证，见上文「已知限制」。

### 两电平

1. `Valpha = Vbeta = 0` 时 `Tcmp1/2/3` 不被更新（第二个 `switch` 缺 `default` 分支）。
2. 缺少 `NULL` 与 `Udc <= 0` 防护。
3. 头文件残留 FOC 代码与过时的文件头注释。

## 移植提示

代码仅依赖 `<math.h>` 与 `<stdint.h>`。在 C2000 上编译时建议启用 FPU32 与 `--fp_mode=relaxed`，并确认 `atan2f`/`sinf`/`hypotf` 由 ROM 数学库或 RTS 库提供。若移植到定点 DSP，需将浮点运算改写为 Q 格式定点。
