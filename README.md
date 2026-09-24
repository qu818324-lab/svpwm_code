# 三电平 SVPWM 生成模块

基于 TMS320F28379D（C2000）实现的**三电平中点钳位（NPC）逆变器空间矢量脉宽调制**算法。模块为纯 C 的浮点实现，输入参考电压矢量，输出六路 ePWM 比较值。

## 目录结构

```
svpwm_code/
├── README.md
└── svpwm_code_Threelevel/
    ├── Svpwm_gen_three_levlel.c   # 算法实现
    └── Svpwm_gen_three_levlel.h   # 数据结构、常数与接口声明
```

## 快速开始

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

## API

### `Svpwm_threelevel_cal`

| 字段 | 方向 | 说明 |
| --- | --- | --- |
| `Valpha_ref` | 输入 | α 轴参考电压 (V) |
| `Vbeta_ref` | 输入 | β 轴参考电压 (V) |
| `Udc` | 输入 | 直流母线电压 (V)，须 `> 0` |
| `Ts` | 输入 | 开关周期 (s) |
| `sw_period` | 输入 | ePWM 周期计数峰值 ARR（中央对齐模式下为**半周期**计数值） |
| `Cmpa` `Cmpb` `Cmpc` | 输出 | A/B/C 相**上管** T1 比较值 |
| `Cpmd` `Cpme` `Cpmf` | 输出 | A/B/C 相**下管** T4 比较值 |

### `void Svpwm_gen_Update(Svpwm_threelevel_cal* svpwm_gen)`

原地更新结构体中的六个比较值。无返回值。

- 指针为 `NULL` 或 `Udc <= 0` 时直接返回，**不修改**输出字段——调用方需自行保证输出初值安全（建议初始化时先写零矢量）。
- 计算过程中不使用静态变量，函数是**可重入**的，但同一结构体不可被并发调用。

## 算法流程

1. **扇区判断**：构造 `u1 = -Vbeta`、`u2 = Vbeta - √3·Valpha`、`u3 = Vbeta + √3·Valpha`，按符号得 `flag = 4a + 2b + c`，查表映射到 6 个大扇区（各 60°）。`Vref = 0` 时 `flag = 0`，按扇区 1 处理并自然输出零矢量。
2. **角度折算**：`theta_1 = (atan2(Vbeta, Valpha) - (sector-1)·60°) mod 60°`，把矢量折到扇区 1 的 0°~60° 内。
3. **调制比**：`k = √3·|Vref| / Udc`。线性调制区边界为 `k = 1`（对应内切圆 `Vref = Udc/√3`），超出后**硬钳位**到 1，此时伏秒平衡无解，避免出现负作用时间。
4. **小区域判断**：以扇区 1 的几何（V1=POO、V2=PPO、V7=PON、PNN、PPN）划出 6 个小三角形区域，用分界线 `ub = -√3·ua + √3/3·Udc`、`ub = √3·ua - √3/3·Udc`、`ub = √3/6·Udc` 判别。
5. **作用时间**：按 `region` 分 6 种情况计算 `T1 / T2 / T3`（相邻小矢量时间、中矢量时间、零矢量时间）。归一化系数 `2·k·Ts` 源自三电平小矢量在 αβ 坐标系中幅值为 `Udc/3`。
6. **数值兜底**：`T1/T2/T3` 逐项非负钳位；若总和 `> Ts` 则等比缩放到 `Ts`；最后逐项钳到 `[0, Ts]`。这一步只用于修正浮点误差，**不应作为常规过调制手段**。
7. **比较值映射**：按七段式对称排列换算成三个切换时刻 `c0/c1/c2`，再依 `sector` 分派到六路比较寄存器。

## ePWM 硬件配置约定

`SVPWM_PWM_INVERTED` 宏（默认 `0`）用于匹配 Action Qualifier 的极性：

| 取值 | 含义 |
| --- | --- |
| `0`（默认） | 上管 T1 在 `counter >= Cmp_up` 时导通（P 态居载波峰顶）；下管 T4 在 `counter <= Cmp_dn` 时导通（N 态居载波谷底） |
| `1` | 比较值越大脉宽越窄的相反配置 |

所需 ePWM 设置：

- **中央对齐**（up-down）计数模式
- 计数器范围 `0 → ARR → 0`
- `ARR = sw_period`，即半周期的计数峰值

## 已知限制

- **外区矢量序列未完成**：`Ta/Tb/Tc` 的 `0.25 / 0.5` 系数只在“内三角 region 1/2 + ePWM 半周期计数 ARR + 中央对齐 + 比较值越大脉宽越宽”的约定下成立。region 3~6（中三角、外三角）需要按各自矢量序列另行排布，当前实现直接套用了同一组系数，**输出正确性未经验证**。
- 线性调制区限制在 `k = 1`（内切圆），尚未实现过调制区的处理策略。
- 中性点电位平衡：算法本身不含中点平衡控制，需在上层加入冗余小矢量调节。
- 未涉及死区补偿与开关损耗优化。

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

## 移植提示

代码仅依赖 `<math.h>` 与 `<stdint.h>`。在 C2000 上编译时建议启用 FPU32 与 `--fp_mode=relaxed`，并确认 `atan2f`/`sinf`/`hypotf` 由 ROM 数学库或 RTS 库提供。若移植到定点 DSP，需重写第 3~7 步的浮点运算。
