
# -*- coding: utf-8 -*-
"""
三电平 SVPWM 发波可视化 —— 输出羊角波
=====================================
与 Svpwm_gen_three_levlel.c 中的 Svpwm_gen_Update() 逐位一致的 Python 移植，
仿真一个完整基波周期，输出：
    svpwm_horn.png       羊角波（三电平相电压对直流中点 + 线电压 + 放大脉冲）
    svpwm_dashboard.png  载波/比较值、桥臂电压、线电压、调制波、矢量轨迹、频谱

用法:
    python svpwm_three_level_viz.py
    python svpwm_three_level_viz.py --m 0.5 --f1 50 --fsw 8000 --udc 600 --arr 1000
    python svpwm_three_level_viz.py --npulse 24      # 羊角波每基波周期的脉冲数
"""

import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import rcParams

rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
rcParams["axes.unicode_minus"] = False

SQRT3 = np.sqrt(3.0)


# =============================================================================
# 1. C 代码的逐位移植
# =============================================================================
def svpwm_update(alpha, beta, udc, arr):
    """对应 Svpwm_gen_Update()，返回 (Cmpa, Cmpb, Cmpc, Cpmd, Cpme, Cpmf)。

    硬件电平关系：P 当 cnt > Cmp，N 当 cnt < Cpm，其余为 O
    =>  P 占空比 = 1 - Cmp/ARR，N 占空比 = Cpm/ARR
    =>  输出电平 p = P - N = 1 - (Cmp + Cpm)/ARR
    """
    hi = int(arr)
    zero = (hi, hi, hi, 0, 0, 0)

    if udc <= 0.0:
        return zero

    vref = float(np.hypot(alpha, beta))
    if vref <= 1e-6 * udc:                      # 零指令保护 -> OOO
        return zero

    # 反 Clarke，直接得到三相归一化电平（单位 Udc/2），不需要三角函数
    pa = (2.0 * alpha) / udc
    pb = (-alpha + SQRT3 * beta) / udc
    pc = (-alpha - SQRT3 * beta) / udc

    k = SQRT3 * vref / udc                      # 调制比，线性区 k <= 1
    if k > 1.0:
        s = 1.0 / k
        pa *= s
        pb *= s
        pc *= s

    # 零序注入（三电平 SVPWM 的等效载波实现）
    poff = 0.5 * (max(pa, pb, pc) + min(pa, pb, pc))
    pa -= poff
    pb -= poff
    pc -= poff

    cmps, cpms = [], []
    for p in (pa, pb, pc):
        P = p if p > 0.0 else 0.0
        N = -p if p < 0.0 else 0.0
        cmps.append(int(min(max(1.0 - P, 0.0), 1.0) * arr + 0.5))
        cpms.append(int(min(max(N, 0.0), 1.0) * arr + 0.5))
    return cmps[0], cmps[1], cmps[2], cpms[0], cpms[1], cpms[2]


def ref_phase(alpha, beta, udc):
    """连续域三相归一化电平：注入前 / 注入后 / 零序分量。"""
    pa = (2.0 * alpha) / udc
    pb = (-alpha + SQRT3 * beta) / udc
    pc = (-alpha - SQRT3 * beta) / udc
    poff = 0.5 * (np.maximum.reduce([pa, pb, pc]) + np.minimum.reduce([pa, pb, pc]))
    return (pa, pb, pc), (pa - poff, pb - poff, pc - poff), poff


# =============================================================================
# 2. 仿真一个完整基波周期
# =============================================================================
def simulate(m=0.8, f1=50.0, fsw=10e3, udc=600.0, arr=1000, spp=400):
    nper = int(round(fsw / f1))          # 一个基波周期内的开关周期数
    tsw = 1.0 / fsw
    n = nper * spp
    dt = tsw / spp

    t = np.arange(n) * dt
    kidx = np.minimum(np.floor(t / tsw).astype(np.int64), nper - 1)
    tau = t / tsw - kidx
    cnt = (1.0 - np.abs(2.0 * tau - 1.0)) * arr          # 三角载波 0->ARR->0

    # 每个开关周期更新一次比较值（对应 DSP 中断里调用一次）
    theta = 2.0 * np.pi * np.arange(nper) / nper
    vref = m * udc / SQRT3
    al, be = vref * np.cos(theta), vref * np.sin(theta)
    C = np.array([svpwm_update(a, b, udc, arr) for a, b in zip(al, be)], dtype=np.int64)

    cmpa, cmpb, cmpc = C[kidx, 0], C[kidx, 1], C[kidx, 2]
    cpmd, cpme, cpmf = C[kidx, 3], C[kidx, 4], C[kidx, 5]

    def level(cmp, cpm):
        return np.where(cnt > cmp, 1, np.where(cnt < cpm, -1, 0))

    la, lb, lc = level(cmpa, cpmd), level(cmpb, cpme), level(cmpc, cpmf)
    va, vb, vc = la * udc / 2.0, lb * udc / 2.0, lc * udc / 2.0

    v_al = (2.0 / 3.0) * (va - 0.5 * vb - 0.5 * vc)
    v_be = (2.0 / 3.0) * (SQRT3 / 2.0) * (vb - vc)

    return dict(t=t, dt=dt, cnt=cnt, kidx=kidx, tsw=tsw, nper=nper,
                C=C, theta=theta, vref=vref,
                cmpa=cmpa, cmpb=cmpb, cmpc=cmpc,
                cpmd=cpmd, cpme=cpme, cpmf=cpmf,
                la=la, lb=lb, lc=lc,
                va=va, vb=vb, vc=vc, vab=va - vb,
                v_al=v_al, v_be=v_be)


def spectrum(sig, dt, fmax):
    n = len(sig)
    amp = 2.0 * np.abs(np.fft.rfft(sig)) / n
    freqs = np.fft.rfftfreq(n, dt)
    msk = freqs <= fmax
    return freqs[msk], amp[msk]


# =============================================================================
# 3. 羊角波主图
# =============================================================================
def plot_horn(m, f1, udc, arr, npulse, out="svpwm_horn.png"):
    """三电平相电压（羊角波）。npulse 控制每基波周期的脉冲数，越小阶梯越清晰。"""
    s = simulate(m, f1, f1 * npulse, udc, arr, spp=800)
    t_ms = s["t"] * 1e3
    T1 = 1e3 / f1
    env = s["vref"] * np.cos(2 * np.pi * f1 * s["t"])

    fig = plt.figure(figsize=(15, 9.5))
    gs = fig.add_gridspec(3, 2, height_ratios=[2.1, 1.4, 1.2], hspace=0.45, wspace=0.16)
    fig.suptitle("三电平相电压 —— 羊角波   |   m=%.2f  f1=%.0fHz  fsw=%.0fHz  Udc=%.0fV"
                 % (m, f1, f1 * npulse, udc), fontsize=15, fontweight="bold")

    # ---- 上：完整基波周期，相电压对直流中点 ----
    ax = fig.add_subplot(gs[0, :])
    ax.fill_between(t_ms, 0, s["va"], step=None, color="tab:red", alpha=0.18, lw=0)
    ax.plot(t_ms, s["va"], lw=0.8, color="tab:red", label="v_aO（P/O/N 三电平脉冲）")
    ax.plot(t_ms, env, "k--", lw=1.3, label="基波包络 ±Vref·cosφ")
    ax.plot(t_ms, -env, "k--", lw=1.3)
    for lv in (udc / 2, 0, -udc / 2):
        ax.axhline(lv, color="0.75", lw=0.8, ls=":")
    ax.set_yticks([-udc / 2, 0, udc / 2])
    ax.set_yticklabels(["N  −Udc/2", "O  0", "P  +Udc/2"])
    ax.annotate("中间平：峰值处脉宽最大", xy=(T1 * 0.055, udc * 0.50), xytext=(T1 * 0.10, udc * 0.63),
                fontsize=10, color="0.3",
                arrowprops=dict(arrowstyle="->", color="0.45", lw=1.0))
    ax.annotate("两端尖：过零点脉宽→0", xy=(T1 * 0.252, udc * 0.10), xytext=(T1 * 0.30, udc * 0.63),
                fontsize=10, color="0.3",
                arrowprops=dict(arrowstyle="->", color="0.45", lw=1.0))
    ax.set_ylabel("v_aO / V")
    ax.set_xlim(0, T1)
    ax.set_ylim(-udc * 0.72, udc * 0.72)
    ax.grid(alpha=0.3)
    ax.legend(loc="lower right", fontsize=10)
    ax.set_title("① 一个基波周期：阶梯脉冲的包络呈羊角形（两端尖、中间平）", fontsize=12)

    # ---- 中左：跨过零点的 4 个开关周期，看 P→O→N ----
    ax = fig.add_subplot(gs[1, 0])
    k0 = max(int(s["nper"] * 0.22), 1)
    sel = (s["kidx"] >= k0) & (s["kidx"] < k0 + 4)
    tt = (t_ms[sel] - t_ms[sel][0]) * 1e3
    ax.step(tt, s["va"][sel], where="post", lw=1.4, color="tab:red", label="v_aO")
    ax.axhline(0, color="0.7", lw=0.8)
    ax.axhline(udc / 2, color="0.85", lw=0.8, ls=":")
    ax.axhline(-udc / 2, color="0.85", lw=0.8, ls=":")
    ax.set_ylim(-udc * 0.68, udc * 0.68)
    ax.set_xlabel("时间 / μs"); ax.set_ylabel("v_aO / V")
    ax.grid(alpha=0.3); ax.legend(fontsize=9, loc="upper right")
    ax.set_title("② 跨零点的 4 个开关周期：P → O → N", fontsize=11)

    # ---- 中右：线电压五电平（整周期）----
    ax = fig.add_subplot(gs[1, 1])
    ax.plot(t_ms, s["vab"], lw=0.7, color="tab:purple")
    ax.axhline(0, color="0.7", lw=0.8)
    ax.set_xlim(0, T1)
    ax.set_xlabel("时间 / ms"); ax.set_ylabel("v_ab / V")
    ax.grid(alpha=0.3)
    ax.set_title("③ 线电压 v_ab：五电平（±Udc, ±Udc/2, 0）", fontsize=11)

    # ---- 下：三相错位 ----
    ax = fig.add_subplot(gs[2, :])
    off = 1.15 * udc
    for v, c, lb, k in ((s["va"], "tab:red", "a", 0), (s["vb"], "tab:green", "b", 1),
                        (s["vc"], "tab:blue", "c", 2)):
        ax.plot(t_ms, v - k * off, lw=0.7, color=c, label="v_%sO" % lb)
        ax.axhline(-k * off, color="0.88", lw=0.6)
    ax.set_xlim(0, T1)
    ax.set_ylim(-2 * off - 0.65 * udc, 0.65 * udc)
    ax.set_xlabel("时间 / ms"); ax.set_ylabel("三相（错位 1.15Udc）")
    ax.grid(alpha=0.3); ax.legend(loc="upper right", fontsize=9, ncol=3)
    ax.set_title("④ 三相桥臂电压，互差 120°", fontsize=11)

    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return s, out


# =============================================================================
# 4. 六联图
# =============================================================================
def plot_dashboard(s, m, f1, fsw, udc, arr, out="svpwm_dashboard.png"):
    fig, axs = plt.subplots(3, 2, figsize=(17, 11))
    fig.suptitle("三电平 SVPWM 发波波形  |  m=%.2f  f1=%.0fHz  fsw=%.1fkHz  Udc=%.0fV  ARR=%d"
                 % (m, f1, fsw / 1e3, udc, arr), fontsize=14, fontweight="bold")

    t_ms = s["t"] * 1e3
    T1 = 1e3 / f1

    # ① 载波与六路比较值
    ax = axs[0, 0]
    k0 = max(int(s["nper"] * 0.15), 1)
    sel = (s["kidx"] >= k0) & (s["kidx"] < k0 + 3)
    tt = (t_ms[sel] - t_ms[sel][0]) * 1e3
    ax.plot(tt, s["cnt"][sel], color="0.55", lw=1.0, label="三角载波 cnt")
    xlab = tt[-1] * 1.012
    for v, c, lb in ((s["cmpa"][sel][0], "tab:red", "Cmpa"),
                     (s["cmpb"][sel][0], "tab:green", "Cmpb"),
                     (s["cmpc"][sel][0], "tab:blue", "Cmpc")):
        ax.axhline(v, color=c, lw=1.4, ls="--", alpha=0.9)
        ax.text(xlab, v, lb, color=c, fontsize=8, va="bottom", ha="left")
    for v, c, lb in ((s["cpmd"][sel][0], "tab:red", "Cpmd"),
                     (s["cpme"][sel][0], "tab:green", "Cpme"),
                     (s["cpmf"][sel][0], "tab:blue", "Cpmf")):
        ax.axhline(v, color=c, lw=1.2, ls=":", alpha=0.8)
        ax.text(xlab, v, lb, color=c, fontsize=8, va="top", ha="left")
    ax.set_xlim(0, tt[-1] * 1.09)
    ax.set_title("① 载波与六路比较值（放大 3 个开关周期）", fontsize=11)
    ax.set_xlabel("时间 / μs"); ax.set_ylabel("计数值")
    ax.set_ylim(-60, arr * 1.12); ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)

    # ② 三相桥臂电压
    ax = axs[0, 1]
    win = t_ms <= T1
    off = 1.2 * udc
    for v, c, lb, k in ((s["va"], "tab:red", "v_aO", 0), (s["vb"], "tab:green", "v_bO", 1),
                        (s["vc"], "tab:blue", "v_cO", 2)):
        ax.plot(t_ms[win], v[win] - k * off, lw=0.6, color=c, label=lb)
        ax.axhline(-k * off, color="0.85", lw=0.6)
    ax.set_title("② 三相桥臂电压（对直流中点，三电平）", fontsize=11)
    ax.set_xlabel("时间 / ms"); ax.set_ylabel("电压 / V（错位 1.2Udc）")
    ax.grid(alpha=0.3); ax.legend(loc="upper right", fontsize=8, ncol=3)

    # ③ 线电压
    ax = axs[1, 0]
    ax.plot(t_ms[win], s["vab"][win], lw=0.6, color="tab:purple")
    ax.axhline(0, color="0.7", lw=0.8)
    ax.set_title("③ 线电压 v_ab（五电平）", fontsize=11)
    ax.set_xlabel("时间 / ms"); ax.set_ylabel("电压 / V")
    ax.grid(alpha=0.3)

    # ④ 调制波
    ax = axs[1, 1]
    tt2 = np.linspace(0, 2 * np.pi, 2000)
    vref = s["vref"]
    (pa, pb, pc), (qa, qb, qc), poff = ref_phase(vref * np.cos(tt2), vref * np.sin(tt2), udc)
    xms = tt2 / (2 * np.pi) * 1e3 / f1
    ax.plot(xms, pa, ls="--", lw=1.0, color="0.6", label="正弦指令 a")
    ax.plot(xms, qa, lw=1.8, color="tab:red", label="注入后 a")
    ax.plot(xms, qb, lw=1.8, color="tab:green", label="注入后 b")
    ax.plot(xms, qc, lw=1.8, color="tab:blue", label="注入后 c")
    ax.plot(xms, -poff, lw=1.2, ls="-.", color="k", label="零序分量 -poff")
    ax.axhline(1, color="0.5", lw=0.8, ls=":")
    ax.axhline(-1, color="0.5", lw=0.8, ls=":")
    ax.set_title("④ 调制波：正弦 → 零序注入后（马鞍波）", fontsize=11)
    ax.set_xlabel("时间 / ms"); ax.set_ylabel("归一化电平 (Udc/2)")
    ax.grid(alpha=0.3); ax.legend(loc="lower right", fontsize=7, ncol=2)

    # ⑤ αβ 轨迹
    ax = axs[2, 0]
    ax.plot(s["v_al"][win], s["v_be"][win], lw=0.35, color="0.72", label="整周期轨迹")
    st = np.array([[a, b, c] for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)], float)
    sa = (2.0 / 3.0) * (st[:, 0] - 0.5 * st[:, 1] - 0.5 * st[:, 2]) * (udc / 2)
    sb = (2.0 / 3.0) * (SQRT3 / 2.0) * (st[:, 1] - st[:, 2]) * (udc / 2)
    ax.scatter(sa, sb, s=22, c="tab:blue", zorder=3, label="19 个基本矢量")
    ph = np.linspace(0, 2 * np.pi, 400)
    ax.plot(vref * np.cos(ph), vref * np.sin(ph), "r--", lw=1.0, label="指令圆")
    ax.set_title("⑤ αβ 平面轨迹与三电平基本矢量", fontsize=11)
    ax.set_xlabel("Vα / V"); ax.set_ylabel("Vβ / V")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=8, loc="upper right")

    # ⑥ 频谱
    ax = axs[2, 1]
    fmax = 5 * f1
    f1_, a1 = spectrum(s["va"], s["dt"], fmax)
    f2_, a2 = spectrum(s["vab"], s["dt"], fmax)
    ax.plot(f1_, a1, lw=1.4, marker="o", ms=3, color="tab:red", label="v_aO（含零序 3 次）")
    ax.plot(f2_, a2, lw=1.4, marker="s", ms=3, color="tab:purple", label="v_ab（线电压）")
    ax.set_xlim(0, fmax)
    ax.set_title("⑥ 低频频谱（0~%.0fHz）：3 次谐波只存在于相电压" % fmax, fontsize=11)
    ax.set_xlabel("频率 / Hz"); ax.set_ylabel("幅值 / V")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out, dpi=150)
    plt.close(fig)
    return out


# =============================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--m", type=float, default=0.8, help="调制比 sqrt3*Vref/Udc，<=1")
    ap.add_argument("--f1", type=float, default=50.0, help="基波频率 Hz")
    ap.add_argument("--fsw", type=float, default=10e3, help="开关频率 Hz")
    ap.add_argument("--udc", type=float, default=600.0, help="母线电压 V")
    ap.add_argument("--arr", type=int, default=1000, help="ePWM 周期寄存器 ARR")
    ap.add_argument("--npulse", type=int, default=40, help="羊角波每基波周期的脉冲数")
    a = ap.parse_args()

    s = simulate(a.m, a.f1, a.fsw, a.udc, a.arr, 400)

    f_, amp = spectrum(s["va"], s["dt"], 10 * a.f1)
    i1 = int(np.argmin(np.abs(f_ - a.f1)))
    i3 = int(np.argmin(np.abs(f_ - 3 * a.f1)))
    fa, aa = spectrum(s["vab"], s["dt"], 10 * a.f1)
    j1 = int(np.argmin(np.abs(fa - a.f1)))
    j3 = int(np.argmin(np.abs(fa - 3 * a.f1)))

    print("=== 校验 ===")
    print("调制比 m                 : %.3f" % a.m)
    print("指令基波幅值             : %.2f V" % s["vref"])
    print("v_aO 基波幅值 (FFT)      : %.2f V  (误差 %+.3f%%)"
          % (amp[i1], 100 * (amp[i1] - s["vref"]) / s["vref"]))
    print("v_aO 3 次谐波            : %.2f V  <- 零序注入，相电压固有" % amp[i3])
    print("v_ab 基波幅值 (FFT)      : %.2f V  (理论 %.2f V)"
          % (aa[j1], s["vref"] * SQRT3))
    print("v_ab 3 次谐波            : %.2f V  <- 线电压应≈0" % aa[j3])
    print("比较值 (θ=0°)            : %s" % (tuple(int(x) for x in s["C"][0]),))

    _, p1 = plot_horn(a.m, a.f1, a.udc, a.arr, a.npulse)
    p2 = plot_dashboard(s, a.m, a.f1, a.fsw, a.udc, a.arr)
    print("\n已输出: %s\n        %s" % (p1, p2))


if __name__ == "__main__":
    main()
