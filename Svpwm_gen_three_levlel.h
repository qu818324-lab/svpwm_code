#ifndef SVPWM_GEN_THREE_LEVLEL_H
#define SVPWM_GEN_THREE_LEVLEL_H

#include <math.h>
#include <stdint.h>

#define pi 3.141592653589793238462
#define rad_to_deg(x) (x * 180.0 / pi)
#define deg_to_rad(x) (x * pi / 180.0)
#define sqrt3 1.73205080756887729352
#define sqrt2 1.414213562373095048222
#define sqrt3_2 0.866025403784438646763   /* sqrt(3)/2 */
#define sqrt3_3 0.577350269189625764509   /* 1/sqrt(3), 小矢量区域分界线常数 */
#define sqrt3_4 0.433012701892219323381   /* sqrt(3)/4 */
#define sqrt3_6 0.288675134594812882255   /* sqrt(3)/6, 中矢量 V7 的 beta 坐标 */
#define pi_3 1.04719755119659774716
#define pi_6 0.5235987755982988626
#define CLAMP01(x) (fmaxf(0.0f, fminf(1.0f, x)))

/* ePWM 比较器极性开关。
   硬件约定（F28379D，中央对齐，计数器 0->ARR->0，ARR = sw_period 为半周期峰值）：
   SVPWM_PWM_INVERTED = 0（默认）：
     上管 T1 在 counter >= Cmp_up 时导通（P 态居载波峰顶），Cmp_up = ARR*(1-dP)；
     下管 T4 在 counter <= Cmp_dn 时导通（N 态居载波谷底），Cmp_dn = ARR*dN。
   若你的 Action Qualifier 配置成相反电平（比较值越大脉宽越窄），把本宏改为 1。 */
#define SVPWM_PWM_INVERTED 0


typedef struct
{
    float Valpha_ref;
    float Vbeta_ref;

    float Udc;
    float Ts;

    uint16_t Cmpa;   /* A 相上管 T1 比较值 */
    uint16_t Cmpb;   /* B 相上管 T1 比较值 */
    uint16_t Cmpc;   /* C 相上管 T1 比较值 */
    uint16_t Cpmd;   /* A 相下管 T4 比较值 */
    uint16_t Cpme;   /* B 相下管 T4 比较值 */
    uint16_t Cpmf;   /* C 相下管 T4 比较值 */
    float sw_period;  /* ePWM 周期计数峰值 ARR（中央对齐=半周期计数值） */
} Svpwm_threelevel_cal;




void Svpwm_gen_Update(Svpwm_threelevel_cal* svpwm_gen);


#endif /* SVPWM_GEN_THREE_LEVLEL_H */
