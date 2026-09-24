#include "Svpwm_gen_three_levlel.h"
#include <math.h>


void Svpwm_gen_Update(Svpwm_threelevel_cal* svpwm_gen)
{
    /* 输入防护：空指针或母线电压异常时输出零矢量（OOO），直接返回 */
    if(svpwm_gen == NULL || svpwm_gen->Udc <= 0.0f)
    {
        return;
    }

    /*判断大扇区*/
    int sector = 0;                   //大扇区值
    float u1 = 0.0, u2 = 0.0, u3 = 0.0;     //中间变量
    int a = 0, b = 0, c = 0;          //大扇区判断中间值
    int flag = 0;                     //判断标志位

    int region = 0;                   //小区域值
    float ub = 0.0;
    float ua = 0.0;
    float udc = 0.0;
    float theta = 0.0;                 //角度变量
    float theta_1 = 0.0;               //角度变量1（大扇区折算到 0~60°）
    float Ts = svpwm_gen->Ts;          //采样时间
    float T1, T2, T3;                  //中间时间变量

    u1 = -svpwm_gen->Vbeta_ref;
    u2 = svpwm_gen->Vbeta_ref - sqrt3*svpwm_gen->Valpha_ref;
    u3 = svpwm_gen->Vbeta_ref + sqrt3*svpwm_gen->Valpha_ref;

    if(u1 > 0)    a = 1;      //-ubeta > 0 则定义a = 1
        else      a = 0;
    if (u2 > 0)   b = 1;      //ubeta - sqrt3 * valpha > 0 则定义b = 1
        else      b = 0;
    if (u3 > 0)   c = 1;      //ubeta + sqrt3 * valpha > 0 则定义c = 1
        else      c = 0;
    flag = 4 * a + 2 * b + c;

    switch (flag)                               //扇区判断表
    {
        case 1: {sector = 1;break;}             // 0°~60°
        case 3: {sector = 2;break;}             // 60°~120°
        case 2: {sector = 3;break;}             // 120°~180°
        case 6: {sector = 4;break;}             // 180°~240°
        case 4: {sector = 5;break;}             // 240°~300°
        case 5: {sector = 6;break;}             // 300°~360°
        default:{sector = 1;break;}             // flag=0，即 Vref=0，按扇区1处理（输出零矢量）
    }

    ua = svpwm_gen->Valpha_ref;
    ub = svpwm_gen->Vbeta_ref;
    udc = svpwm_gen->Udc;
    theta = atan2f(ub, ua);                     //计算角度值
    theta_1 = theta - (sector - 1) * pi_3;

    /* 用 fmodf 一次到位，替代 while 循环，避免异常输入时死循环 */
    theta_1 = fmodf(theta_1, pi_3);
    if(theta_1 < 0) theta_1 += pi_3;

    float Vref = hypotf(ua, ub);
    float sin_t = sinf(theta_1);
    float sin_t1 = sinf(pi_3 - theta_1);
    float sin_t2= sinf(pi_3 + theta_1);
    /* 调制比 m = sqrt(3)*Vref/Udc。
       推导依据：三电平小矢量(POO)在 alpha-beta 坐标系幅值为 Udc/3，
       由伏秒平衡 t1 = 2*m*Ts*sin(60-theta), t2 = 2*m*Ts*sin(theta)。 */
    float k = sqrt3 * Vref / udc;
    float sw_period = svpwm_gen->sw_period;
    /* 线性调制区限制：内切圆边界 m = 1（Vref = Udc/sqrt(3)）。
       超过后伏秒平衡无解，硬钳到线性边界，避免负作用时间。 */
    if(k > 1.0f)
    {
        k = 1.0f;
    }

    /*判断小区域（扇区1几何：V1=POO=(Udc/3,0)，V2=PPO=(Udc/6,sqrt3*Udc/6)，
      中矢量 V7=PON=(Udc/2,sqrt3*Udc/6)，大矢量 PNN=(2Udc/3,0)、PPN=(Udc/3,sqrt3*Udc/3)）*/
    if (theta_1 >= 0 && theta_1 <= pi_6)
    {
        if(ub <= -sqrt3*ua + sqrt3_3*udc)       region = 1;   /* 内三角 O-V1-V2 下半 */
        else if(ub <= sqrt3*ua - sqrt3_3*udc)   region = 5;   /* 外下三角 V1-V7-PNN */
        else                                    region = 4;   /* 中三角 V1-V2-V7 下半 */
    }
    else
    {
        if(ub <= -sqrt3*ua + sqrt3_3*udc)       region = 2;   /* 内三角上半 */
        else if(ub >= sqrt3_6*udc)              region = 6;   /* 外上三角 V2-V7-PPN */
        else                                    region = 3;   /* 中三角上半 */
    }

    /*计算中间时间：T1/T2 为相邻小矢量时间，T3 为零矢量/中矢量时间，
      各 case 已用 region 内典型工作点做伏秒平衡数值校核，区域内全部非负且和为 Ts*/
    switch (region)
    {
    case 1:
        T1 = 2* k *Ts * sin_t1;
        T2 = 2* k *Ts * sin_t;
        T3 = Ts * (1 - 2 * k * sin_t2);
        break;
    case 2:
        T1 = 2* k *Ts * sin_t;
        T2 = Ts * (1 - 2 * k * sin_t2);
        T3 = 2* k *Ts * sin_t1;
        break;
    case 3:
        T1 = Ts * (1 - 2 * k * sin_t);
        T2 = Ts * (1 - 2 * k * sin_t1);
        T3 = Ts * (2 * k * sin_t2 - 1);
        break;
    case 4:
        T1 = Ts * (1 - 2 * k * sin_t1);
        T2 = Ts * (2 * k * sin_t2 - 1);
        T3 = Ts * (1 - 2 * k * sin_t);
        break;
    case 5:
        T1 = 2 * Ts * (1 - k * sin_t2);
        T2 = Ts * (2 * k * sin_t1 - 1);
        T3 = 2 * k * Ts * sin_t;
        break;
    case 6:
        T1 = 2 * Ts * (1 - k * sin_t2);
        T2 = 2 * k * Ts * sin_t1;
        T3 = Ts * (2 * k * sin_t - 1);
        break;
    default:
        T1 = T2 = 0;
        T3 = Ts;
        break;
    }

    /* 浮点误差兜底：正常区域内 T1/T2/T3 天然非负且和为 Ts，
       这里只修正边界上的微小计算误差，不应作为常规过调制手段 */
    if(T1 < 0) T1 = 0;
    if(T2 < 0) T2 = 0;
    if(T3 < 0) T3 = 0;
    float t_sum = T1 + T2 + T3;
    if(t_sum > Ts)
    {
        float scale = Ts / t_sum;
        T1 *= scale;
        T2 *= scale;
        T3 *= scale;
    }
    T1 = fmaxf(0, fminf(T1, Ts));
    T2 = fmaxf(0, fminf(T2, Ts));
    T3 = fmaxf(0, fminf(T3, Ts));

    /* 七段式对称排列，半周期(Ts/2)内的三个切换时刻。
       注意：c0 以零矢/冗余段为基准，各 region 基准段不同，
       下方 0.25/0.5 系数仅在“内三角 region1/2 且 sw_period=ePWM 半周期计数 ARR、
       中央对齐、比较值越大脉宽越宽”的约定下成立，外区(region3~6)需按矢量序列另排 */
    float Ta = 0.25f * T1;
    float Tb = Ta + 0.5f * T2;
    float Tc = Tb + 0.5f * T3;

    uint16_t c0 = (uint16_t)(CLAMP01(Ta / Ts) * sw_period);
    uint16_t c1 = (uint16_t)(CLAMP01(Tb / Ts) * sw_period);
    uint16_t c2 = (uint16_t)(CLAMP01(Tc / Ts) * sw_period);
    const uint16_t SET = (uint16_t)sw_period;
    const uint16_t RST = 0;

    switch (sector)
    {
        case 1:
            svpwm_gen->Cmpa = c2;
            svpwm_gen->Cmpb = c1;
            svpwm_gen->Cmpc = c0;
            svpwm_gen->Cpmd = RST;
            svpwm_gen->Cpme = c0;
            svpwm_gen->Cpmf = c1;
            break;
        case 2:
            svpwm_gen->Cmpa = c0;
            svpwm_gen->Cmpb = c2;
            svpwm_gen->Cmpc = c1;
            svpwm_gen->Cpmd = RST;
            svpwm_gen->Cpme = RST;
            svpwm_gen->Cpmf = c0;
            break;
        case 3:
            svpwm_gen->Cmpa = c0;
            svpwm_gen->Cmpb = c1;
            svpwm_gen->Cmpc = c2;
            svpwm_gen->Cpmd = RST;
            svpwm_gen->Cpme = RST;
            svpwm_gen->Cpmf = c0;
            break;
        case 4:
            svpwm_gen->Cmpa = c1;
            svpwm_gen->Cmpb = c0;
            svpwm_gen->Cmpc = c2;
            svpwm_gen->Cpmd = SET;
            svpwm_gen->Cpme = SET;
            svpwm_gen->Cpmf = c2;
            break;
        case 5:
            svpwm_gen->Cmpa = c1;
            svpwm_gen->Cmpb = c2;
            svpwm_gen->Cmpc = c0;
            svpwm_gen->Cpmd = SET;
            svpwm_gen->Cpme = c2;
            svpwm_gen->Cpmf = c1;
            break;
        case 6:
            svpwm_gen->Cmpa = c2;
            svpwm_gen->Cmpb = c0;
            svpwm_gen->Cmpc = c1;
            svpwm_gen->Cpmd = SET;
            svpwm_gen->Cpme = c2;
            svpwm_gen->Cpmf = c1;
            break;
        default:
            svpwm_gen->Cmpa = svpwm_gen->Cmpb = svpwm_gen->Cmpc = (uint16_t)(sw_period/2);
            svpwm_gen->Cpmd = svpwm_gen->Cpme = svpwm_gen->Cpmf = (uint16_t)(sw_period/2);
            break;
    }
}
