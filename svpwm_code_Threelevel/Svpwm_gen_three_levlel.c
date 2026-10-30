/*============================================================================
 *  Svpwm_gen_three_level.c  —— 三电平 NPC 逆变器 SVPWM
 *
 *  输入：Valpha_ref / Vbeta_ref  参考电压矢量（α-β 坐标，单位 V）
 *        Udc                    母线电压（V）
 *        Ts                     PWM 周期（s）
 *        sw_period              定时器 ARR（一个 PWM 周期的计数值）
 *  输出：Cmpa ~ Cmpf            六路比较值
 *        Cmpa/Cpmb = A 相 S1 / S2
 *        Cmpc/Cpmd = B 相 S1 / S2
 *        Cmpe/Cpmf = C 相 S1 / S2
 *        （S1 通 = P 态；S2 通 = P 或 O 态；S3=!S1、S4=!S2 由硬件互补驱动）
 *
 *  算法流程（7 步）：
 *    1. 大扇区判断（60° 一个扇区，共 6 个）
 *    2. 角度折算到 0~60°（把矢量旋转 -(sector-1)*60°）
 *    3. 算调制比 k = √3·Vref/Udc，并限幅到 1
 *    4. 小区域判断（每个大扇区再分 6 个小三角）
 *    5. 按小区域算三个矢量的作用时间 T1/T2/T3
 *    6. 伏秒平衡反推三相归一化极电压 s = 2v/Udc
 *    7. s 换算成每相两路比较值
 *
 *  基础矢量（折算到扇区 1，单位 Udc）：
 *    小矢量  POO = (1/3, 0)          PPO = (1/6, √3/6)
 *    中矢量  PON = (1/2, √3/6)
 *    大矢量  PNN = (2/3, 0)          PPN = (1/3, √3/3)
 *    零矢量  OOO / PPP / NNN
 *===========================================================================*/
#include "Svpwm_gen_three_levlel.h"
#include <math.h>


void Svpwm_gen_Update(Svpwm_threelevel_cal* svpwm_gen)
{
    /*------------------------------------------------------------------------
     * 输入防护
     *   Udc 异常时不能做除法，直接强制输出零矢量 OOO（三相全接中点）。
     *   OOO 的驱动组合：S1 全关（比较值 0）、S2 全开（比较值 ARR）。
     *   注意不能写成 S1 全开 + S2 全关 —— 那会让上管 S1 与下管 S4 同时导通，
     *   造成直流母线直通。
     *----------------------------------------------------------------------*/
    if(svpwm_gen == NULL || svpwm_gen->Udc <= 0.0f)
    {
        if (svpwm_gen != NULL)
        {
            uint16_t hi = (uint16_t)svpwm_gen->sw_period - 1;   /* 留 1 个计数余量，避免 100% 占空比 */
            svpwm_gen->Cmpa = 0;   svpwm_gen->Cmpb = hi;   svpwm_gen->Cmpc = 0;
            svpwm_gen->Cpmd = hi;  svpwm_gen->Cpme = 0;  svpwm_gen->Cpmf = hi;
        }
        return;
    }

    /*判断大扇区*/
    int sector = 0;                   //大扇区值，1~6
    float u1 = 0.0, u2 = 0.0, u3 = 0.0;     //中间变量
    int a = 0, b = 0, c = 0;          //大扇区判断中间值
    int flag = 0;                     //判断标志位

    int region = 0;                   //小区域值，1~6
    float ub = 0.0;
    float ua = 0.0;
    float udc = 0.0;
    float theta = 0.0;                 //角度变量
    float theta_1 = 0.0;               //角度变量1（大扇区折算到 0~60°）
    float Ts = svpwm_gen->Ts;          //采样时间
    float T1, T2, T3;                  //中间时间变量（三个矢量的作用时间）

    /*------------------------------------------------------------------------
     * 第 1 步：大扇区判断
     *   用三个判据的符号组合出扇区号，避免用 atan2 + 查表。
     *   u1 = -Vβ                ：Vβ < 0 时为 1
     *   u2 =  Vβ - √3·Vα        ：过 60° 分界线
     *   u3 =  Vβ + √3·Vα        ：过 120° 分界线
     *   三者符号组合成 flag = 4a+2b+c，恰好对应 6 个扇区。
     *----------------------------------------------------------------------*/
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

    /*------------------------------------------------------------------------
     * 第 2 步：把角度折算到 0~60°
     *
     *   原始角度 theta ∈ (-π, π]，减去 (sector-1)·60° 就落到本扇区的局部角度。
     *   fmodf 负责把负角度也规整到 [0, 60°)，避免用 while 循环。
     *
     *   【边界补丁】theta 正好等于 60°/120°/180° 时，fmodf(π/3, π/3) 返回 0，
     *   会把 60° 算成 0°，导致该扇区所有 sin 全错。下面这行专门兜住这种情况：
     *   只有当"折算前的原始角度确实约等于整数倍 60°"、而 fmodf 又给出了 0 时，
     *   才把 theta_1 修正回 60°。
     *----------------------------------------------------------------------*/
    theta = atan2f(ub, ua);                     //计算角度值
    theta_1 = theta - (sector - 1) * pi_3;

    theta_1 = fmodf(theta_1, pi_3);
    if(theta_1 < 0) theta_1 += pi_3;
    if(theta_1 < 1e-5f && fabsf(theta - (sector - 1) * pi_3) > 1e-3f)
        theta_1 = pi_3;                         //兜住扇区上边界的 fmod 退化

    /*------------------------------------------------------------------------
     * 第 3 步：调制比
     *
     *   m = √3·Vref/Udc
     *   推导：三电平小矢量 POO 的幅值是 Udc/3，由伏秒平衡
     *         T1 = 2·m·Ts·sin(60°-θ)、T2 = 2·m·Ts·sinθ，
     *         反推得到 Vref = m·Udc/√3，即 m = √3·Vref/Udc。
     *
     *   线性区上限 m = 1，对应 Vref = Udc/√3（大矢量六边形的内切圆）。
     *   超过后伏秒平衡无解、会算出负的作用时间，所以硬钳到 1。
     *------------------------------------------------------------------------*/
    float Vref = hypotf(ua, ub);
    float sin_t = sinf(theta_1);                // sinθ
    float sin_t1 = sinf(pi_3 - theta_1);        // sin(60°-θ)
    float sin_t2= sinf(pi_3 + theta_1);         // sin(60°+θ)

    float k = sqrt3 * Vref / udc;
                                                //（原 float sw_period = ... 已删：后面直接用 arr，未使用）

    if(k > 1.0f)
    {
        k = 1.0f;
    }
    Vref = k * udc / sqrt3;                     //限幅后的矢量幅值
    ua = Vref * cosf(theta_1);                  //折算坐标（限幅后）的 α 分量
    ub = Vref * sin_t;                          //折算坐标（限幅后）的 β 分量
                                                //sin_t 就是 sinf(theta_1)，不用重算

    /*------------------------------------------------------------------------
     * 第 4 步：小区域判断
     *
     *   必须用"折算坐标 ua/ub"来判，不能用原始的 Valpha/Vbeta ——
     *   下面这几条边界线都是扇区 1 局部坐标系里的。
     *
     *   扇区 1 内的几何（单位 Udc）：
     *     V1=POO=(1/3,0)      V2=PPO=(1/6,√3/6)     V7=PON=(1/2,√3/6)
     *     PNN=(2/3,0)         PPN=(1/3,√3/3)
     *
     *   三条分界线：
     *     线 V1V2： ub = -√3·ua + √3/3·Udc   → 内三角与外部的分界
     *     线 V1V7： ub =  √3·ua - √3/3·Udc   → 中三角与外下三角的分界
     *     线 V2V7： ub =  √3/6·Udc           → 中三角与外上三角的分界（水平线）
     *
     *   6 个区域的几何形状：
     *     region 1/2：内三角 O-V1-V2 的下半/上半（θ≤30° / θ>30°）
     *     region 4/3：中三角 V1-V2-V7 的下半/上半
     *     region 5/6：外三角 V1-V7-PNN 与 V2-V7-PPN
     *----------------------------------------------------------------------*/
    if (theta_1 >= 0 && theta_1 <= pi_6)        // θ ∈ [0,30°]：下半区
    {
        if(ub <= -sqrt3*ua + sqrt3_3*udc)       region = 1;   /* 内三角 O-V1-V2 下半 */
        else if(ub <= sqrt3*ua - sqrt3_3*udc)   region = 5;   /* 外下三角 V1-V7-PNN */
        else                                    region = 4;   /* 中三角 V1-V2-V7 下半 */
    }
    else                                        // θ ∈ (30°,60°]：上半区
    {
        if(ub <= -sqrt3*ua + sqrt3_3*udc)       region = 2;   /* 内三角上半 */
        else if(ub >= sqrt3_6*udc)              region = 6;   /* 外上三角 V2-V7-PPN */
        else                                    region = 3;   /* 中三角上半 */
    }

    /*------------------------------------------------------------------------
     * 第 5 步：三个矢量的作用时间
     *
     *   每个小区域用 3 个基本矢量做伏秒平衡，T1/T2/T3 就是这三个矢量的时间，
     *   三者之和恒等于 Ts。各 case 里 T1/T2/T3 与矢量的对应关系：
     *
     *     region 1： POO , PPO , OOO
     *     region 2： PPO , OOO , POO
     *     region 3： PPO , PON , POO
     *     region 4： POO , PPO , PON
     *     region 5： POO , PNN , PON
     *     region 6： PPO , PON , PPN
     *
     *   注意 region 3 和 region 4 的排法不一样（分别对应 MATLAB 里的 n=4 和 n=3），
     *   两者虽然时间公式看起来相似，但"哪个时间是哪个矢量"是反的，不能混。
     *----------------------------------------------------------------------*/
    switch (region)
    {
    case 1:                                     // POO, PPO, OOO
        T1 = 2* k *Ts * sin_t1;                 // T_POO
        T2 = 2* k *Ts * sin_t;                  // T_PPO
        T3 = Ts * (1 - 2 * k * sin_t2);         // T_OOO（零矢量）
        break;
    case 2:                                     // PPO, OOO, POO
        T1 = 2* k *Ts * sin_t;                  // T_PPO
        T2 = Ts * (1 - 2 * k * sin_t2);         // T_OOO
        T3 = 2* k *Ts * sin_t1;                 // T_POO
        break;
    case 3:                                     // PPO, PON, POO（中三角上半）
        T1 = Ts * (1 - 2 * k * sin_t1);         // T_PPO
        T2 = Ts * (2 * k * sin_t2 - 1);         // T_PON（中矢量）
        T3 = Ts * (1 - 2 * k * sin_t);          // T_POO
        break;
    case 4:                                     // POO, PPO, PON（中三角下半）
        T1 = Ts * (1 - 2 * k * sin_t);          // T_POO
        T2 = Ts * (1 - 2 * k * sin_t1);         // T_PPO
        T3 = Ts * (2 * k * sin_t2 - 1);         // T_PON（中矢量）
        break;
    case 5:                                     // POO, PNN, PON（外下三角）
        T1 = 2 * Ts * (1 - k * sin_t2);         // T_POO
        T2 = Ts * (2 * k * sin_t1 - 1);         // T_PNN（大矢量）
        T3 = 2 * k * Ts * sin_t;                // T_PON（中矢量）
        break;
    case 6:                                     // PPO, PON, PPN（外上三角）
        T1 = 2 * Ts * (1 - k * sin_t2);         // T_PPO
        T2 = 2 * k * Ts * sin_t1;               // T_PON（中矢量）
        T3 = Ts * (2 * k * sin_t - 1);          // T_PPN（大矢量）
        break;
    default:
        T1 = T2 = 0;
        T3 = Ts;
        break;
    }

    /*------------------------------------------------------------------------
     * 浮点兜底
     *   正常情况下三个时间天然非负、且和恰为 Ts。
     *   这里只处理边界上的微小负值和舍入误差，不应该当成过调制手段用
     *   （真正的过调制已经在第 3 步把 k 钳到 1 了）。
     *----------------------------------------------------------------------*/
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

    /*------------------------------------------------------------------------
     * 第 6 步：由作用时间反推三相归一化极电压
     *
     *   对每一相做伏秒平衡：
     *       s_x = ( Σ Ti · L_i(x) ) / Ts ,   L ∈ {+1(P), 0(O), -1(N)}
     *   其中 s_x = 2·v_x/Udc，即极电压相对 Udc/2 的归一化值。
     *
     *   折算坐标系下三相依次称为 lead（超前相）/ mid（中间相）/ lag（滞后相）。
     *   例如 region 1 的三个矢量是 POO、PPO、OOO：
     *       lead 相：POO 时=P、PPO 时=P、OOO 时=O → s_lead = (T1+T2)/Ts
     *       mid  相：POO 时=O、PPO 时=P、OOO 时=O → s_mid  =  T2/Ts
     *       lag  相：三个矢量都是 O                → s_lag  =  0
     *
     *   region 3~6 里 lead 相全程为 P（s_lead 恒为 1），这是三电平高调制区
     *   的正常现象——超前相整周期都挂在上母线。
     *----------------------------------------------------------------------*/
    float s_lead, s_mid, s_lag;
    switch (region)
    {
        case 1:  s_lead = (T1 + T2) / Ts;      s_mid =  T2 / Ts;        s_lag =  0.0f;            break;
        case 2:  s_lead = (T1 + T3) / Ts;      s_mid =  T1 / Ts;        s_lag =  0.0f;            break;
        case 3:  s_lead =  1.0f;               s_mid =  T1 / Ts;        s_lag = -T2 / Ts;         break;
        case 4:  s_lead =  1.0f;               s_mid =  T2 / Ts;        s_lag = -T3 / Ts;         break;
        case 5:  s_lead =  1.0f;               s_mid = -T2 / Ts;        s_lag = -(T2 + T3) / Ts;  break;
        case 6:  s_lead =  1.0f;               s_mid = (T1 + T3) / Ts;  s_lag = -(T2 + T3) / Ts;  break;
        default: s_lead =  1.0f;               s_mid =  0.0f;           s_lag = -1.0f;            break;
    }

    /*------------------------------------------------------------------------
     * 第 7 步：折算坐标 → 实际三相
     *
     *   折算坐标系的"A 轴"其实是实际空间的 (sector-1)·60° 方向，
     *   它不对应任何一相的真实轴（三相轴只在 0°/120°/240°），
     *   所以不能用"相序轮换"来映射。
     *
     *   正确的对应关系是：空间矢量转 60° ⟺ 三相量取负并循环移位，
     *   即 (x,y,z) → (-y,-z,-x)。连做 (sector-1) 次即可，展开后就是下面 6 个 case。
     *   奇数扇区取正、偶数扇区取负，对应你 MATLAB 里 vectorshare_up/down 的
     *   两套序列（用来让共模电压正负交替，平衡中点电位）。
     *----------------------------------------------------------------------*/
    float sa, sb, sc;
    switch (sector)
    {
        case 1:  sa =  s_lead;  sb =  s_mid;   sc =  s_lag;   break;
        case 2:  sa = -s_mid;   sb = -s_lag;   sc = -s_lead;  break;
        case 3:  sa =  s_lag;   sb =  s_lead;  sc =  s_mid;   break;
        case 4:  sa = -s_lead;  sb = -s_mid;   sc = -s_lag;   break;
        case 5:  sa =  s_mid;   sb =  s_lag;   sc =  s_lead;  break;
        default: sa = -s_lag;   sb = -s_lead;  sc = -s_mid;   break;   /* sector 6 */
    }

    /* 浮点兜底限幅（正常不会触发） */
    if(sa >  1.0f) sa =  1.0f;
    if(sa < -1.0f) sa = -1.0f;
    if(sb >  1.0f) sb =  1.0f;
    if(sb < -1.0f) sb = -1.0f;
    if(sc >  1.0f) sc =  1.0f;
    if(sc < -1.0f) sc = -1.0f;

    /*------------------------------------------------------------------------
     * 第 8 步：s 换算成每相两路比较值
     *
     *   三电平单桥臂有两个独立驱动信号：
     *       S1：载波 < Cmp_P  时导通  → 极电压被拉到 +Udc/2（P 态）
     *       S2：载波 < Cmp_S2 时导通  → 极电压被拉到 0 或 +Udc/2（P 态或 O 态）
     *       S3 = !S1、S4 = !S2（死区由硬件插入）
     *
     *   于是：
     *       s ≥ 0（正半周）：只用 P 和 O  →  Cmp_P = s·ARR,        Cmp_S2 = ARR
     *       s <  0（负半周）：只用 O 和 N  →  Cmp_P = 0,            Cmp_S2 = (1+s)·ARR
     *
     *   这样极电压 = (Udc/2)·s，并且恒有 Cmp_P ≤ Cmp_S2，
     *   不会出现 S1 与 S4 同时导通。
     *----------------------------------------------------------------------*/
    uint16_t arr = (uint16_t)svpwm_gen->sw_period;
    float    top = (float)arr - 1.0f;           //留 1 个计数余量，避免 100% 占空比

    float cpa = fmaxf(sa, 0.0f)            * (float)arr;    // A 相 S1 比较值
    float cna = (1.0f + fminf(sa, 0.0f))   * (float)arr;    // A 相 S2 比较值
    float cpb = fmaxf(sb, 0.0f)            * (float)arr;    // B 相 S1 比较值
    float cnb = (1.0f + fminf(sb, 0.0f))   * (float)arr;    // B 相 S2 比较值
    float cpc = fmaxf(sc, 0.0f)            * (float)arr;    // C 相 S1 比较值
    float cnc = (1.0f + fminf(sc, 0.0f))   * (float)arr;    // C 相 S2 比较值

    if(cpa > cna) cpa = cna;
    if(cpb > cnb) cpb = cnb;
    if(cpc > cnc) cpc = cnc;

    if(cpa < 0.0f) cpa = 0.0f;
    if(cpa > top)  cpa = top;
    if(cna < 0.0f) cna = 0.0f;
    if(cna > top)  cna = top;
    if(cpb < 0.0f) cpb = 0.0f;
    if(cpb > top)  cpb = top;
    if(cnb < 0.0f) cnb = 0.0f;
    if(cnb > top)  cnb = top;
    if(cpc < 0.0f) cpc = 0.0f;
    if(cpc > top)  cpc = top;
    if(cnc < 0.0f) cnc = 0.0f;
    if(cnc > top)  cnc = top;

    /* 四舍五入后写入寄存器（通道映射见文件头说明）
    通道顺序：a/b = A 相 S1/S2，c/d = B 相 S1/S2，e/f = C 相 S1/S2 */
    svpwm_gen->Cmpa = (uint16_t)(cpa + 0.5f);   /* A 相 S1 */
    svpwm_gen->Cmpb = (uint16_t)(cna + 0.5f);   /* A 相 S2 */
    svpwm_gen->Cmpc = (uint16_t)(cpb + 0.5f);   /* B 相 S1 */
    svpwm_gen->Cpmd = (uint16_t)(cnb + 0.5f);   /* B 相 S2 */
    svpwm_gen->Cmpe = (uint16_t)(cpc + 0.5f);   /* C 相 S1 */
    svpwm_gen->Cpmf = (uint16_t)(cnc + 0.5f);   /* C 相 S2 */
}
