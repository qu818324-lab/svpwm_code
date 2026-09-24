/*
 * foc_functions.h
 *
 *  Created on: 2026年3月31日
 *      Author: zjc
 */

#ifndef SVPWM_GEN_TWO_LEVLEL_H
#define SVPWM_GEN_TWO_LEVLEL_H

// #include"Include_CPU1.h"

#define pi 3.141592653589793238462
#define rad_to_deg(x) (x * 180.0 / pi)
#define deg_to_rad(x) (x * pi / 180.0)
#define sqrt3 1.73205080756887729352
#define sqrt2 1.414213562373095048222
#define sqrt3_2 0.866025403784438646763   /* sqrt(3)/2 */
#define sqrt3_3 0.577350269189625764509   /* 1/sqrt(3), 小矢量区域分界线常数 */

typedef struct
{
    float Id_ref;
    float Iq_ref;
    float Speed_ref;
    float theta;//转子角度
    float ia;
    float ib;
    float ic;
    float Udc;//电机供电电压
    float Tpwm;//电机pwm周期
}User_IN;

typedef struct
{
    float Vd;
    float Vq;
}Voltage_DQ_DEF;

typedef struct
{
    float Valpha;
    float Vbeta;
}Voltage_Alpha_Beta_DEF;//输入到svpwm，从svpwm输出到tcmp1，2，3

typedef struct
{
    float Udc;
    float Tpwm;
    float Valpha;
    float Vbeta;
    float Tcmp1;
    float Tcmp2;
    float Tcmp3;
}Svpwm_cal;

typedef struct
{
    float Ia;
    float Ib;
    float Ic;
}Current_ABC_DEF;


//extern User_IN User_in;                //输入结构体变量
//extern Voltage_DQ_DEF V_DQ;            //d_q轴电压

//void FOC_Init(void);
//void FOC_Run(void);
//void Angle_To_Cos_Sin(float angle, Trans_Cos_Sin_DEF* cos_sin);
void SVPWM_Calc(Svpwm_cal *p);

#endif /* __FOC_FUNCTIONS_H_ */
