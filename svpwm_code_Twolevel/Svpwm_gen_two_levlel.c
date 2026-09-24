/*
 * foc_function.c
 *
 *  Created on: 2026年3月31日
 *      Author: zjc
 */
#include "Svpwm_gen_two_levlel.h"
#include "math.h"
///定义变量
//User_IN User_in;
//Svpwm_OUT Svpwm_out;            //输出结构体变量
//Trans_Cos_Sin_DEF Trans_Cos_Sin;//角度三角函数计算值
//Voltage_DQ_DEF V_DQ;            //d_q轴电压


//SVPWM计算
void Svpwm_gen_Update(Svpwm_cal *p)
{
    int sector;
    float t1,t2,x,y,z,T,Ta,Tb,Tc;
    sector = 0;
    if(p->Vbeta > 0)
        sector = 1;             
    if((sqrt3 * p->Valpha -p->Vbeta) > 0)
        sector = sector + 2;
    if((-sqrt3 * p->Valpha -p->Vbeta) > 0)
        sector = sector + 4;
    ////xyz计算
    x = sqrt3 * p->Vbeta * p->Tpwm / p->Udc;
    y = (1.5f * p->Valpha + sqrt3 / 2 * p->Vbeta) * p->Tpwm / p->Udc;   
    z = (-1.5f * p->Valpha + sqrt3 / 2 * p->Vbeta) * p->Tpwm / p->Udc;
    switch(sector)
    {
        case 1:
            t1 = z;t2 = y;break;
        case 2:
            t1 = y;t2 = -x;break;
        case 3:
            t1 = -z;t2 = x;break;
        case 4:
            t1 = -x;t2 = z;break;
        case 5:
            t1 = x;t2 = -y;break;
        default:
            t1 = -y;t2 = -z;break;
    }
    //过调制
    T = t1 + t2;
    if(T > p->Tpwm)
    {
        float scale = p->Tpwm / T;
        t1 *= scale;
        t2 *= scale;
    }
    Ta = (p->Tpwm - (t1 + t2)) / 4.0;
    Tb = Ta + t1 / 2;
    Tc = Tb + t2 / 2;
    switch(sector)
    {
        case 1:
            p->Tcmp1 = Tb;
            p->Tcmp2 = Ta;
            p->Tcmp3 = Tc;
            break;
        case 2:
            p->Tcmp1 = Ta;
            p->Tcmp2 = Tc;
            p->Tcmp3 = Tb;
            break;
        case 3:
            p->Tcmp1 = Ta;
            p->Tcmp2 = Tb;
            p->Tcmp3 = Tc;
            break;
        case 4:
            p->Tcmp1 = Tc;
            p->Tcmp2 = Tb;
            p->Tcmp3 = Ta;
            break;
        case 5:
            p->Tcmp1 = Tc;
            p->Tcmp2 = Ta;
            p->Tcmp3 = Tb;
            break;
        case 6:
            p->Tcmp1 = Tb;
            p->Tcmp2 = Tc;
            p->Tcmp3 = Ta;
            break;
        default:    /* sector == 0：仅当 Valpha = Vbeta = 0 时出现，输出零矢量 */
                    /* 三相占空比相同 → 线电压为 0；取 Tpwm/4 与线性区 Vref→0 时连续 */
            p->Tcmp1 = p->Tcmp2 = p->Tcmp3 = 0.25f * p->Tpwm;
            break;
    }
}
