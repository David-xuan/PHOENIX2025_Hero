#ifndef _FILTER_H_
#define _FILTER_H_

#include "main.h"

#define BUFFER_SIZE 1000
#define FILTER_ORDER 4

#define FILTERSAMPLERATE 20000 // ²ÉÑùÂÊ
typedef struct
{
    float states0;
    float states1;
    float b0;
    float b1;
    float b2;
    float a0;
    float a1;
    float a2;
    float gain0;
    float gain1;

    float out_Iq;
} Iq_IIR;

extern Iq_IIR SPEEDFilter, ENSPEEDFilter;

extern void IIR_Init(float temp[8], Iq_IIR *IIR);
extern float IIR_Calc(float in, Iq_IIR *IIR);

#endif
