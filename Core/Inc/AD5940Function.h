#ifndef _AD5940Function_H_
#define _AD5940Function_H_





void print_rststa(uint32_t reg);
void AD5940_Reset(void);
void AD5940_messege(void);
static void AD5940_PGA_Calibration(void);
void AD5940_ADC(void);
void AD5940ImpedanceStructInit(void);
void AD5940_impedance_init(void);
void AD5940_impedance(void);
static int32_t AD5940_impedance_sys(void);
int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount);
void AD5940_temperature(void);
void AD5940_run_temperature(void);
void AD5940_init_BATZ(void);
void AD5940_RUN_BATZ(void);
#endif



