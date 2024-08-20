#ifndef __PortF103Cn0510_H__
#define __PortF103Cn0510_H__

#include "main.h"

extern SPI_HandleTypeDef hspi1;


void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer, unsigned char *pRecvBuffer, unsigned long length);
void AD5940_CsClr(void);
void AD5940_CsSet(void);
void AD5940_RstSet(void);
void AD5940_RstClr(void) ;
void AD5940_Delay10us(uint32_t time);
uint32_t AD5940_GetMCUIntFlag(void);
uint32_t AD5940_ClrMCUIntFlag(void);
uint32_t AD5940_MCUResourceInit(void *pCfg);
void EXTI0_IRQHandler(void);

uint32_t AD5940_SET_ucInterrupted(void);


#endif

