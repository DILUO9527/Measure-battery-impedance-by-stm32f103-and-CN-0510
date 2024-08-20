#include "stdio.h"
#include "AD5940.h"
#include "AD5940Function.h"
#include "Impedance.h"
#include "main.h"
#include "math.h"
#include "BATImpedance.h"
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
#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];




/*       ADC    有问题  */
#define ADCPGA_GAIN_SEL   ADCPGA_1P5
static void AD5940_PGA_Calibration(void){
  AD5940Err err;
  ADCPGACal_Type pgacal;
  pgacal.AdcClkFreq = 16e6;
  pgacal.ADCSinc2Osr = ADCSINC2OSR_178;
  pgacal.ADCSinc3Osr = ADCSINC3OSR_4;
  pgacal.SysClkFreq = 16e6;
  pgacal.TimeOut10us = 1000;
  pgacal.VRef1p11 = 1.11f;
  pgacal.VRef1p82 = 1.82f;
  pgacal.PGACalType = PGACALTYPE_OFFSETGAIN;
  pgacal.ADCPga = ADCPGA_GAIN_SEL;
  err = AD5940_ADCPGACal(&pgacal);
  if(err != AD5940ERR_OK){
    printf("AD5940 PGA calibration failed.");
  }
}

void AD5940_ADC(void)
{
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;
  
  /* Use hardware reset */
  AD5940_HWReset();

  /* Firstly call this function after reset to initialize AFE registers. */
  AD5940_Initialize();
  
  AD5940_PGA_Calibration();
  /* Configure AFE power mode and bandwidth */
  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);
  
  /* Initialize ADC basic function */
  AD5940_AFECtrlS(AFECTRL_DACREFPWR|AFECTRL_HSDACPWR, bTRUE); //We are going to measure DAC 1.82V reference.
  adc_base.ADCMuxP = ADCMUXP_VREF1P8DAC;
  adc_base.ADCMuxN = ADCMUXN_VSET1P1;
  adc_base.ADCPga = ADCPGA_GAIN_SEL;
  AD5940_ADCBaseCfgS(&adc_base);
  
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH */
  adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;
  adc_filter.ADCSinc2Osr = ADCSINC2OSR_1333;
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */   
  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */ 
  AD5940_ADCFilterCfgS(&adc_filter);
  
  //AD5940_ADCMuxCfgS(ADCMUXP_AIN2, ADCMUXN_VSET1P1);   /* Optionally, you can change ADC MUX with this function */

  /* Enable all interrupt at Interrupt Controller 1. So we can check the interrupt flag */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); 

  //AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_SINC2NOTCH, bTRUE);
  //AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);
  AD5940_ADCPowerCtrlS(bTRUE);
  AD5940_ADCConvtCtrlS(bTRUE);

  while(1)
  {
    uint32_t rd;
   if(AD5940_INTCTestFlag(AFEINTC_1,AFEINTSRC_SINC2RDY))  
    {
     static uint32_t count;
      AD5940_INTCClrFlag(AFEINTSRC_SINC2RDY);
      rd = AD5940_ReadAfeResult(AFERESULT_SINC2);
      count ++;
      /* ADC Sample rate is 800kSPS. SINC3 OSR is 4, SINC2 OSR is 1333. So the final output data rate is 800kSPS/4/1333 = 150.0375Hz */
      if(count%150==0) /* Print data @1Hz */
      {
       // count = 0;
        float diff_volt = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82);
        printf("ADC Code:%d, diff-volt: %.4f, volt:%.4f \r\n",rd, diff_volt, diff_volt+1.11);
//        if (count==1500){
//         AD5940_Reset();
//        count=0;
//        }
      }
    }
  }
}



/*       AD5940_messege      */
void AD5940_messege(void)
{
  unsigned long temp;
  AD5940_HWReset();
  AD5940_Initialize();
  temp = AD5940_ReadReg(REG_AFECON_ADIID);
  printf("Read ADIID register, got: 0x%04lx\n", temp);
  if(temp != AD5940_ADIID)
    printf("Read register test failed.\n" );
  else
    printf("Read register test pass\n");
  AD5940_CsSet();

AD5940_RstSet();
}






/*       reset      */
void AD5940_Reset(void)
{
  uint32_t temp;
  printf("Wait 5 secondes\n");
  AD5940_Delay10us(100*5000); /* Delay 5s */
  printf("\n1. AD5940 Power ON\n");
  temp = AD5940_ReadReg(REG_ALLON_RSTSTA);
  print_rststa(temp);
  AD5940_WriteReg(REG_ALLON_RSTSTA, 0xf);  /* Clear reset status. This register will remain its value until we manually clear it. Reset operation won't reset this register. */

  printf("\n2. Perform Hardware reset now!\n");
  AD5940_HWReset();
  printf("Hardware reset done, status is:\n");
  temp = AD5940_ReadReg(REG_ALLON_RSTSTA);
  print_rststa(temp);
  AD5940_WriteReg(REG_ALLON_RSTSTA, 0xf);

  printf("\n3. Perform Software Reset now \n");
  AD5940_SoftRst();
  printf("Software reset done, status is:\n");
  temp = AD5940_ReadReg(REG_ALLON_RSTSTA);
  print_rststa(temp);
  printf("\nReset Test done \n");
  /**
   * @note MUST call this function whenever there is reset happened. This function will put AD5940 to right state.
  */
  AD5940_Initialize();
  AD5940_WriteReg(REG_ALLON_RSTSTA, 0xf); /* Clear reset status register. */

  //while(1)
  {
    printf("reset end\r\n");
  HAL_Delay(1000);
  }
    
}


void print_rststa(uint32_t reg)
{
  printf("<<<<<<<Reset Status<<<<<\n");
  if(reg & 0x01)
    printf("POR Reset Happened\n");
  if(reg & 0x02)
    printf("\r Hardware/External Reset Happened\n");
  if(reg & 0x08)
    printf("Software Reset Happened\n");
  if((reg&0xb) == 0)
    printf("No reset happened\n");
  printf(">>>>>>>Reset Status Done>>>>>\n");
}



/*       阻抗impedance      */




static int32_t AD5940_impedance_sys(void)
{
CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;//AppIMPCfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);
  fifo_cfg.FIFOEn = bTRUE;
  AD5940_FIFOCfg(&fifo_cfg);
  
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC;
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  return 0;
}  



void AD5940_impedance(void)
{
  uint32_t temp;  
 
//  while(1)
//  {
   if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();        //清楚标志位，进中断处理
      temp = APPBUFF_SIZE;
     AppIMPISR(AppBuff, &temp);     //有数据就进中断，处理数据
        ImpedanceShowResult(AppBuff, temp);//展示数据
    }
//  }
}


int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;

  fImpPol_Type *pImp = (fImpPol_Type*)pData;
  AppIMPCtrl(IMPCTRL_GETFREQ, &freq);

  printf("Freq:%.2f ", freq);
  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
    printf("RzMag: %f Ohm , RzPhase: %f  datacount:%d \r\n",pImp[i].Magnitude,pImp[i].Phase*180/MATH_PI,DataCount);
  }
  return 0;
}


void AD5940ImpedanceStructInit(void)
{
  AppIMPCfg_Type *pImpedanceCfg;
  
  AppIMPGetCfg(&pImpedanceCfg);
  /* Step1: configure initialization sequence Info */
  pImpedanceCfg->SeqStartAddr = 0;
  pImpedanceCfg->MaxSeqLen = 512; /* @todo add checker in function */

  pImpedanceCfg->RcalVal = 10000.0;
  pImpedanceCfg->SinFreq = 60000.0;
  pImpedanceCfg->FifoThresh = 4;
	
	/* Set switch matrix to onboard(EVAL-AD5940ELECZ) dummy sensor. */
	/* Note the RCAL0 resistor is 10kOhm. */
	pImpedanceCfg->DswitchSel = SWD_CE0;
	pImpedanceCfg->PswitchSel = SWP_RE0;
	pImpedanceCfg->NswitchSel = SWN_SE0;
	pImpedanceCfg->TswitchSel = SWT_SE0LOAD;
	/* The dummy sensor is as low as 5kOhm. We need to make sure RTIA is small enough that HSTIA won't be saturated. */
	pImpedanceCfg->HstiaRtiaSel = HSTIARTIA_5K;	
	
	/* Configure the sweep function. */
	pImpedanceCfg->SweepCfg.SweepEn = bTRUE;
	pImpedanceCfg->SweepCfg.SweepStart = 100.0f;	/* Start from 1kHz */
	pImpedanceCfg->SweepCfg.SweepStop = 100e3f;		/* Stop at 100kHz */
	pImpedanceCfg->SweepCfg.SweepPoints = 101;		/* Points is 101 */
	pImpedanceCfg->SweepCfg.SweepLog = bTRUE;
	/* Configure Power Mode. Use HP mode if frequency is higher than 80kHz. */
	pImpedanceCfg->PwrMod = AFEPWR_HP;
	/* Configure filters if necessary */
	pImpedanceCfg->ADCSinc3Osr = ADCSINC3OSR_2;		/* Sample rate is 800kSPS/2 = 400kSPS */
  pImpedanceCfg->DftNum = DFTNUM_16384;
  pImpedanceCfg->DftSrc = DFTSRC_SINC3;
}


void AD5940_impedance_init(void)
{
AD5940_impedance_sys();
AD5940ImpedanceStructInit();
AppIMPInit(AppBuff, APPBUFF_SIZE);    /* Initialize IMP application. Provide a buffer, which is used to store sequencer commands */
AppIMPCtrl(IMPCTRL_START, 0);          /* Control IMP measurement to start. Second parameter has no meaning with this command. */
printf("\r\nAD5940_impedance_init finished\r\n");
}






/*       温度          */
#define SINC3OSR_SEL  ADCSINC3OSR_4
#define SINC2OSR_SEL  ADCSINC2OSR_22
#define MEASURE_FREQ	4.0f	//4Hz(4SPS)
#define FIFO_THRESHOLD	4		//generate FIFO threshold interrupt every 4 data.


#define BUFF_SIZE 4
//this buffer will be used by sequence generator and used to store result from AD5940
uint32_t buff[BUFF_SIZE];
uint32_t data_count = 0;  //the temperature data count in buffer.


static void AD5940_sys_temperature(void){
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  SEQCfg_Type seq_cfg;
  AGPIOCfg_Type gpio_cfg;
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                      /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH;
  fifo_cfg.FIFOThresh = FIFO_THRESHOLD;
  AD5940_FIFOCfg(&fifo_cfg);                             /* Disable to reset FIFO. */
  fifo_cfg.FIFOEn = bTRUE;  
  AD5940_FIFOCfg(&fifo_cfg);                             /* Enable FIFO here */
  /* Configure sequencer and stop it */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  /* Step3. Interrupt controller */
  
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);           /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);   /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  //AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_ALLINT, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP6_SYNC|GP5_SYNC|GP2_TRIG|GP1_SYNC|GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin5|AGPIO_Pin6;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = AGPIO_Pin2;
  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Enable AFE to enter sleep mode. */
}

void _ad5940_analog_init_temperature(void){
  //AD5940_TemperatureInit运行了analog
  AFERefCfg_Type aferef_cfg;
  ADCBaseCfg_Type adc_base;
  ADCFilterCfg_Type adc_filter;
  //init ad5940 for temperature measurement.
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  /* Init all to disable state */
  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;       /* The High speed buffers are automatically turned off during hibernate */
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bFALSE;
  aferef_cfg.Lp1V8BuffEn = bFALSE;
  /* LP reference control - turn off them to save power*/
  aferef_cfg.LpBandgapEn = bFALSE;
  aferef_cfg.LpRefBufEn = bFALSE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);	
  /* Initialize ADC basic function */
  adc_base.ADCMuxP = ADCMUXP_TEMPP;
  adc_base.ADCMuxN = ADCMUXN_TEMPN;
  adc_base.ADCPga = ADCPGA_1P5;
  AD5940_ADCBaseCfgS(&adc_base);
  /* Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH */
  adc_filter.ADCSinc3Osr = SINC3OSR_SEL;
  adc_filter.ADCSinc2Osr = SINC2OSR_SEL;
  adc_filter.ADCAvgNum = ADCAVGNUM_2;         /* Don't care about it. Average function is only used for DFT */
  adc_filter.ADCRate = ADCRATE_800KHZ;        /* If ADC clock is 32MHz, then set it to ADCRATE_1P6MHZ. Default is 16MHz, use ADCRATE_800KHZ. */
  adc_filter.BpNotch = bTRUE;                 /* SINC2+Notch is one block, when bypass notch filter, we can get fresh data from SINC2 filter. */
  adc_filter.BpSinc3 = bFALSE;                /* We use SINC3 filter. */
  adc_filter.Sinc2NotchEnable = bTRUE;        /* Enable the SINC2+Notch block. You can also use function AD5940_AFECtrlS */
  AD5940_ADCFilterCfgS(&adc_filter);
  AD5940_AFECtrlS(AFECTRL_TEMPSPWR, bTRUE);   /* Turn on temperature sensor power */
}

void AD5940_TemperatureInit(void){   //SEQ CLK
  uint32_t const *pSeqCmd;
  uint32_t seq_len;
  SEQInfo_Type seq_info;
  WUPTCfg_Type wupt_cfg;
  ClksCalInfo_Type clks_cal;
  uint32_t WaitClks;
  clks_cal.DataType = DATATYPE_SINC2;
  clks_cal.DataCount = 1;             /* Sample one data when wakeup */
  clks_cal.ADCSinc2Osr = SINC2OSR_SEL;
  clks_cal.ADCSinc3Osr = SINC3OSR_SEL;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = 1; /* Assume ADC clock is same as system clock */
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  _ad5940_analog_init_temperature();
  //generate sequence to measure temperature sensor output
  AD5940_SEQGenInit(buff, BUFF_SIZE); //init sequence generator
  AD5940_SEQGenCtrl(bTRUE); //from now on, record all register operations rather than write them to AD5940 through SPI.

  AD5940_SEQGpioCtrlS(AGPIO_Pin1);  //pull high AGPIO1 so we know the sequencer is running by observing pin status with oscilloscope etc.
  AD5940_SEQGenInsert(SEQ_WAIT(16*200));  /* Time for reference settling(if ad5940 is just wake up from hibernate mode) */
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE); /* Turn ON ADC power */
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));   /* wait another 50us for ADC to settle. */
  AD5940_AFECtrlS(AFECTRL_TEMPCNV|AFECTRL_ADCCNV, bTRUE);  /* Start ADC convert */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  AD5940_AFECtrlS(AFECTRL_TEMPCNV|AFECTRL_ADCPWR, bFALSE);    /* Stop ADC */
  AD5940_SEQGenInsert(SEQ_WAIT(20));			/* Add some delay before put AD5940 to hibernate, needs some clock to move data to FIFO. */
  AD5940_SEQGpioCtrlS(0);     /* pull low AGPIO so we know end of sequence.*/
  AD5940_EnterSleepS();/* Goto hibernate */
  AD5940_SEQGenCtrl(bFALSE);  /* stop sequence generator */
  if(AD5940_SEQGenFetchSeq(&pSeqCmd, &seq_len) != AD5940ERR_OK){
    puts("Sequence generator error!");
  }
  seq_info.pSeqCmd = pSeqCmd;
  seq_info.SeqId = SEQID_0; //use SEQ0 to run this sequence
  seq_info.SeqLen = seq_len;
  seq_info.SeqRamAddr = 0;  //place this sequence from start of SRAM.
  seq_info.WriteSRAM = bTRUE;// we need to write this sequence to AD5940 SRAM.
  AD5940_SEQInfoCfg(&seq_info);
  
  //now configure wakeup timer to trigger above sequence periodically to measure temperature data.
  wupt_cfg.WuptEn = bFALSE; // do not start it right now.
  wupt_cfg.WuptEndSeq = WUPTENDSEQ_A;
  wupt_cfg.WuptOrder[0] = SEQID_0;
  wupt_cfg.SeqxSleepTime[SEQID_0] = 4-1;
  wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(32e3f/MEASURE_FREQ)-4-1;
  AD5940_WUPTCfg(&wupt_cfg);
  //enable sequencer
  AD5940_SEQCtrlS(bTRUE); //now sequencer is ready to be triggered.
}


void AD5940_TemperatureISR(void){
  //process data from AD5940 FIFO.
  uint32_t FifoCnt, IntcFlag;
  if(AD5940_WakeUp(10) > 10){  /* Wakeup AFE by read register, read 10 times at most */
    printf("Failed to wakeup AD5940!\n");
    return;
  }
  
 // printf("REG_INTC_INTCSEL0 :%x     REG_INTC_INTCSEL1:%x \r\n ",AD5940_INTCGetCfg(REG_INTC_INTCSEL0),AD5940_INTCGetCfg(REG_INTC_INTCSEL1));
  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);  /* We need time to read data from FIFO, so, do not let AD5940 goes to hibernate automatically */
  IntcFlag = AD5940_INTCGetFlag(AFEINTC_1);
  printf("\nIntcFlag:%x \r %d \r\n",IntcFlag,IntcFlag);
  printf("AFEINTC_0 %x     AFEINTC_1:%x \r\n",AD5940_INTCGetFlag(AFEINTC_0),AD5940_INTCGetFlag(AFEINTC_1));
  printf("AFEINTSRC_DATAFIFOTHRESH:%x\r\n",AFEINTSRC_DATAFIFOTHRESH);
  //if(IntcFlag&AFEINTSRC_DATAFIFOTHRESH)
    {
    FifoCnt = AD5940_FIFOGetCnt();
      printf("FifoCnt:%d \r",FifoCnt);
    FifoCnt = FifoCnt>BUFF_SIZE?BUFF_SIZE:FifoCnt;
      printf("?FifoCnt:%d \r\n",FifoCnt);
    data_count = FifoCnt;
    AD5940_FIFORd(buff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);    /* Allow AFE to enter sleep mode. AFE will stay at active mode until sequencer trigger sleep */
    AD5940_EnterSleepS();	//If MCU is too slow, comment this line, otherwise there is chance the sequencer is running at this point.
  }
}

void AD5940_PrintResult_temperature(void)
  {
  for(int i=0; i<data_count; i++){
    int32_t data = buff[i]&0xffff;
    data -= 0x8000;	//data from SINC2 is added 0x8000, while data from register TEMPSENSDAT has no 0x8000 offset.
    printf("Result[%d] = %d, %.2f(C)\n", i, data, data/8.13f/1.5f-273.15f);
  }
}

void AD5940_temperature(void){
  AD5940_sys_temperature();
  printf("Internal calibration register value:\nGain: 0x%08x\n", AD5940_ReadReg(REG_AFE_ADCGAINDIOTEMPSENS));
  printf("Offset: 0x%08x\n", AD5940_ReadReg(REG_AFE_ADCOFFSETEMPSENS1));
  AD5940_TemperatureInit();
  AD5940_WUPTCtrl(bTRUE); 
  printf("\r\nAD5940_TemperatureInit\r\n");
}
void AD5940_run_temperature(void)
{
  //while(1)
    { 
    // printf("AFECON: %x\r",AD5940_ReadReg(0x00002000));
    /* Check if interrupt flag which will be set when interrupt occurred. */
    printf("run flg : %d\r\n",AD5940_GetMCUIntFlag());
      //HAL_Delay(100);
   if(AD5940_GetMCUIntFlag())
      {
    AD5940_ClrMCUIntFlag(); /* Clear this flag */
      AD5940_TemperatureISR();
      AD5940_PrintResult_temperature();
    }

  }
}
  
  /*       BATZ电池阻抗  */
  
  
  
  /* It's your choice here how to do with the data. Here is just an example to print them to UART */
//输出数据
int32_t BATShowResult(uint32_t *pData, uint32_t DataCount)
{
  fImpCar_Type *pImp = (fImpCar_Type*)pData;
	float freq;
	AppBATCtrl(BATCTRL_GETFREQ, &freq);
  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
    printf("Freq: %f   (real, image) =  %f , %f   mOhm",freq, pImp[i].Real,pImp[i].Image);
  }
  return 0;
}
  
  
  
  /* Initialize AD5940 basic blocks like clock */
static int32_t AD5940PlatformCfg_BATZ(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
  /* Use hardware reset */
  AD5940_HWReset();
  /* Platform configuration */
  AD5940_Initialize();
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC; //on battery board, there is a 32MHz crystal.
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;//AppBATCfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);                             /* Disable to reset FIFO. */
  fifo_cfg.FIFOEn = bTRUE;  
  AD5940_FIFOCfg(&fifo_cfg);                             /* Enable FIFO here */
  
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);           /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);   /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP0_INT|GP2_SYNC;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  return 0;
}

void AD5940BATStructInit_BATZ(void)
{
  AppBATCfg_Type *pBATCfg;
  AppBATGetCfg(&pBATCfg);
  pBATCfg->SeqStartAddr = 0;
  pBATCfg->MaxSeqLen = 512;
  pBATCfg->RcalVal = 50.0;  							/* Value of RCAL on EVAL-AD5941BATZ board is 50mOhm */
  pBATCfg->ACVoltPP = 300.0f;							/* Pk-pk amplitude is 300mV */
  pBATCfg->DCVolt = 1200.0f;							/* Offset voltage of 1.2V*/
  pBATCfg->DftNum = DFTNUM_8192;
  
  pBATCfg->FifoThresh = 2;      					/* 2 results in FIFO, real and imaginary part. */
	
	pBATCfg->SinFreq = 200;									/* Sin wave frequency. THis value has no effect if sweep is enabled */
	
	pBATCfg->SweepCfg.SweepEn = bTRUE;			/* Set to bTRUE to enable sweep function */
	pBATCfg->SweepCfg.SweepStart = 1.0f;		/* Start sweep at 1Hz  */
	pBATCfg->SweepCfg.SweepStop = 50000.0f;	/* Finish sweep at 1000Hz */
	pBATCfg->SweepCfg.SweepPoints = 50;			/* 100 frequencies in the sweep */
	pBATCfg->SweepCfg.SweepLog = bTRUE;			/* Set to bTRUE to use LOG scale. Set bFALSE to use linear scale */
	
}

void AD5940_init_BATZ(void)
{uint32_t temp;
  AD5940PlatformCfg_BATZ();
  AD5940BATStructInit_BATZ(); /* Configure your parameters in this function */
  printf("\n开始自校准Start self calibration\r\n");
  AppBATInit(AppBuff, APPBUFF_SIZE);    /* Initialize BAT application. Provide a buffer, which is used to store sequencer commands */
  AppBATCtrl(BATCTRL_MRCAL, 0);     /* Measur RCAL each point in sweep */
	AppBATCtrl(BATCTRL_START, 0); 
      printf("\n自校准完毕Self calibration completed\r\n");
  /* Check if interrupt flag which will be set when interrupt occurred. */
printf("初始化完毕\r\n");
}


void AD5940_RUN_BATZ(void)
  
{
    printf("开始测量电池阻抗\r\n");
  while(1)
  {  uint32_t temp;
    /* Check if interrupt flag which will be set when interrupt occurred. */

    HAL_Delay(1000);
    if(AD5940_GetMCUIntFlag())
    {
				AD5940_ClrMCUIntFlag(); 				/* Clear this flag */
				temp = APPBUFF_SIZE;
				AppBATISR(AppBuff, &temp); 			/* Deal with it and provide a buffer to store data we got */
				AD5940_Delay10us(100000);
				BATShowResult(AppBuff, temp);		/* Print measurement results over UART */		
				AD5940_SEQMmrTrig(SEQID_0);  		/* Trigger next measurement ussing MMR write*/      
   }
  }
  
  
}



