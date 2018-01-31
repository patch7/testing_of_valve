/******************************************************************************
  Ядро и переферию настроили на максимальную частоту. Настройка каналов АЦП для обработки органов управления, обратной связи. Настройка таймера для генерации ШИМ с автоматическим регулирование частоты от 100 до 400 Гц с возможностью настройки шага изменения частоты. Настройка ПДП (ДМА) для автоматической обработки данных с АЦП (ручное заполнение, ток на клапане, ток на клапане с внешнего шунта).
  По кану принимаем настройки системы: режим работы (ручной, автоматическое пропорциональное управление, автоматическое дискретное управление, максимальный ток); ручное управление частотой ШИМ; время заполнения ШИМ в авто управлении; процент пропуска верхнего предела тока; шаг частоты в авто управлении; количество выборок на каждой частоте.
  Для диагностики и фиксации процессов выводим по кану: заполнение ШИМ; ток; частоту; время заполнения ШИМ в авто управлении; режим работы; процент пропуска верхнего предела тока; шаг частоты в авто управлении; количество выборок на каджой частоте.
  Для фильтрации шумов с АЦП используем класс скользящей медианы.

  Разработал Михайлов А.В.
  Дата изменения 19.01.2018

  Заметки:
  1. Необходимо все таймеры инициализировать в одном месте, для избежания дублирования кода и возможного прерывания до окончания инициализации.
  2. Для настройки тактирования необходимо либо править файл system_stm32f4xx.c, либо явно вызывать функции SystemInit() и SystemCoreClockUpdate() и настраивать тактирование через RCC модуль.
  3. Для фильтрации сообщений кан по идентификатору есть статейка forum.easyelectronics.ru/viewtopic.php?p=229471
******************************************************************************/

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "sliding_median.h"
#include "testingvalve.h"

#define in_current  ADCValue[0]
#define out_current ADCValue[1]
#define filling     ADCValue[2]
#define pressure    ADCValue[3]

static uint16_t ADCValue[4] = {0};
static uint16_t ADCDith[1]  = {0};

SlidingMedian<uint16_t> SMCurIn(7);
SlidingMedian<uint16_t> SMCurOut(7);
SlidingMedian<uint16_t> SMFill(7);
SlidingMedian<uint16_t> SMPress(7);
SlidingMedian<uint16_t> SMDith(7);

uint32_t time_count = 0;

TestingValve test;
uint8_t  timecur = 0;
bool PropState = false, StepState = false;

void RccBusConfig();
void DMAofADCinit();
void ADCinit();
void CANinit();
void TIMinit();

void main()
{
  GPIO_DeInit(GPIOA);//CAN1, ADC3_ch_1 (токовый)
  GPIO_DeInit(GPIOB);//TIM4(PWM), CAN2
  GPIO_DeInit(GPIOC);//ADC3_ch_10 (11), ADC3_ch_11 (29)

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  RccBusConfig();
  DMAofADCinit();
  ADCinit();
  CANinit();
  TIMinit();

  while(true)
    __NOP();
}
void RccBusConfig()//Настройка ядра и всей переферии на максимальные частоты от HSE 8 Mhz
{
  SystemInit();
  SystemCoreClockUpdate();

  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);
  RCC_ClockSecuritySystemCmd(ENABLE);

  ErrorStatus State = RCC_WaitForHSEStartUp();
  
  if(State != SUCCESS)
    while(true);
  else
  {
    RCC_HCLKConfig(RCC_SYSCLK_Div1);//AHB clock = SYSCLK
    RCC_PCLK1Config(RCC_HCLK_Div4);//APB1 clock = HCLK/4
    RCC_PCLK2Config(RCC_HCLK_Div2);//APB2 clock = HCLK/2

    RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);//8 MHz
    RCC_PLLCmd(ENABLE);

    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//SYSCLK clock = PLLCLK
    while(RCC_GetSYSCLKSource() != 8) {}
  }
}
void DMAofADCinit()//Настройка модуля DMA2 для автоматической обработки каналов АЦП3
{
  DMA_DeInit(DMA2_Stream0);
  DMA_DeInit(DMA2_Stream2);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  DMA_InitTypeDef DMA_InitStruct;
  DMA_StructInit(&DMA_InitStruct);
  DMA_InitStruct.DMA_Channel            = DMA_Channel_2;
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(ADC3->DR);
  DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)ADCValue;
  DMA_InitStruct.DMA_BufferSize         = 4;
  DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
  DMA_InitStruct.DMA_Mode               = DMA_Mode_Circular;
  DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
  DMA_Init(DMA2_Stream0, &DMA_InitStruct);

  DMA_InitStruct.DMA_Channel            = DMA_Channel_1;
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(ADC2->DR);
  DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)ADCDith;
  DMA_InitStruct.DMA_BufferSize         = 1;
  DMA_Init(DMA2_Stream2, &DMA_InitStruct);

  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);//если прерывание ненадо, можно ли убрать
  DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);//если прерывание ненадо, можно ли убрать

  NVIC_InitTypeDef NVIC_InitStruct;//Настройка прерывания, без него не работает!
  NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream0_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x0;
  NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream2_IRQn;
  NVIC_Init(&NVIC_InitStruct);

  DMA_Cmd(DMA2_Stream0, ENABLE);
  DMA_Cmd(DMA2_Stream2, ENABLE);
}
void ADCinit()//Настройка АЦП2 и АЦП3, преобразование по таймеру 2 по спаду и нарастанию ШИМ
{
  ADC_DeInit();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_StructInit(&GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;//Ручное заполнение и ток с внешнего шунта
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0;//Ток с дизерингом
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;//Ток с внутреннего шунта
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_7;//Датчик давления
  GPIO_Init(GPIOF, &GPIO_InitStruct);

  ADC_CommonInitTypeDef ADC_CommonInitStruct;
  ADC_CommonStructInit(&ADC_CommonInitStruct);
  ADC_CommonInit(&ADC_CommonInitStruct);

  ADC_InitTypeDef ADC_InitStruct;
  ADC_StructInit(&ADC_InitStruct);

  ADC_InitStruct.ADC_ScanConvMode         = ENABLE;
  ADC_InitStruct.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T2_CC2;
  ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_RisingFalling;
  ADC_InitStruct.ADC_NbrOfConversion      = 4;
  ADC_Init(ADC3, &ADC_InitStruct);

  ADC_InitStruct.ADC_NbrOfConversion      = 1;
  ADC_Init(ADC2, &ADC_InitStruct);

  ADC_RegularChannelConfig(ADC3, ADC_Channel_1,  1, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 2, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 3, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_5,  4, ADC_SampleTime_480Cycles);

  ADC_RegularChannelConfig(ADC2, ADC_Channel_8,  1, ADC_SampleTime_480Cycles);

  ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);//Запрос после последней передачи,
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);//без него не работает
  ADC_DMACmd(ADC2, ENABLE);
  ADC_DMACmd(ADC3, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
  ADC_Cmd(ADC3, ENABLE);
  ADC_SoftwareStartConv(ADC2);
  ADC_SoftwareStartConv(ADC3);
}
void CANinit()
{
  CAN_DeInit(CAN1);
  CAN_DeInit(CAN2);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_13;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5,  GPIO_AF_CAN2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

  CAN_InitTypeDef CAN_InitStruct;//Частота шины 42МГц / 24 = 1.75МГц / 7 = 250 кбит/с
  CAN_StructInit(&CAN_InitStruct);

  CAN_InitStruct.CAN_ABOM = ENABLE;
  CAN_InitStruct.CAN_TXFP = ENABLE;
  CAN_InitStruct.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStruct.CAN_Prescaler = 24;
  CAN_Init(CAN1, &CAN_InitStruct);
  CAN_Init(CAN2, &CAN_InitStruct);

  CAN_SlaveStartBank(0);//задаем номер фильтра, без него не работает
  CAN_FilterInitTypeDef CAN_FilterInitStruct;//без инициализации фильтра не работает

  CAN_FilterInitStruct.CAN_FilterIdHigh         = 0;
  CAN_FilterInitStruct.CAN_FilterIdLow          = 0;
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh     = 0;
  CAN_FilterInitStruct.CAN_FilterMaskIdLow      = 0;
  CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
  CAN_FilterInitStruct.CAN_FilterNumber         = 0;
  CAN_FilterInitStruct.CAN_FilterMode           = CAN_FilterMode_IdMask;
  CAN_FilterInitStruct.CAN_FilterScale          = CAN_FilterScale_32bit;
  CAN_FilterInitStruct.CAN_FilterActivation     = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStruct);

  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel                   = CAN2_RX0_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;
  NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
}
void TIMinit()
{
  TIM_DeInit(TIM2);
  TIM_DeInit(TIM3);
  TIM_DeInit(TIM4);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
  
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;//Максимальная скорость для работы ШИМ
  GPIO_Init(GPIOC, &GPIO_InitStructure);//TIM3 Дизеринг ШИМ
  GPIO_Init(GPIOB, &GPIO_InitStructure);//TIM4 Обычный ШИМ

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
  TIM_TimeBaseInitStruct.TIM_Prescaler = 9;//всегда +1
  TIM_TimeBaseInitStruct.TIM_Period    = 1680;//200 Гц на 25 точек, несущая частота 5 кГц
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

  TIM_TimeBaseInitStruct.TIM_Prescaler = 839;//всегда +1
  TIM_TimeBaseInitStruct.TIM_Period    = 1000;//100 Gz
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

  TIM_TimeBaseInitStruct.TIM_Period    = 100;//1 мс для того чтобы отследить синусоиду на 200Гц
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

  TIM_OCInitTypeDef TIM_OCInitStruct;
  TIM_OCStructInit(&TIM_OCInitStruct);
  TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM1;
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_Pulse       = 0;
  TIM_OC1Init(TIM3, &TIM_OCInitStruct);
  TIM_OC1Init(TIM4, &TIM_OCInitStruct);

  TIM_OCInitStruct.TIM_Pulse = 50;//полупериод
  TIM_OC2Init(TIM2, &TIM_OCInitStruct);//Для сканирования АЦП по прерыванию. GPIOх ненужно
  TIM_SetCounter(TIM2, 0);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);//разрешает загрузку в CCR1
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);//разрешает загрузку в CCR1
  TIM_ARRPreloadConfig(TIM3, ENABLE);//разрешает предварительную загрузку в ARR
  TIM_ARRPreloadConfig(TIM4, ENABLE);//разрешает предварительную загрузку в ARR

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//прерывание по переполнению
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//прерывание по переполнению
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);//прерывание по переполнению

  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_EnableIRQ(TIM3_IRQn);
  NVIC_EnableIRQ(TIM4_IRQn);

  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
}

void ProportionalSet()
{
  static uint16_t mycount = 0;
  if(mycount != sizeof(Table) / sizeof(*Table))
    TIM_SetCompare1(TIM4, Table[mycount++]);
  else
  {
    mycount = 0;
    PropState = false;
  }
}
void StepSetCurrent()
{
  static uint16_t mycount = 0;
  static uint16_t pwm = 8;
  if(!(mycount % (20 + (20 * timecur))))
  {
    TIM_SetCompare1(TIM4, pwm);
    pwm += 8;
  }
  if(mycount == 2520*(timecur+1))
  {
    mycount = 0;
    pwm = 8;
    StepState = false;
  }
  ++mycount;
}


extern "C"
{
  void DMA2_Stream0_IRQHandler()//Обработчик привязан к TIM2 OC2.
  {
    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
    {
      DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
      SMCurIn.push(in_current);
      SMCurOut.push(out_current);
      SMFill.push(filling);
      SMPress.push(pressure);
    }
  }
  void DMA2_Stream2_IRQHandler()//Обработчик привязан к TIM2 OC2.
  {
    if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))
    {
      DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
      SMDith.push(ADCDith[0]);
    }
  }
  /****************************************************************************
  Реализованно 4 режима управления ШИМ сигналом:
  1. Ручной - коэффициент заполнения регулируется с помощью потенциометра.
  2. Авто пропорциональное управление - заполнение ШИМ за определенное время, частота меняется с заданым шагом c заданым числом выборок на каждой. Можно ограничить верхний предел тока в %.
  3. Максимальный ток - выставляется максимальный ток на клапане.
  4. Авто дискретное управление - выставляется максимальный ток на заданное время, с заданым числом выборок. Частота изменяется с заданым шагом.
  ****************************************************************************/
  void TIM4_IRQHandler()
  {
    if(TIM_GetITStatus(TIM4, TIM_IT_Update))
    {
      TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
      test.Run(SMFill.get());
    }
  }
  void TIM3_IRQHandler()
  {
    if(TIM_GetITStatus(TIM3, TIM_IT_Update))
    {
      TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
      test.DitheringSinus();
    }
  }
  void TIM2_IRQHandler()//Вывод данных для контроля параметров.
  {
    if(TIM_GetITStatus(TIM2, TIM_IT_Update))
    {
      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
      ++time_count;

      CanTxMsg TxMessage;
      TxMessage.RTR = CAN_RTR_DATA;
      TxMessage.IDE = CAN_ID_STD;
      TxMessage.DLC = 8;

      if(!(time_count % 3))//3ms
      {
        TxMessage.StdId   = 0x003;
        TxMessage.Data[0] = static_cast<uint8_t>(SMCurOut.get());
        TxMessage.Data[1] = static_cast<uint8_t>(SMCurOut.get() >> 8);
        TxMessage.Data[2] = static_cast<uint8_t>(SMCurIn.get());
        TxMessage.Data[3] = static_cast<uint8_t>(SMCurIn.get()  >> 8);
        TxMessage.Data[4] = static_cast<uint8_t>(SMFill.get());
        TxMessage.Data[5] = static_cast<uint8_t>(SMFill.get()   >> 8);
        TxMessage.Data[6] = static_cast<uint8_t>(SMPress.get());
        TxMessage.Data[7] = static_cast<uint8_t>(SMPress.get()  >> 8);
        while(!CanTxMailBoxIsEmpty(CAN2));
        CAN_Transmit(CAN2, &TxMessage);

        CanTxMsg TxMessage;
        TxMessage.RTR     = CAN_RTR_DATA;
        TxMessage.IDE     = CAN_ID_STD;
        TxMessage.DLC     = 2;
        TxMessage.StdId   = 0x005;
        TxMessage.Data[0] = static_cast<uint8_t>(SMDith.get());
        TxMessage.Data[1] = static_cast<uint8_t>(SMDith.get() >> 8);
        while(!CanTxMailBoxIsEmpty(CAN2));
        CAN_Transmit(CAN2, &TxMessage);
      }
      if(!(time_count % 5))//5ms
      {
        TxMessage.StdId   = 0x004;
        TxMessage.Data[0] = static_cast<uint8_t>(SMCurOut.get());
        TxMessage.Data[1] = static_cast<uint8_t>(SMCurOut.get() >> 8);
        TxMessage.Data[2] = static_cast<uint8_t>(SMCurIn.get());
        TxMessage.Data[3] = static_cast<uint8_t>(SMCurIn.get()  >> 8);
        TxMessage.Data[4] = static_cast<uint8_t>(SMFill.get());
        TxMessage.Data[5] = static_cast<uint8_t>(SMFill.get()   >> 8);
        TxMessage.Data[6] = static_cast<uint8_t>(SMPress.get());
        TxMessage.Data[7] = static_cast<uint8_t>(SMPress.get()  >> 8);
        while(!CanTxMailBoxIsEmpty(CAN2));
        CAN_Transmit(CAN2, &TxMessage);

        if(PropState)
          ProportionalSet();
        if(StepState)
          StepSetCurrent();
      }
      if(!(time_count % 99))//100ms
        test.SendMsg();
    }
  }
  //Для взаимодействия с оператором вводится обработка принимаемых сообщений.
  void CAN2_RX0_IRQHandler(void)
  {
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0))
    {
      CanRxMsg RxMessage;
      CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
      CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);

      if(RxMessage.StdId == 0x001)
      {
        test.SetData(RxMessage);
        timecur = RxMessage.Data[6];
      }
      else if(RxMessage.StdId == 0x010)
        PropState = true;
      else if(RxMessage.StdId == 0x011)
        StepState = true;
    }
  }
}