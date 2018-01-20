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

enum mode{manual, max_cur, auto_p, auto_d, idle};

#define in_current  ADCValue[0]
#define out_current ADCValue[1]
#define filling     ADCValue[2]
#define pressure    ADCValue[3]

const static uint32_t freqt = 100000;
const static uint16_t maxf  = 400;
const static uint16_t minf  = 100;

static uint16_t ADCValue[4] = {0};
static uint16_t freq        = minf;
static uint16_t count       = 0;
static uint8_t  time        = 2;
static uint8_t  stepf       = 25;
static uint8_t  amount      = 2;
static uint8_t  percent     = 0;
static uint8_t  timecur     = 0;

bool PropState = false, StepState = false;
bool Reverse = false;

uint16_t Table[200] = {0};
uint16_t DTable[125] = {0,    14,   27,   41,   54,   68,   81,   95,   108,  122,  135,  149,
                        163,  176,  190,  203,  217,  230,  244,  257,  271,  285,  298,  312,
                        325,  339,  352,  366,  379,  393,  406,  420,  434,  447,  461,  474,
                        488,  501,  515,  528,  542,  555,  569,  583,  596,  610,  623,  637,
                        650,  664,  677,  691,  705,  718,  732,  745,  759,  772,  786,  799,
                        813,  826,  840,  854,  867,  881,  894,  908,  921,  935,  948,  962,
                        975,  989,  1003, 1016, 1030, 1043, 1057, 1070, 1084, 1097, 1111, 1125,
                        1138, 1152, 1165, 1179, 1192, 1206, 1219, 1233, 1246, 1260, 1274, 1287,
                        1301, 1314, 1328, 1341, 1355, 1368, 1382, 1395, 1409, 1423, 1436, 1450,
                        1463, 1477, 1490, 1504, 1517, 1531, 1545, 1558, 1572, 1585, 1599, 1612,
                        1626, 1639, 1653, 1666, 1680};
uint32_t time_count = 0;

mode m = manual;

SlidingMedian<uint16_t> SMCurIn(7);
SlidingMedian<uint16_t> SMCurOut(7);
SlidingMedian<uint16_t> SMFill(7);
SlidingMedian<uint16_t> SMPress(7);

void RccBusConfig();
void DMAofADCinit();
void ADCinit();
void CANinit();
void TIMinit();
bool CanTxMailBox_IsEmpty(CAN_TypeDef*);

void main()
{
  for(uint8_t i = 0; i < sizeof(Table) / sizeof(*Table); ++i)
  {
    if(i < 20)
      Table[i] = 1000;
    else if(i == 20)
      Table[i] = 105;
    else
      Table[i] = Table[i -1] + 5;
  }

  RccBusConfig();

  GPIO_DeInit(GPIOA);//CAN1, ADC3_ch_1 (токовый)
  GPIO_DeInit(GPIOB);//TIM4(PWM), CAN2
  GPIO_DeInit(GPIOC);//ADC3_ch_10 (11), ADC3_ch_11 (29)

  ADC_DeInit();
  DMA_DeInit(DMA2_Stream0);
  CAN_DeInit(CAN1);
  CAN_DeInit(CAN2);
  TIM_DeInit(TIM2);
  TIM_DeInit(TIM4);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  DMAofADCinit();
  ADCinit();
  CANinit();
  TIMinit();

  while(true)
    __NOP();
}
//Настройка ядра и всей переферии на максимальные частоты от HSE 8 Mhz
void RccBusConfig()
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
//Настройка модуля DMA2 для автоматической обработки каналов АЦП3
void DMAofADCinit()
{
  DMA_DeInit(DMA2_Stream0);
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

  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);//если прерывание ненадо, можно ли убрать
  DMA_Cmd(DMA2_Stream0, ENABLE);

  //Настройка прерывания, без него не работает!
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream0_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x0;
  NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
}
/******************************************************************************
Настройка АЦП3, каналы 1, 10, 11 на преобразование по прерыванию таймера 2 канала 2. Прерывание по спаду и нарастанию. Это необходимо для улучшения отклика системы на управление и фильтрации шумов сигнала.
******************************************************************************/
void ADCinit()
{
  ADC_DeInit();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_StructInit(&GPIO_InitStruct);

  //Ручное заполнение и ток на клапане с внешнего шунта
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1;//Ток на клапане
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_7;//ДД
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

  ADC_RegularChannelConfig(ADC3, ADC_Channel_1,  1, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 2, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 3, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_5,  4, ADC_SampleTime_480Cycles);

  //Запрос после последней передачи, без него не работает
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
  ADC_DMACmd(ADC3, ENABLE);
  ADC_Cmd(ADC3, ENABLE);
  ADC_SoftwareStartConv(ADC3);
}
//Настройка модуля CAN.
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

  //частота шины 42МГц / 24 = 1.75МГц / 7 = 250 кбит/с
  CAN_InitTypeDef CAN_InitStruct;
  CAN_StructInit(&CAN_InitStruct);

  CAN_InitStruct.CAN_ABOM = ENABLE;
  CAN_InitStruct.CAN_TXFP = ENABLE;
  CAN_InitStruct.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStruct.CAN_Prescaler = 24;
  CAN_Init(CAN1, &CAN_InitStruct);
  CAN_Init(CAN2, &CAN_InitStruct);

  CAN_SlaveStartBank(0);//задаем номер фильтра, без этого не работает
  CAN_FilterInitTypeDef CAN_FilterInitStruct;//без фильтра не работает

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
/******************************************************************************
Настройка TIM2 для отправки по CAN: сообщений - заполнение, ток, частоту, время заполнения в авто управлении, режим работы, процент пропуска верхнего предела тока, шаг частоты в авто управлении, количество выборок на каджой частоте; прерывания по спаду и фронту для обработки с помощью DMA каналов АЦП и чтобы за время прерывания таймера 2 успел заполниться класс фильтра.
Настройка TIM4 OC1 для ШИМ, прерывание по переполнению.
******************************************************************************/
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
  GPIO_Init(GPIOC, &GPIO_InitStructure);//TIM3
  GPIO_Init(GPIOB, &GPIO_InitStructure);//TIM4

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
  TIM_TimeBaseInitStruct.TIM_Prescaler = 0;//всегда +1
  TIM_TimeBaseInitStruct.TIM_Period    = 1680;//200 Gz на 250 точек
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

  TIM_TimeBaseInitStruct.TIM_Prescaler = 839;//всегда +1
  TIM_TimeBaseInitStruct.TIM_Period    = 1000;//100 Gz
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

  TIM_TimeBaseInitStruct.TIM_Period    = 100;//1 мс
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

  TIM_OCInitTypeDef TIM_OCInitStruct;
  TIM_OCStructInit(&TIM_OCInitStruct);
  TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM1;
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_Pulse       = 0;
  TIM_OC1Init(TIM3, &TIM_OCInitStruct);
  TIM_OC1Init(TIM4, &TIM_OCInitStruct);

  //Настройка TIM2 ОС2 для сканирования АЦП по прерыванию. Настройка GPIOх ненужна
  TIM_OCInitStruct.TIM_Pulse = 50;//в 2 раза быстрее TIM2
  TIM_OC2Init(TIM2, &TIM_OCInitStruct);
  TIM_SetCounter(TIM2, 0);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);//разрешает загрузку в CCR1
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);//разрешает загрузку в CCR1
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

bool CanTxMailBox_IsEmpty(CAN_TypeDef* CANx)
{
  if((CANx->TSR & CAN_TSR_TME0) == CAN_TSR_TME0 ||
     (CANx->TSR & CAN_TSR_TME1) == CAN_TSR_TME1 ||
     (CANx->TSR & CAN_TSR_TME2) == CAN_TSR_TME2)
    return true;
  else
    return false;
}

void ReinitializeTIM()
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
  TIM_TimeBaseInitStruct.TIM_Prescaler = 839;//всегда +1
  TIM_TimeBaseInitStruct.TIM_Period    = freqt / freq;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
}
void SetNextFreq(uint8_t& cnt)
{
  //частота увеличивается в условии, при изменении кода быть внимательнее!!!
  if(cnt == amount && (freq += stepf) <= maxf) 
  {
    //без переинициализации сбиваются настройки
    ReinitializeTIM();
    count = time * freq;
    cnt   = 0;
  }
  else if(freq > maxf)
  {
    freq = minf;
    m    = idle;
    cnt  = 0;
    TIM_SetCompare1(TIM4, 0);
  }
}
void AutoState()
{
  static uint16_t i    = 0;
  static uint8_t  cnt  = 0;
  static bool     flag = false;

  if(i <= (count - (uint16_t)(count * percent / 100)) && !flag)
  {
    TIM_SetCompare1(TIM4, (uint32_t)(i * ((double)(freqt / freq) / count)));

    if(i++ == (count - (uint16_t)(count * percent / 100)))
      flag = true;
  }
  else
  {
    TIM_SetCompare1(TIM4, (uint32_t)(--i * ((double)(freqt / freq) / count)));

    if(!i)
    {
      flag = false;
      ++cnt;
    }
    SetNextFreq(cnt);
  }
}
void AutoStateMaxCurrent()
{
  static uint16_t i     = 0;
  static bool     flag  = false;
  static bool     flagm = false;
  static uint8_t  cnt   = 0;

  if(i <= (uint16_t)(count * 0.1) && !flag)
  {
    if(!flagm)
    {
      TIM_SetCompare1(TIM4, (uint32_t)(freqt / freq));
      flagm = true;
    }
    if(i++ == (uint16_t)(count * 0.1))
      flag = true;
  }
  else
  {
    if(flagm)
    {
      TIM_SetCompare1(TIM4, 0);
      flagm = false;
    }

    if(!(--i))
    {
      flag = false;
      ++cnt;
    }
    SetNextFreq(cnt);
  }
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
  //Обработчик привязан к TIM2 OC2. Закидываем значения с АЦП в фильтр.
  void DMA2_Stream0_IRQHandler()
  {
    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
    {
      DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
      SMCurIn.push(in_current);
      SMCurOut.push(out_current);
      SMFill.push(filling);
      SMPress.push(pressure);
    }
  }
  /****************************************************************************
  Реализованно 4 режима управления ШИМ сигналом:
  1. Ручной - коэффициент заполнения регулируется с помощью потенциометра.
  2. Автоматическое пропорциональное управление - заполнение ШИМ за определенное (заданное оператором) время, частота изменяется с заданым шагом и проходит заданое число выборок. Так же есть возможность ограничить верхний предел тока в процентах.
  3. Максимальный ток - выставляется максимальный ток на клапане.
  4. Автоматическое дискретное управление - выставляется максимальный ток на заданное время, с заданым числом выборок. Частота изменяется с заданым шагом.
  ****************************************************************************/
  void TIM4_IRQHandler()
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

    if(m == manual)
      TIM_SetCompare1(TIM4,(uint32_t)((double)(freqt/freq)*((double)SMFill.get()/4095)));
    else if(m == auto_p)
      AutoState();
    else if(m == max_cur)
      TIM_SetCompare1(TIM4, (uint32_t)(freqt / freq));
    else if(m == auto_d)
      AutoStateMaxCurrent();
    else if(m == idle);
  }
  void TIM3_IRQHandler()
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    static uint8_t myc = 0;

    if(!Reverse)
    {
      TIM_SetCompare1(TIM3, DTable[myc++]);
      if(myc == 125)
        Reverse = true;
    }
    else
    {
      TIM_SetCompare1(TIM3, DTable[myc-- - 1]);
      if(myc == 0)
        Reverse = false;
    }
  }
  //Для контроля параметров, данные выводятся через CAN.
  void TIM2_IRQHandler()
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    ++time_count;

    CanTxMsg TxMessage;
    TxMessage.RTR     = CAN_RTR_DATA;
    TxMessage.IDE     = CAN_ID_STD;
    TxMessage.DLC     = 8;

    if(!(time_count % 5))
    {
      TxMessage.StdId   = 0x002;
      TxMessage.Data[0] = static_cast<uint8_t>(freq / 10);
      TxMessage.Data[1] = time;
      TxMessage.Data[2] = static_cast<uint8_t>(m);
      TxMessage.Data[3] = percent;
      TxMessage.Data[4] = stepf;
      TxMessage.Data[5] = amount;
      TxMessage.Data[6] = 0;
      TxMessage.Data[7] = 0;
      while(!CanTxMailBox_IsEmpty(CAN2));
      CAN_Transmit(CAN2, &TxMessage);

      TxMessage.StdId   = 0x003;
      TxMessage.Data[0] = static_cast<uint8_t>(SMCurOut.get());
      TxMessage.Data[1] = static_cast<uint8_t>(SMCurOut.get() >> 8);
      TxMessage.Data[2] = static_cast<uint8_t>(SMCurIn.get());
      TxMessage.Data[3] = static_cast<uint8_t>(SMCurIn.get()  >> 8);
      TxMessage.Data[4] = static_cast<uint8_t>(SMFill.get());
      TxMessage.Data[5] = static_cast<uint8_t>(SMFill.get()   >> 8);
      TxMessage.Data[6] = static_cast<uint8_t>(SMPress.get());
      TxMessage.Data[7] = static_cast<uint8_t>(SMPress.get()  >> 8);
      while(!CanTxMailBox_IsEmpty(CAN2));
      CAN_Transmit(CAN2, &TxMessage);

      if(PropState)
        ProportionalSet();
      if(StepState)
        StepSetCurrent();
    }
    if(!(time_count % 10))
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
      while(!CanTxMailBox_IsEmpty(CAN2));
      CAN_Transmit(CAN2, &TxMessage);
    }
    if(!(time_count % 20))
    {
      TxMessage.StdId   = 0x005;
      TxMessage.Data[0] = static_cast<uint8_t>(SMCurOut.get());
      TxMessage.Data[1] = static_cast<uint8_t>(SMCurOut.get() >> 8);
      TxMessage.Data[2] = static_cast<uint8_t>(SMCurIn.get());
      TxMessage.Data[3] = static_cast<uint8_t>(SMCurIn.get()  >> 8);
      TxMessage.Data[4] = static_cast<uint8_t>(SMFill.get());
      TxMessage.Data[5] = static_cast<uint8_t>(SMFill.get()   >> 8);
      TxMessage.Data[6] = static_cast<uint8_t>(SMPress.get());
      TxMessage.Data[7] = static_cast<uint8_t>(SMPress.get()  >> 8);
      while(!CanTxMailBox_IsEmpty(CAN2));
      CAN_Transmit(CAN2, &TxMessage);
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
        freq    = RxMessage.Data[0] * 10;
        time    = RxMessage.Data[1];
        m       = static_cast<mode>(RxMessage.Data[2]);
        percent = RxMessage.Data[3];
        stepf   = RxMessage.Data[4];
        amount  = RxMessage.Data[5];
        timecur = RxMessage.Data[6];
        //без переинициализации сбиваются настройки
        ReinitializeTIM();
        count = time * freq;
      }
      else if(RxMessage.StdId == 0x010)
        PropState = true;
      else if(RxMessage.StdId == 0x011)
        TIM_SetCompare1(TIM4, 0);
      else if(RxMessage.StdId == 0x012)
        StepState = true;
    }
  }
}