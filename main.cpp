/******************************************************************************
  Ядро и переферию настроили на максимальную частоту. Настройка каналов АЦП для обработки органов управления, обратной связи. Настройка таймера для генерации ШИМ с автоматическим регулирование частоты от 100 до 500 Гц с возможностью настройки шага изменения частоты. Настройка ПДП (ДМА) для автоматической обработки данных с АЦП (ручное заполнение, ток на клапане, ток на клапане с внешнего шунта).
  По кану принимаем настройки системы: режим работы (ручной, автоматическое пропорциональное управление, автоматическое дискретное управление, максимальный ток); ручное управление частотой ШИМ; время заполнения ШИМ в авто управлении; процент пропуска верхнего предела тока; шаг частоты в авто управлении; количество выборок на каждой частоте.
  Для диагностики и фиксации процессов выводим по кану: заполнение ШИМ; ток; частоту; время заполнения ШИМ в авто управлении; режим работы; процент пропуска верхнего предела тока; шаг частоты в авто управлении; количество выборок на каджой частоте.
  Для фильтрации шумов с АЦП используем класс скользящей медианы.

  Разработал Михайлов А.В.
  Дата изменения 14.09.2017

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
#include "sliding_median.h"

#define in_current  ADCValue[0]
#define out_current ADCValue[1]
#define filling     ADCValue[2]

const static uint32_t freq_tim = 100000;

const static uint16_t byte     = 256;
const static uint16_t max_f    = 500;
const static uint16_t min_f    = 100;

static uint16_t ADCValue[3] = {0};
static uint16_t freq        = 0;
static uint16_t count       = 0;
static uint8_t  time        = 0;
static uint8_t  step_freq   = 0;
static uint8_t  amount      = 1;
static uint8_t  percent     = 0;

typedef enum mode{manual, max_cur, auto_p, auto_d} m;

SlidingMedian<uint16_t> SMCurIn(9);
SlidingMedian<uint16_t> SMCurOut(9);
SlidingMedian<uint16_t> SMFill(9);

void RccBusConfig();
void DMAofADCinit();
void ADCinit();
void CANinit();
void TIMinit();

void main()
{
  //RccBusConfig();

  GPIO_DeInit(GPIOA);//CAN1, ADC3_ch_1
  GPIO_DeInit(GPIOB);//TIM4(PWM), CAN2
  GPIO_DeInit(GPIOC);//ADC3_ch_10, ADC3_ch_11

  ADC_DeInit();
  DMA_DeInit(DMA2_Stream0);
  CAN_DeInit(CAN1);
  CAN_DeInit(CAN2);
  TIM_DeInit(TIM2);
  TIM_DeInit(TIM4);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  //DMAofADCinit();
  //ADCinit();
  //CANinit();
  //TIMinit();

  while(true)
    __NOP();
}
