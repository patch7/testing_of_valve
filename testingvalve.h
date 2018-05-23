#ifndef __TESTINGVALVE
#define __TESTINGVALVE

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"

class TestingValve
{
public:
  enum mode{manual, max_cur, auto_p, auto_d, idle};

  TestingValve()                               = default;
  TestingValve(const TestingValve&)            = delete;
  TestingValve(TestingValve&&)                 = delete;
  TestingValve& operator=(const TestingValve&) = delete;
  TestingValve& operator=(TestingValve&&)      = delete;
  ~TestingValve()                              = default;

  void SetData(CanRxMsg&);
  void Run(const double);
  void SendMsg() const;

  void Dithering();
  void DitheringSinus();
private:
  void Manual(const double) const;
  void AutoProp();
  void MaxCurrent() const;
  void AutoMaxCur();
  void NextFrequency();

  const uint32_t freqt = 100000;
  const uint16_t bit12 = 4095;
  const uint16_t maxf  = 400;
  const uint16_t minf  = 100;

  uint16_t DTab[25] = {0,    70,   140,  210,  280,  350,  420,  490,  560,  630,  700,  770,  840,
                       910,  980,  1050, 1120, 1190, 1260, 1330, 1400, 1470, 1540, 1610, 1680};

  uint16_t count  = 0;
  uint16_t icount = 0;

  uint16_t freq    = minf;
  uint8_t  time    = 2;
  uint8_t  stepf   = 25;
  uint8_t  amount  = 2;
  uint8_t  percent = 0;

  uint8_t  dith_sin_count = 0;
  uint8_t  amount_cnt     = 0;

  mode m;

  bool Reverse = false;
  bool flag    = false;
};

inline void TestingValve::Manual(const double fill) const
{
  TIM_SetCompare1(TIM4, static_cast<double>(freqt / freq) * (fill / bit12));
}
inline void TestingValve::MaxCurrent() const       { TIM_SetCompare1(TIM4, freqt / freq); }

inline bool CanTxMailBoxIsEmpty(const CAN_TypeDef* CANx)
{
  return ((CANx->TSR & CAN_TSR_TME0) ==CAN_TSR_TME0 || (CANx->TSR & CAN_TSR_TME1) ==CAN_TSR_TME1 ||
          (CANx->TSR & CAN_TSR_TME2) ==CAN_TSR_TME2);
}
#endif //__TESTINGVALVE