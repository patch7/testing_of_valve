#include "testingvalve.h"

void TestingValve::AutoProp()
{
  if(!flag)
  {
    TIM_SetCompare1(TIM4, icount * static_cast<double>(freqt / freq) / count);
    if(icount++ == (count - count * percent / 100))
      flag = true;
  }
  else
  {
    TIM_SetCompare1(TIM4, --icount * static_cast<double>(freqt / freq) / count);
    if(icount == 0)
    {
      flag = false;
      ++amount_cnt;
      NextFrequency();
    }
  }
}
void TestingValve::AutoMaxCur()
{
  static bool flagm = false;

  if(!flag)
  {
    if(!flagm)
    {
      TIM_SetCompare1(TIM4, freqt / freq);
      flagm = true;
    }
    if(icount++ == count / 10)
      flag = true;
  }
  else
  {
    if(flagm)
    {
      TIM_SetCompare1(TIM4, 0);
      flagm = false;
    }
    if(--icount == 0)
    {
      flag = false;
      ++amount_cnt;
      NextFrequency();
    }
  }
}
void TestingValve::NextFrequency()
{
  //частота увеличивается в условии, при изменении кода быть внимательнее!!!
  if(amount_cnt == amount && (freq += stepf) <= maxf)
  {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;//без переинициализации сбиваются настройки
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
    TIM_TimeBaseInitStruct.TIM_Prescaler = 839;//всегда +1
    TIM_TimeBaseInitStruct.TIM_Period    = freqt / freq;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
    count = time * freq;
    amount_cnt = 0;
  }
  else if(freq > maxf)
  {
    freq = minf;
    m    = idle;
    amount_cnt = 0;
    TIM_SetCompare1(TIM4, 0);
  }
}
void TestingValve::SetData(CanRxMsg& msg)
{
  freq    = msg.Data[0] * 10;
  time    = msg.Data[1];
  m       = static_cast<mode>(msg.Data[2]);
  percent = msg.Data[3];
  stepf   = msg.Data[4];
  amount  = msg.Data[5];

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;//без переинициализации сбиваются настройки
  TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
  TIM_TimeBaseInitStruct.TIM_Prescaler = 839;//всегда +1
  TIM_TimeBaseInitStruct.TIM_Period    = freqt / freq;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
  count = time * freq;

  if(m == idle)
    TIM_SetCompare1(TIM4, 0);
}
void TestingValve::Run(const double fill)
{
  if(m == manual)
    Manual(fill);
  else if(m == auto_p)
    AutoProp();
  else if(m == max_cur)
    MaxCurrent();
  else if(m == auto_d)
    AutoMaxCur();
  else if(m == idle);
}
void TestingValve::SendMsg() const
{
  CanTxMsg TxMessage;
  TxMessage.RTR     = CAN_RTR_DATA;
  TxMessage.IDE     = CAN_ID_STD;
  TxMessage.DLC     = 6;
  TxMessage.StdId   = 0x002;
  TxMessage.Data[0] = static_cast<uint8_t>(freq / 10);
  TxMessage.Data[1] = time;
  TxMessage.Data[2] = static_cast<uint8_t>(m);
  TxMessage.Data[3] = percent;
  TxMessage.Data[4] = stepf;
  TxMessage.Data[5] = amount;
  while(!CanTxMailBoxIsEmpty(CAN2));
  CAN_Transmit(CAN2, &TxMessage);
}
void TestingValve::Dithering()
{
  static uint16_t my_icount = 0;
  const  uint16_t per       = 1680;
  const  uint16_t points    = 12600;

  if(!flag)
  {
    TIM_SetCompare1(TIM3, my_icount * static_cast<double>(per) / points);
    if(my_icount++ == points)
      flag = true;
  }
  else
  {
    TIM_SetCompare1(TIM3, --my_icount * static_cast<double>(per) / points);
    if(my_icount == 0)
      flag = false;
  }
}
void TestingValve::DitheringSinus()
{
  if(!Reverse)
  {
    TIM_SetCompare1(TIM3, DTab[dith_sin_count++]);
    if(dith_sin_count == 25)
      Reverse = true;
  }
  else
  {
    TIM_SetCompare1(TIM3, DTab[--dith_sin_count]);
    if(dith_sin_count == 0)
      Reverse = false;
  }
}