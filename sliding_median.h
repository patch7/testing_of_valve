#ifndef __SLIDING
#define __SLIDING

#include "stm32f4xx.h"
#include <deque>
#include <algorithm>

template<typename Any>
class SlidingMedian : public std::deque<Any>
{
public:
  SlidingMedian(uint32_t cnt = 7) : std::deque<Any>(), count(cnt) {}
  SlidingMedian(const SlidingMedian&)            = delete;
  SlidingMedian(SlidingMedian&&)                 = delete;
  SlidingMedian& operator=(const SlidingMedian&) = delete;
  SlidingMedian& operator=(SlidingMedian&&)      = delete;
  inline void push(const Any&);
  Any get();
  ~SlidingMedian()                               = default;
private:
  uint32_t count;
  bool change = false;
  Any mediane;
};

template<typename Any> void SlidingMedian<Any>::push(const Any& a)
{
  if(count == std::deque<Any>::size())
    std::deque<Any>::pop_front();
  std::deque<Any>::push_back(a);
  change = true;
}

template<typename Any> Any SlidingMedian<Any>::get()
{
  if(change)
  {
    std::deque<Any> d(*this);
    std::sort(d.begin(), d.end());
    mediane = *(d.begin() + d.size() / 2);
    change = false;
  }
  return mediane;
}

#endif /* __SLIDING */