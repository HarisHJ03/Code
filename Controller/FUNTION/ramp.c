#include "ramp.h"

void ramp_init(ramp_t *ramp,uint32_t set_cnt)
{
  ramp->get_count = 0;
  ramp->out = 0;
  ramp->set_count = set_cnt;
}


float ramp_calc(ramp_t *ramp)
{
  if(ramp->set_count <= 0)
    return 0;
  if(ramp->get_count >= ramp->set_count)
    ramp->get_count = ramp->set_count;
  else
    ramp->get_count++;
  
  ramp->out = (float)ramp->get_count/(float)ramp->set_count;
  return ramp->out;
}

//#include "ramp.h"

//void ramp_init(ramp_t *ramp, int32_t scale)
//{
//  ramp->count = 0;
//  ramp->scale = scale;
//}

//float ramp_calc(ramp_t *ramp)
//{
//  if (ramp->scale <= 0)
//    return 0;
//  
//  if (ramp->count++ >= ramp->scale)
//    ramp->count = ramp->scale;
//  
//  ramp->out = ramp->count / ((float)ramp->scale);		// task_period / n
//  return ramp->out;
//}
