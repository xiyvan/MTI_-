
#include "CHASSIS_POWER.h"
#include "mk_task.h"
#include "REMOTE_task.h"


extern Remote_angle_t remote_angle;



void Power_limted(s16* date)
{
    float x = 1.0f;
    if(remote_angle.huan <= 60 && remote_angle.huan > 40)
    {
        x = 0.8;
    }
    else if(remote_angle.huan <= 40 && remote_angle.huan > 20)
    {
        x = 0.4;
    }
    else if(remote_angle.huan < 20)
    {
        x = 0.1;
    }


    for(u8 i = 0;i < 4;i++)
    {
        date[i] *= x;
    }
}





