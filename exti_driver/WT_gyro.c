/**********************************
 * @author 韩昂轩
 * @brief 维特陀螺仪数据解码
 * 时间：23.12.04
 *      1.0.1   维特数据解码函数 加速度 角速度 角度解码，，这是更新过后的，，第一次没有记录时间忘记怎么写的了。
************************************/

#include "WT_gyro.h"


WT_gyro_t wt_data_main;

void WT_gyro_decode(u8* data,WT_gyro_t* wt_data)
{
    if(data[0] == 0x55)
    {
        if(data[1] == WT_DATA_TYPE_ACCLE)           //如果数据类型为加速度
        {
            wt_data->ax = (short)((short)data[3] << 8 | data[2]) / WT_DECODE_COEFFIENT * 16.0f; // 角速度数据解码
            wt_data->ay = (short)((short)data[5] << 8 | data[4]) / WT_DECODE_COEFFIENT * 16.0f;
            wt_data->az = (short)((short)data[7] << 8 | data[6]) / WT_DECODE_COEFFIENT * 16.0f;
            wt_data->temputer = ((short)data[9] << 8 | data[8]) / 100;                    //温度数据
            if(data[11] == 0x55)
            {
                wt_data->wx = (short)((short)data[14] << 8 | data[13]) / WT_DECODE_COEFFIENT * 2000.0f* 0.01745329252;
                wt_data->wy = (short)((short)data[16] << 8 | data[15]) / WT_DECODE_COEFFIENT * 2000.0f* 0.01745329252;
                wt_data->wz = (short)((short)data[18] << 8 | data[17]) / WT_DECODE_COEFFIENT * 2000.0f* 0.01745329252; //角速度解码
            }
            if(data[22] == 0x55)
            {
                wt_data->roll = (short)((short)data[25] << 8 | data[24]) / WT_DECODE_COEFFIENT * 180.0f * 0.01745329252;    // 角度解码
                wt_data->pitch = (short)((short)data[27] << 8 | data[26]) / WT_DECODE_COEFFIENT * 180.0f * 0.01745329252 ;
                wt_data->yaw = 180+(short)((short)data[29] << 8 | data[28]) / WT_DECODE_COEFFIENT * 180.0f;
            }
        }
    }
}

