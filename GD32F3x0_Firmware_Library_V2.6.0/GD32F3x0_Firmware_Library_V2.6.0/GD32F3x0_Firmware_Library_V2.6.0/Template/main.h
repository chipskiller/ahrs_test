/*
 * @Author: zhou 3144608802@qq.com
 * @Date: 2026-07-14 20:58:17
 * @LastEditors: zhou 3144608802@qq.com
 * @LastEditTime: 2026-07-14 22:31:32
 * @FilePath: \Keil5_projectc:\download\GD32F3x0_Firmware_Library_V2.6.0\GD32F3x0_Firmware_Library_V2.6.0\GD32F3x0_Firmware_Library_V2.6.0\Template\main.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*!
    \file    main.h
    \brief   the header file of main

    \version 2026-01-01, V2.6.0, firmware for GD32F3x0
*/

/*
    Copyright (c) 2026, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

typedef struct {
  float w, x, y, z;
} quaternion_t;

// 陀螺温度补偿零偏参数
typedef struct {
  float gz_bias;  // Z轴陀螺静态零偏
  float temp_ref; // 标定时基准温度
} gyro_bias_t;

/* 姿态信息结构体 */
typedef struct {
  float pitch;      /*!< 实时俯仰角 X轴 */
  float roll;       /*!< 实时横滚角 Y轴 */
  float yaw_now;    /*!< 实时旋转角 Z轴 */
  float pitch_base; /*!< 安装基准俯仰零点 */
  float roll_base;  /*!< 安装基准横滚零点 */
  float yaw_base;   /*!< 安装基准旋转零点 */
} attitude_info_t;

/* 磁力计原始数据结构体 */
typedef struct {
  float mx;
  float my;
  float mz;
  float mag_norm;
} mag_raw_data_t;

// IMU原始传感器数据
typedef struct {
  float ax;   // X轴加速度 g
  float ay;   // Y轴加速度 g
  float az;   // Z轴加速度 g
  float gx;   // X轴角速度 °/s
  float gy;   // Y轴角速度 °/s
  float gz;   // Z轴角速度 °/s
  float temp; // 芯片温度 ℃
} icm_raw_data_t;

/* 全局变量声明（定义在 main.c） */
extern icm_raw_data_t icm_raw;
extern mag_raw_data_t mag_raw;
extern attitude_info_t att;
extern gyro_bias_t gyro_bias;
extern quaternion_t quat;

extern uint8_t day_mode;                 // 1=白天6轴模式 0=夜间9轴融合
extern uint8_t mag_disturb_flag;         // 地磁受大车干扰标记
extern uint8_t fault_type;               // 偏转报警类型标记
// extern uint16_t alarm_filter_cnt; // 报警防抖计数器
extern uint32_t stable_cnt;
// extern volatile uint8_t imu_loop_flag; // 定时器中断标志

void imu_main_loop(uint8_t rtc_hour);
int save_install_zero_point(void);

#endif /* MAIN_H */
