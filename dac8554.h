#ifndef __DAC8554_H
#define __DAC8554_H

/*
*********************************************************************************************************
*                                             
*	模块名称 : DAC8554 驱动模块
*	版    本 : V1.1
* 作    者 : 张豪洋
*	说    明 : 模块和CPU之间采用SPI接口。
*********************************************************************************************************
*/
/************************ 必要头文件包含 ************************/
#include "stdint.h"                   // 提供uint8_t/uint16_t等类型定义
#include "stm32g4xx_hal.h"            // 替换为实际MCU型号（如stm32f4xx_hal.h/stm32g4xx_hal.h）
#include "math.h"                     // 兼容roundf浮点函数



/************************ 工具函数声明 ************************/
/**
 * @brief  浮点数电压转换为DAC8554的16位数字码
 * @param  Vref: DAC参考电压（单位：V）
 * @param  Voltage: 目标输出电压（单位：V，范围0~Vref）
 * @retval 16位无符号数字码（0~65535）
 */
uint16_t DAC8554_VoltagetoData(float Vref, float Voltage);

/************************ 示例1：同步更新函数（写缓冲后批量更新）************************/
/**
 * @brief  写通道A缓冲（仅存数据，不更新输出）
 * @param  Voltage: 目标电压（0~DAC8554_VREF）
 * @retval 无
 */
void DAC_WriteA_Buffer(float Voltage);

/**
 * @brief  写通道B缓冲（仅存数据，不更新输出）
 * @param  Voltage: 目标电压（0~DAC8554_VREF）
 * @retval 无
 */
void DAC_WriteB_Buffer(float Voltage);

/**
 * @brief  写通道C缓冲（仅存数据，不更新输出）
 * @param  Voltage: 目标电压（0~DAC8554_VREF）
 * @retval 无
 */
void DAC_WriteC_Buffer(float Voltage);

/**
 * @brief  写通道D缓冲并同步更新所有通道（A/B/C/D同时输出）
 * @param  Voltage: 通道D目标电压（0~DAC8554_VREF）
 * @retval 无
 */
void DAC_WriteD_Buffer_SyncAll(float Voltage);

/************************ 示例2：顺序更新函数（写缓冲后立即更新）************************/
/**
 * @brief  写入通道A并立即更新输出
 * @param  Voltage: 目标电压（0~DAC8554_VREF）
 * @retval 无
 */
void DAC_WriteA_Immediate(float Voltage);

/**
 * @brief  写入通道B并立即更新输出
 * @param  Voltage: 目标电压（0~DAC8554_VREF）
 * @retval 无
 */
void DAC_WriteB_Immediate(float Voltage);

/**
 * @brief  写入通道C并立即更新输出
 * @param  Voltage: 目标电压（0~DAC8554_VREF）
 * @retval 无
 */
void DAC_WriteC_Immediate(float Voltage);

void DAC_WriteD_Immediate(float Voltage);
void DAC8554_WriteA(float Voltage);                  // 写通道A
void DAC8554_WriteB(float Voltage);                  // 写通道B
void DAC8554_WriteC(float Voltage);                  // 写通道C
void DAC8554_WriteD(float Voltage);                  // 写通道D
//void DAC_WriteA(float Voltage);                  // 写通道A
//void DAC_WriteB(float Voltage);                  // 写通道B
//void DAC_WriteC(float Voltage);                  // 写通道C
//void DAC_WriteD(float Voltage);                  // 写通道D
#endif /* __DAC8554_H */
