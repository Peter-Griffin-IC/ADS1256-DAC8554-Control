/*
*********************************************************************************************************
*                                             
*	模块名称 : DAC8554 驱动模块
*	版    本 : V1.1
* 作    者 : 张豪洋
*	说    明 : 模块和CPU之间采用SPI接口。
*********************************************************************************************************
*/


#include "spi.h"
#include "gpio.h"
#include "stdint.h"
#include "math.h"
#include "dac8554.h"

/************************ 控制位宏定义（严格对齐图片示例）************************/
// DAC8554 24位控制字拆分（Bit23~Bit16）：
// Bit23:A1 | Bit22:A0 | Bit21:LD1 | Bit20:LD0 | Bit19:DC | Bit18:DAC Sel1 | Bit17:DAC Sel0 | Bit16:PD0
// PD1=Bit15（数据位最高位？不，图片中PD仅PD0，PD1=0；掉电模式默认00=正常工作）
#define DAC8554_A1           0x00    // A1位（多芯片级联用，单芯片设0）
#define DAC8554_A0           0x00    // A0位（单芯片设0）

// LD1/LD0：加载DAC控制位（核心，对应图片示例）
#define DAC8554_LD_NONE      0x00    // LD1=0, LD0=0：仅写缓冲，不更新DAC（示例1前3步）
#define DAC8554_LD_IMMEDIATE 0x01    // LD1=0, LD0=1：写缓冲后立即更新对应DAC（示例2）
#define DAC8554_LD_ALL       0x02    // LD1=1, LD0=0：写缓冲后同步更新所有DAC（示例1第4步）

// DAC Sel1/Sel0：通道选择（对应图片）
#define DAC8554_SEL_A        0x00    // DAC Sel1=0, Sel0=0 → 通道A
#define DAC8554_SEL_B        0x01    // DAC Sel1=0, Sel0=1 → 通道B
#define DAC8554_SEL_C        0x02    // DAC Sel1=1, Sel0=0 → 通道C
#define DAC8554_SEL_D        0x03    // DAC Sel1=1, Sel0=1 → 通道D

// DC位：广播位（多芯片同步用，单芯片设0）
#define DAC8554_DC_SINGLE    0x00    // 单芯片模式
// PD0位：掉电模式（0=正常工作，对应图片PD0=0）
#define DAC8554_PD_NORMAL    0x00

/************************ 硬件配置宏（用户根据实际修改）************************/
#define DAC8554_VREF         4.7f    // DAC参考电压（5V）
#define DAC8554_SPI_TIMEOUT  10      // SPI传输超时时间（10ms）
#define DAC8554_CS_PIN       GPIO_PIN_12
#define DAC8554_CS_PORT      GPIOB
#define DAC8554_SPI_HANDLE   &hspi2  // SPI句柄

/************************ 工具函数：电压转16位DAC数据 ************************/
/**
 * @brief  浮点数电压转换为DAC8554的16位数字码
 * @param  Vref: DAC参考电压（单位：V）
 * @param  Voltage: 目标输出电压（单位：V，范围0~Vref）
 * @retval 16位无符号数字码（0~65535）
 */
uint16_t DAC8554_VoltagetoData(float Vref, float Voltage)
{
    // 电压范围保护，避免溢出
    if (Voltage > Vref) Voltage = Vref;
    if (Voltage < 0) Voltage = 0;
    
    // DAC8554为16位单极性DAC，公式：D = (Voltage / Vref) * 65535
    return (uint16_t)roundf((65535.0f * Voltage) / Vref);
}

/************************ 底层函数：组装24位控制字并写入 ************************/
/**
 * @brief  组装24位控制字并通过SPI写入DAC8554
 * @param  ld_mode: 加载模式（LD_NONE/LD_IMMEDIATE/LD_ALL，对应LD1/LD0）
 * @param  dac_sel: 通道选择（SEL_A/SEL_B/SEL_C/SEL_D）
 * @param  data: 16位DAC数据（0~65535）
 * @retval 无
 */
static void DAC8554_Write_Control(uint8_t ld_mode, uint8_t dac_sel, uint16_t data)
{
    uint8_t tx_buf[3] = {0};
    // 组装24位控制字的高8位（Bit23~Bit16）
    tx_buf[0] = (DAC8554_A1 << 7) | (DAC8554_A0 << 6) | (ld_mode << 4) | 
                (DAC8554_DC_SINGLE << 3) | (dac_sel << 1) | DAC8554_PD_NORMAL;
    // 16位数据拆分为高8位+低8位
    tx_buf[1] = (uint8_t)(data >> 8)& 0xFF;   // 数据高8位
    tx_buf[2] = (uint8_t)(data & 0xFF); // 数据低8位

    // SPI时序：片选拉低→传输→片选拉高
    HAL_GPIO_WritePin(DAC8554_CS_PORT, DAC8554_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(DAC8554_SPI_HANDLE, tx_buf, 3, DAC8554_SPI_TIMEOUT);
    HAL_GPIO_WritePin(DAC8554_CS_PORT, DAC8554_CS_PIN, GPIO_PIN_SET);
}

/************************ 示例1：同步更新（写缓冲后批量更新所有通道）************************/
/**
 * @brief  写通道A缓冲（仅存数据，不更新输出）
 * @param  Voltage: 目标电压（0~5V）
 * @retval 无
 */
void DAC_WriteA_Buffer(float Voltage)
{
    uint16_t dac_data = DAC8554_VoltagetoData(DAC8554_VREF, Voltage);
    DAC8554_Write_Control(DAC8554_LD_NONE, DAC8554_SEL_A, dac_data);
}

/**
 * @brief  写通道B缓冲（仅存数据，不更新输出）
 * @param  Voltage: 目标电压（0~5V）
 * @retval 无
 */
void DAC_WriteB_Buffer(float Voltage)
{
    uint16_t dac_data = DAC8554_VoltagetoData(DAC8554_VREF, Voltage);
    DAC8554_Write_Control(DAC8554_LD_NONE, DAC8554_SEL_B, dac_data);
}

/**
 * @brief  写通道C缓冲（仅存数据，不更新输出）
 * @param  Voltage: 目标电压（0~5V）
 * @retval 无
 */
void DAC_WriteC_Buffer(float Voltage)
{
    uint16_t dac_data = DAC8554_VoltagetoData(DAC8554_VREF, Voltage);
    DAC8554_Write_Control(DAC8554_LD_NONE, DAC8554_SEL_C, dac_data);
}

/**
 * @brief  写通道D缓冲并同步更新所有通道（示例1核心步骤）
 * @param  Voltage: 通道D目标电压（0~5V）
 * @retval 无
 */
void DAC_WriteD_Buffer_SyncAll(float Voltage)
{
    uint16_t dac_data = DAC8554_VoltagetoData(DAC8554_VREF, Voltage);
    // LD_ALL模式：写D缓冲后，所有通道（A/B/C/D）同步更新输出
    DAC8554_Write_Control(DAC8554_LD_ALL, DAC8554_SEL_D, dac_data);
}

/************************ 示例2：顺序更新（写缓冲后立即更新对应通道）************************/
/**
 * @brief  写入通道A并立即更新输出（示例2）
 * @param  Voltage: 目标电压（0~5V）
 * @retval 无
 */
void DAC_WriteA_Immediate(float Voltage)
{
    uint16_t dac_data = DAC8554_VoltagetoData(DAC8554_VREF, Voltage);
    // LD_IMMEDIATE模式：写A缓冲后立即更新A通道输出
    DAC8554_Write_Control(DAC8554_LD_IMMEDIATE, DAC8554_SEL_A, dac_data);
}

/**
 * @brief  写入通道B并立即更新输出（示例2）
 * @param  Voltage: 目标电压（0~5V）
 * @retval 无
 */
void DAC_WriteB_Immediate(float Voltage)
{
    uint16_t dac_data = DAC8554_VoltagetoData(DAC8554_VREF, Voltage);
    DAC8554_Write_Control(DAC8554_LD_IMMEDIATE, DAC8554_SEL_B, dac_data);
}

/**
 * @brief  写入通道C并立即更新输出（示例2）
 * @param  Voltage: 目标电压（0~5V）
 * @retval 无
 */
void DAC_WriteC_Immediate(float Voltage)
{
    uint16_t dac_data = DAC8554_VoltagetoData(DAC8554_VREF, Voltage);
    DAC8554_Write_Control(DAC8554_LD_IMMEDIATE, DAC8554_SEL_C, dac_data);
}

/**
 * @brief  写入通道D并立即更新输出（示例2）
 * @param  Voltage: 目标电压（0~5V）
 * @retval 无
 */
void DAC_WriteD_Immediate(float Voltage)
{
    uint16_t dac_data = DAC8554_VoltagetoData(DAC8554_VREF, Voltage);
    DAC8554_Write_Control(DAC8554_LD_IMMEDIATE, DAC8554_SEL_D, dac_data);
}

/************************ 兼容原有简单写入函数（默认顺序更新）************************/
void DAC8554_WriteA(float Voltage)  
{
    DAC_WriteA_Immediate(Voltage);
}

void DAC8554_WriteB(float Voltage) 
{
    DAC_WriteB_Immediate(Voltage);
}

void DAC8554_WriteC(float Voltage) 
{
    DAC_WriteC_Immediate(Voltage);
}

void DAC8554_WriteD(float Voltage)
{
    DAC_WriteD_Immediate(Voltage);
}

//void DAC_WriteA(float Voltage)  
//{
//    DAC_WriteA_Immediate(Voltage);
//}

//void DAC_WriteB(float Voltage) 
//{
//    DAC_WriteB_Immediate(Voltage);
//}

//void DAC_WriteC(float Voltage) 
//{
//    DAC_WriteC_Immediate(Voltage);
//}

//void DAC_WriteD(float Voltage)
//{
//    DAC_WriteD_Immediate(Voltage);
//}
