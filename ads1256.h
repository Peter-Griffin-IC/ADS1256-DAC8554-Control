/*
*********************************************************************************************************
*                                                
*	模块名称 : ADS1256 驱动模块(8通道带PGA的24位ADC)
*	文件名称 : bsp_ads1256.h
* 制 作 者 ：张 豪 洋
*********************************************************************************************************
*/
/*
    ADS1256   STM32G431CBT6
      +5V   <------  5.0V      5V
      GND   -------  GND       

      DRDY  ------>  PB6       
      CS    <------  PA15       SPI_CS
      DIN   <------  PB5       SPI_MOSI
      DOUT  ------>  PB4       SPI_MISO
      SCLK  <------  PB3       SPI
      GND   -------  GND       
      RST   <------  PA10       
			SYNC  <------  PA0       

*/
#ifndef __ADS1256_H
#define __ADS1256_H
#include "main.h"
#include "spi.h"
#include "stdio.h"

#define	u8 unsigned char
#define	u16 unsigned int
#define	u32 unsigned long

#define ADS_CS_LOW()    HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_RESET)
#define ADS_CS_HIGH()   HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_SET)

#define ADS_RST_LOW()   HAL_GPIO_WritePin(ADS_RST_GPIO_Port, ADS_RST_Pin, GPIO_PIN_RESET)
#define ADS_RST_HIGH()  HAL_GPIO_WritePin(ADS_RST_GPIO_Port, ADS_RST_Pin, GPIO_PIN_SET)

#define ADS_SYNC_LOW()    HAL_GPIO_WritePin(ADS_SYNC_GPIO_Port, ADS_SYNC_Pin, GPIO_PIN_RESET)
#define ADS_SYNC_HIGH()   HAL_GPIO_WritePin(ADS_SYNC_GPIO_Port, ADS_SYNC_Pin, GPIO_PIN_SET)

#define DRDY_IS_LOW()		(HAL_GPIO_ReadPin(ADS_DRDY_GPIO_Port, ADS_DRDY_Pin) == GPIO_PIN_RESET)

/* 增益选项 */
typedef enum
{
	ADS1256_GAIN_1			= (0),	/* 增益1（缺省） */
	ADS1256_GAIN_2			= (1),	/* 增益2 */
	ADS1256_GAIN_4			= (2),	/* 增益4 */
	ADS1256_GAIN_8			= (3),	/* 增益8 */
	ADS1256_GAIN_16			= (4),	/* 增益16 */
	ADS1256_GAIN_32			= (5),	/* 增益32 */
	ADS1256_GAIN_64			= (6),	/* 增益64 */
}ADS1256_GAIN_E;



/* 采样速率选项 */
typedef enum
{
	ADS1256_30000SPS = 0,
	ADS1256_15000SPS,
	ADS1256_7500SPS,
	ADS1256_3750SPS,
	ADS1256_2000SPS,
	ADS1256_1000SPS,
	ADS1256_500SPS,
	ADS1256_100SPS,
	ADS1256_60SPS,
	ADS1256_50SPS,
	ADS1256_30SPS,
	ADS1256_25SPS,
	ADS1256_15SPS,
	ADS1256_10SPS,
	ADS1256_5SPS,
	ADS1256_2d5SPS,

	ADS1256_DRATE_MAX
}ADS1256_DRATE_E;

#define ADS1256_DRAE_COUNT = 15;

typedef struct
{
	ADS1256_GAIN_E Gain;			/* 增益 */
	ADS1256_DRATE_E DataRate;	/* 数据输出速率 */
	int32_t AdcNow[8];				/* 8路ADC采集结果（实时）有符号数 */
	uint8_t Channel;					/* 当前通道 */
	uint8_t ScanMode;					/* 扫描模式，0表示单端8路， 1表示差分4路 */
}ADS1256_VAR_T;

void bsp_InitADS1256(void);
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);
void ADS1256_DelayDATA(void);
void ADS1256_WriteCmd(uint8_t _cmd);
uint8_t ADS1256_ReadChipID(void);
void ADS1256_StartScan(uint8_t _ucScanMode);
void ADS1256_StopScan(void);
int32_t ADS1256_GetAdc(uint8_t _ch);
void ADS1256_SetDiffChannal(uint8_t _ch);
extern ADS1256_VAR_T g_tADS1256;

void ADS1256_ISR(void);

#endif
/*
*********************************************************************************************************
*                                             
*	模块名称 : ADS1256 驱动模块(8通道带PGA的24位ADC)
*	版    本 : V1.1
* 作    者 : 张豪洋
*	说    明 : ADS1256模块和CPU之间采用SPI接口。
*********************************************************************************************************
*/
