#ifndef __nRF24L01P__
#define __nRF24L01P__

#include "main.h"
#include "spi.h"
#include "gpio.h"

//nRF24L01+ PIN DEFINITION
//拉低片选
#define	CS_LOW         HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port,SPI1_CSN_Pin,GPIO_PIN_RESET);
//拉高片选
#define CS_HIGH        HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port,SPI1_CSN_Pin,GPIO_PIN_SET);
//拉低使能
#define	CE_LOW         HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin,GPIO_PIN_RESET);
//拉高使能
#define CE_HIGH        HAL_GPIO_WritePin(SPI1_EN_GPIO_Port, SPI1_EN_Pin,GPIO_PIN_SET);

//#define	MOSI                  // Master Out, Slave In pin (output)
//#define	MISO                  // Master In, Slave Out pin (input)
//#define	SCK                   // Serial Clock pin, (output)
//#define	IRQ                   // Interrupt signal, from nRF24L01 (input)

#define TX_ADR_WIDTH   5  				// 5字节宽度的发送/接收地址
#define TX_PLOAD_WIDTH 17  				// 数据通道有效数据宽度

#define CHANNEL      40                   //射频通道

//********************************************************************************************************************//
// SPI(nRF24L01+) commands
#define nRF24L01P_READ_REG     0x00         // 读寄存器命令
#define nRF24L01P_WRITE_REG    0x20         // 写寄存器命令
#define RD_RX_PLOAD            0x61         // 读接收FIFO有效数据
#define WR_TX_PLOAD            0xA0         // 写发送FIFO有效数据
#define FLUSH_TX               0xE1         // 清空发送FIFO
#define FLUSH_RX               0xE2         // 清空接收FIFO
#define REUSE_TX_PL            0xE3         // 重复使用上一包发送数据
#define NOP                    0xFF         // 空操作，常用于读状态寄存器

//********************************************************************************************************************//
// SPI(nRF24L01+) registers(addresses)
#define CONFIG          0x00        // 配置寄存器地址
#define EN_AA           0x01        // 使能自动应答寄存器地址
#define EN_RXADDR       0x02        // 使能接收通道地址寄存器地址
#define SETUP_AW        0x03        // 设置地址宽度寄存器地址
#define SETUP_RETR      0x04        // 设置自动重发寄存器地址
#define RF_CH           0x05        // RF通道寄存器地址
#define RF_SETUP        0x06        // RF设置寄存器地址
#define STATUS          0x07        // 状态寄存器地址
#define OBSERVE_TX      0x08        // 发送观察寄存器地址
#define RPD             0x09        // 接收功率检测器寄存器地址
#define RX_ADDR_P0      0x0A        // 接收通道0地址寄存器地址
#define RX_ADDR_P1      0x0B        // 接收通道1地址寄存器地址
#define RX_ADDR_P2      0x0C        // 接收通道2地址寄存器地址
#define RX_ADDR_P3      0x0D        // 接收通道3地址寄存器地址
#define RX_ADDR_P4      0x0E        // 接收通道4地址寄存器地址
#define RX_ADDR_P5      0x0F        // 接收通道5地址寄存器地址
#define TX_ADDR         0x10        // 发送地址寄存器地址
#define RX_PW_P0        0x11        // 接收通道0有效数据宽度寄存器地址
#define RX_PW_P1        0x12        // 接收通道1有效数据宽度寄存器地址
#define RX_PW_P2        0x13        // 接收通道2有效数据宽度寄存器地址
#define RX_PW_P3        0x14        // 接收通道3有效数据宽度寄存器地址
#define RX_PW_P4        0x15        // 接收通道4有效数据宽度寄存器地址
#define RX_PW_P5        0x16        // 接收通道5有效数据宽度寄存器地址
#define FIFO_STATUS     0x17        // FIFO状态寄存器地址

//********************************************************************************************************************//
// STATUS Register 
#define RX_DR						0x40    //接收完成
#define TX_DS						0x20    //发送完成
#define MAX_RT					    0x10    //达到最大重发次数

//********************************************************************************************************************//
//                                        FUNCTION's PROTOTYPES                                                       //
//********************************************************************************************************************//
//nRF24L01+ API Functions
void nRF24L01P_Init(void); //24L01+ Pin Init
uint8_t nRF24L01P_Write_Reg(uint8_t reg, uint8_t value); 
uint8_t nRF24L01P_Write_Buf(uint8_t reg, const uint8_t *pBuf, uint8_t size);
uint8_t nRF24L01P_Read_Reg(uint8_t reg);
uint8_t nRF24L01P_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes);

void nRF24L01P_RX_Mode(void);
void nRF24L01P_TX_Mode(void);
uint8_t nRF24L01P_RxPacket(uint8_t *rxbuf);
uint8_t nRF24L01P_TxPacket(uint8_t *txbuf);

//********************************************************************************************************************//
#endif

