#include "stm32f1xx_hal.h"
#include "nRF24L01P.h"
#include "FreeRTOS.h"
#include "task.h"
uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x0A,0x01,0x07,0x0E,0x01};  // 定义一个静态发送地址

/**
 * @brief SPI读写一个字节，传入要写入的参数，以及读出内容
 * @param byte ：要写入的参数
 * @return 读取到的数据
 */
static uint8_t SPI_RW(uint8_t byte)
{
	uint8_t rx_data=0;
	HAL_SPI_TransmitReceive(&hspi1,&byte,&rx_data,1,1000);
	return rx_data;
}

/********************************************************
函数功能：写寄存器的值（单字节）                
入口参数：reg:寄存器映射地址（格式：nRF24L01P_WRITE_REG｜reg）
					value:寄存器的值
返回  值：状态寄存器的值
*********************************************************/
uint8_t nRF24L01P_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	CS_LOW;
  status = SPI_RW(reg);				
	SPI_RW(value);
	CS_HIGH;
	
	return status;
}


/********************************************************
函数功能：写寄存器的值（多字节）                  
入口参数：reg:寄存器映射地址（格式：nRF24L01P_WRITE_REG｜reg）
					pBuf:写数据首地址
					size:写数据字节数
返回  值：状态寄存器的值
*********************************************************/
uint8_t nRF24L01P_Write_Buf(uint8_t reg, const uint8_t *pBuf, uint8_t size)
{
	uint8_t status,byte_ctr;

	  CS_LOW;
	  status = SPI_RW(reg);
	  for(byte_ctr=0; byte_ctr<size; byte_ctr++) {
	  	SPI_RW(*pBuf++);
	  }

	  CS_HIGH;

  return(status);       
}							  					   


/********************************************************
函数功能：读取寄存器的值（单字节）                  
入口参数：reg:寄存器映射地址（格式：nRF24L01PREAD_REG｜reg）
返回  值：寄存器值
*********************************************************/
uint8_t nRF24L01P_Read_Reg(uint8_t reg)
{
 	uint8_t value;

	CS_LOW;
	SPI_RW(reg);
	value = SPI_RW(0);
	CS_HIGH;

	return(value);
}


/********************************************************
函数功能：读取寄存器的值（多字节）                  
入口参数：reg:寄存器映射地址（nRF24L01PREAD_REG｜reg）
					pBuf:接收缓冲区的首地址
					size:读取字节数
返回  值：状态寄存器的值
*********************************************************/
uint8_t nRF24L01P_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t size)
{
	uint8_t status,byte_ctr;

	  CS_LOW;
	  status = SPI_RW(reg);
	  for(byte_ctr=0;byte_ctr<size;byte_ctr++) {
	  	pBuf[byte_ctr] = SPI_RW(0);                   //读取数据，低字节在前
	  }
	  CS_HIGH;

  return(status);    
}


/********************************************************
函数功能：nRF24L01+接收模式初始化                      
入口参数：无
返回  值：无
*********************************************************/
void nRF24L01P_RX_Mode(void)
{
	CE_LOW;
	nRF24L01P_Write_Buf(nRF24L01P_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);		// 接收设备接收通道0使用和发送设备相同的发送地址
	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG + EN_AA, 0x01);               					// 使能接收通道0自动应答
	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG + EN_RXADDR, 0x01);           					// 使能接收通道0
	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG + RF_CH, CHANNEL);                 				// 选择射频通道0x40
	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);  					// 接收通道0选择和发送通道相同有效数据宽度
	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG + RF_SETUP, 0x06);            					// 数据传输率1Mbps，发射功率4dBm，低噪声放大器增益(nRF24L01+忽略该位）
	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG + CONFIG, 0x0f);              					// CRC使能，16位CRC校验，上电，接收模式
	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG + STATUS, 0xff);  								//清除所有的中断标志位
	CE_HIGH;                                            									// 拉高CE启动接收设备
}						


/********************************************************
函数功能：nRF24L01+发送模式初始化                      
入口参数：无
返回  值：无
*********************************************************/
void nRF24L01P_TX_Mode(void)
{
	CE_LOW;
	nRF24L01P_Write_Buf(nRF24L01P_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // 写入发送地址
	nRF24L01P_Write_Buf(nRF24L01P_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 为了应答接收设备，接收通道0地址和发送地址相同

	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG + EN_AA, 0x01);       					// 使能接收通道0自动应答
	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG + EN_RXADDR, 0x01);   					// 使能接收通道0
	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG + SETUP_RETR, 0x0a);  					// 自动重发延时等待250us+86us，自动重发10次
	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG + RF_CH, CHANNEL);         					// 选择射频通道0x40
	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG + RF_SETUP, 0x06);    					// 数据传输率1Mbps，发射功率4dBm，低噪声放大器增益(nRF24L01+忽略该位）
	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG + CONFIG, 0x0e);      					// CRC使能，16位CRC校验，上电
	CE_HIGH;
}


/********************************************************
函数功能：读取接收数据                       
入口参数：rxbuf:接收数据存放首地址
返回  值：0:接收到数据
          1:没有接收到数据
*********************************************************/
uint8_t nRF24L01P_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;
	state = nRF24L01P_Read_Reg(STATUS);  									//读取状态寄存器的值
	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG+STATUS,state);					//清除RX_DS中断标志

	if(state & RX_DR)								                        //接收到数据
	{
		nRF24L01P_Read_Buf(RD_RX_PLOAD,rxbuf,TX_PLOAD_WIDTH);				//读取数据
		nRF24L01P_Write_Reg(FLUSH_RX,0xff);									//清除RX FIFO寄存器
		return 0; 
	}	   
	return 1;																//没收到任何数据
}


/********************************************************
函数功能：发送一个数据包                      
入口参数：txbuf:要发送的数据
返回  值： 0:发送成功
          1:发送失败
*********************************************************/
uint8_t nRF24L01P_TxPacket(uint8_t *txbuf)
{
	uint8_t state;
	CE_LOW;																	//CE拉低，使能24L01配置
	nRF24L01P_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);					//写数据到TX FIFO,32个字节
 	CE_HIGH;																//CE置高，使能发送
	state=nRF24L01P_Read_Reg(STATUS);  										//读取状态寄存器的值
	while (((state&TX_DS)==0)&&((state&MAX_RT)==0))								//等待发送完成
	{
		state=nRF24L01P_Read_Reg(STATUS);
		vTaskDelay(1);														//读取状态寄存器的值
	}
	nRF24L01P_Write_Reg(nRF24L01P_WRITE_REG+STATUS,state); 					//清除TX_DS或MAX_RT中断标志
	if(state&MAX_RT)														//达到最大重发次数
	{
		nRF24L01P_Write_Reg(FLUSH_TX,0xff);									//清除TX FIFO寄存器
		return 1;															//发送失败
	}


	if(state&TX_DS) {
		return 0;															//发送完成
	}

	return 1;																//发送失败
}

uint8_t nRF24L01P_rx_Buf[5]={0};

/**
 * @brief nRF24L01P的初始化检测
 * @return uint8_t 0:检测成功，1：检测失败
 */
uint8_t nRF24L01P_Check(void) {

	nRF24L01P_Read_Buf(nRF24L01P_READ_REG + TX_ADDR,nRF24L01P_rx_Buf,TX_ADR_WIDTH);
	//1.测试SPI通信能供正常读写寄存器
	//1.1写入发送地址
	nRF24L01P_Write_Buf(nRF24L01P_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
	//2.在同样的地址读出相同的数据
	nRF24L01P_Read_Buf(nRF24L01P_READ_REG + TX_ADDR,nRF24L01P_rx_Buf,TX_ADR_WIDTH);
	for (uint8_t i=0;i<TX_ADR_WIDTH;i++) {
		if (TX_ADDRESS[i] != nRF24L01P_rx_Buf[i]) {
			return 1;
		}
	}
	return 0;
}

/********************************************************
函数功能：nRF24L01+功能测试函数
入口参数：无
返回  值：无
*********************************************************/

void nRF24L01P_Init(void)
{
	HAL_Delay(200); // 上电延时
	//校验检测
	while(nRF24L01P_Check()) {
		//每隔10ms检测一次
		HAL_Delay(10);
	}
	//默认nRF24L01P的状态为接收模式
	nRF24L01P_RX_Mode();
	printf("nRF24L01P Init Finished\n");
}