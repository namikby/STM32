#include "spi.h"

void initSPI1(uint16_t prescaler)
{/// init SPI1 in master mode
	///wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	/// SPI1 Pin configuration
	///-----------------------------------------------------------------
	/// PE4 = CS
	/// PA5 = SCK
	/// PA6 = MISO
	/// PA7 = MOSI
	///wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 								//
	GPIOA->MODER |= 0x0000A800; 										// 
	GPIOA->AFR[0] |= 0x55500000;										//   
	GPIOA->OSPEEDR |= 0x0000A800;										// 
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	GPIOE->MODER |= 0x00000100; 
	GPIOE->OTYPER &= ~(0x00000008);
	GPIOE->OSPEEDR |= 0x00000080;										// 
	SPI1_CS_HIGH;
	
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; 								// 	
	SPI1->CR1 = (SPI_CR1_MSTR);											// enable 8 bit data & master mode			
	SPI1->CR1 |= (SPI_CR1_SSI)|(SPI_CR1_SSM);
	
	SPI1->CR1 |= prescaler;		
	SPI1->CR1 |= (SPI_CR1_SPE);											// 
}

uint8_t rxByteSPI1(void)
{///  receive data using SPI1
	uint8_t data;
	
	SPI1->DR = 0x00;						 							// 
	while(!(SPI1->SR & SPI_I2S_FLAG_TXE)); 								// 
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE)); 							// 
	while(SPI1->SR & SPI_I2S_FLAG_BSY); 								//	
	data = SPI1->DR; 													//
	
	return data;
}	

uint8_t txByteSPI1(uint8_t data)
{/// send data using SPI1
	uint8_t tmp;

	SPI1->DR = data; 													//
	while(!(SPI1->SR & SPI_I2S_FLAG_TXE));								// 
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE)); 							// 
	while(SPI1->SR & SPI_I2S_FLAG_BSY); 								// 
	tmp = SPI1->DR; 													// 
		
	return tmp;
}

