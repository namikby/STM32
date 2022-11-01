#include "rfid.h"
#include "spi.h"
#include "delay.h"

uint8_t rfid_init(void){
	initSPI1(SPI_BaudRatePrescaler_32); //init SPI
	writeReg(MFRC522_REG_COMMAND,PCD_RESETPHASE); //reset RC522
	writeReg(MFRC522_REG_T_MODE, 0x8D); //test
	writeReg(MFRC522_REG_T_PRESCALER, 0x3E);
	writeReg(MFRC522_REG_T_RELOAD_L, 30);           
	writeReg(MFRC522_REG_T_RELOAD_H, 0);
	writeReg(MFRC522_REG_RF_CFG, 0x70); //config of antenna
	writeReg(MFRC522_REG_TX_AUTO, 0x40);
	writeReg(MFRC522_REG_MODE, 0x3D);

	uint8_t temp;  //config of control register
	temp = readReg(MFRC522_REG_TX_CONTROL);
	if (!(temp & 0x03)) {
		writeReg(MFRC522_REG_TX_CONTROL,readReg(MFRC522_REG_TX_CONTROL) | 0x03);
	};
	
	temp=readReg(MFRC522_REG_VERSION);//test using version register
	return temp;
	}

uint8_t rfid_check(uint8_t* a){ //rfid card check
	uint8_t status;
	status = request(PICC_REQIDL, a);	
	if (status == 1) {
		status = coll(a);
	};
	
	//hibernation
	uint16_t b;
	uint8_t buff[4]; 
	buff[0] = PICC_HALT;
	buff[1] = 0;
	calculateCRC(buff, 2, &buff[2]);
	toCard(PCD_TRANSCEIVE, buff, 4, buff, &b); 
	
	return status;
}

void writeReg(uint8_t adr, uint8_t val){ //write to register
	SPI1_CS_LOW;
	txByteSPI1((adr << 1) & 0x7E);
	txByteSPI1(val);
	SPI1_CS_HIGH;
}

uint8_t readReg(uint8_t adr){ //read register
	uint8_t val;
	SPI1_CS_LOW;
	txByteSPI1(((adr << 1) & 0x7E) | 0x80);
	val=txByteSPI1(0x00);
	SPI1_CS_HIGH;
	return val;
}

void calculateCRC(uint8_t*  a, uint8_t b, uint8_t* out){ //CRC - random value
	uint8_t i, n;

	clearBitMask(MFRC522_REG_DIV_IRQ, 0x04);
	setBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);

	//write to FIFO
	for (i = 0; i < b; i++) {   
		writeReg(MFRC522_REG_FIFO_DATA, *(a+i));   
	}
	writeReg(MFRC522_REG_COMMAND, PCD_CALCCRC);

	//wait CRC
	i = 0xFF;
	do{
		n = readReg(MFRC522_REG_DIV_IRQ);
		i--;
	}while ((i!=0) && !(n&0x04));

	//read result of CRC
	out[0] = readReg(MFRC522_REG_CRC_RESULT_L);
	out[1] = readReg(MFRC522_REG_CRC_RESULT_M);
}

uint8_t request(uint8_t rqMode, uint8_t* tagType){ //data request sending
	uint8_t status;
	uint16_t backBits;

	writeReg(MFRC522_REG_BIT_FRAMING, 0x07);

	tagType[0] = rqMode;
	status = toCard(PCD_TRANSCEIVE, tagType, 1, tagType, &backBits);

	if((status != 1) || (backBits != 0x10)){
		status = 0;
	};

	return status;
}

uint8_t toCard(uint8_t com, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen){ //communication with RFID card
	uint8_t status = 0;
	uint8_t irqEn = 0x00;
	uint8_t waitIRq = 0x00;
	uint8_t lastBits;
	uint8_t n;
	uint16_t i;

	switch (com) {
		case PCD_AUTHENT: {
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case PCD_TRANSCEIVE: {
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
			break;
	}

	writeReg(MFRC522_REG_COMM_IE_N, irqEn | 0x80);
	clearBitMask(MFRC522_REG_COMM_IRQ, 0x80);
	setBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);

	writeReg(MFRC522_REG_COMMAND, PCD_IDLE);

	//write to FIFO
	for (i = 0; i < sendLen; i++) {   
		writeReg(MFRC522_REG_FIFO_DATA, sendData[i]);    
	}

	writeReg(MFRC522_REG_COMMAND, com);
	if (com == PCD_TRANSCEIVE) {    
		setBitMask(MFRC522_REG_BIT_FRAMING, 0x80);
	}   

	i = 2000;
	do{
		n = readReg(MFRC522_REG_COMM_IRQ);
		i--;
	}while ((i!=0) && !(n&0x01) && !(n&waitIRq));

	clearBitMask(MFRC522_REG_BIT_FRAMING, 0x80);

	if(i != 0){
		if(!(readReg(MFRC522_REG_ERROR) & 0x1B)){
			status = 1;
			if(n & irqEn & 0x01){
				status = 0;	
			}

			if(com == PCD_TRANSCEIVE){
				n = readReg(MFRC522_REG_FIFO_LEVEL);
				lastBits = readReg(MFRC522_REG_CONTROL) & 0x07;
				if(lastBits){   
					*backLen = (n - 1) * 8 + lastBits;   
				}else{   
					*backLen = n * 8;   
				}

				if(n == 0){   
					n = 1;    
				}
				if(n > MFRC522_MAX_LEN){
					n = MFRC522_MAX_LEN;   
				}

				for(i = 0; i < n; i++){
					backData[i] = readReg(MFRC522_REG_FIFO_DATA);
				}
			}
		} else {
			status = 0;  
		}
	}

	return status;
}

void setBitMask(uint8_t reg, uint8_t mask){ //bit mask setting
	writeReg(reg,readReg(reg) | mask);
}

void clearBitMask(uint8_t reg, uint8_t mask){ //bit mask delete
	writeReg(reg,readReg(reg) & (~mask));
} 

uint8_t coll(uint8_t* srNum){ //collision check
	uint8_t status;
	uint8_t i;
	uint8_t srNumCheck = 0;
	uint16_t unLen;

	writeReg(MFRC522_REG_BIT_FRAMING, 0x00);

	srNum[0] = PICC_ANTICOLL;
	srNum[1] = 0x20;
	status = toCard(PCD_TRANSCEIVE, srNum, 2, srNum, &unLen);
	
	//card serial number check
	if (status == 1) { 
		for (i = 0; i < 4; i++) {   
			srNumCheck ^= srNum[i];
		}
		if (srNumCheck != srNum[i]) {   
			status = 0;    
		}
	}
	return status;
}

void open(void){ //access allowed - green LED
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER &=~(0xFF000000);
	GPIOD->MODER |= 0x55000000;
    GPIOD->OTYPER |= 0x00000000;
    GPIOD->OSPEEDR |= 0xFF000000;
	GPIOD->ODR &= ~(0xf000);
	GPIOD->ODR |= 0x1000;
	delay_ms(5000);
	GPIOD->ODR &=~(0x1000);
	}

void no_open(void){ //access not allowed - red LED
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER &=~(0xFF000000);
	GPIOD->MODER |= 0x55000000;
    GPIOD->OTYPER |= 0x00000000;
    GPIOD->OSPEEDR |= 0xFF000000;
	GPIOD->ODR &= ~(0xf000);
	GPIOD->ODR |= 0x4000;
	delay_ms(5000);
	GPIOD->ODR &=~(0x4000);
	}

uint8_t compare(uint8_t* a,uint8_t* b){ //two cards compare
	uint8_t i;
	for (i = 0; i <= 4; i++){
		if (a[i] != b[i]){
			return 0;
			};
		};
	return 1;
}
