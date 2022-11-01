#include <stdlib.h>
#include "stm32f4xx.h"
#include "delay.h"
#include "rfid.h"
#include "usart.h"

int main(void){
	initUSART2(USART2_BAUDRATE_115200);
	if(rfid_init()==146)
	sprintUSART2("\n Init succesfull! ");
	else
	sprintUSART2("\n Init error! ");
	uint8_t key[5]; // RFID input
	uint8_t lock[5]={76,164,117,99,254}; // code for comparison
    while(1){
		if(rfid_check(key)){
			if(compare(key,lock)==1){
				sprintUSART2("\n OPEN! ");
				printUSART2(" %d %d %d %d %d ",key[0],key[1],key[2],key[3],key[4]);
				open();
			} else{
				sprintUSART2("\n LOCKED! ");
				printUSART2(" %d %d %d %d %d ",key[0],key[1],key[2],key[3],key[4]);
				no_open();
			};
		};
		delay_ms(500);
	};
	return 0;
};
