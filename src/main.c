#include "stm32f10x.h"
#include "main.h"
#include "delay.h"
#include "i2c_pcf2119.h"

#define LCD_ADDR  118 // Адрес PCF2119 = 118[dec] = 0x76

void I2C1_Initialize(void);
/*void I2C1_PCF2119_HW_Init(uint8_t addr);*/

uint8_t busyflag;

int main(void){
	SystemInit();

	SysTick_Config(SystemCoreClock/1000);

	I2C1_Initialize();

	I2C1_PCF2119_SW_Init(LCD_ADDR);
//  I2C1_wr_string(LCD_ADDR, 1, 1, "~!@#$%^&*()_");
//  I2C1_wr_string(LCD_ADDR, 2, 1, "+,-./:;<=>? ");
//  I2C1_wr_string(LCD_ADDR, 1, 3, "123456789012");
///// Установка контрастности. 255 = максимум
//  I2C1_PCF2119_contrast(LCD_ADDR, 30);
//  I2C1_wr_string(LCD_ADDR, 2, 3, "ABCDEFabcdef");
//  I2C1_PCF2119_contrast(LCD_ADDR, 42);
  I2C1_wr_string(LCD_ADDR, 1, 1, "123456789012");
  I2C1_wr_string(LCD_ADDR, 2, 1, "ABCDEFabcdef");
  char word[12];
  for (uint8_t q = 0; q < 12; q++){
    word[q] = I2C1_PCF2119_read_ddram_char(118, 2, q+1);
    uint8_t ac = I2C1_PCF2119_AC_read(118);
    uint8_t bf = I2C1_PCF2119_BF_read(118);
  }

  char letter = I2C1_PCF2119_read_ddram_char(118, 2, 1);
//  I2C1_wr_string(LCD_ADDR, 1, 6, "4 21");
//  I2C1_wr_string(LCD_ADDR, 2, 1, "        ");
}


void I2C1_Initialize(void){
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN |
										RCC_APB2ENR_AFIOEN;
	RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN;

  // I2C GPIO CONFIG
   GPIOB->CRL |=  GPIO_CRL_CNF6 | GPIO_CRL_CNF7 | GPIO_CRL_MODE5 | GPIO_CRL_MODE6 | GPIO_CRL_MODE7; //GPIO_CRL_MODE3 |
   GPIOB->CRL &= ~(GPIO_CRL_CNF5);  //GPIO_CRL_CNF3 |
   GPIOA->CRH |=  GPIO_CRH_MODE11;
   GPIOA->CRH &= ~(GPIO_CRH_CNF11);
   // PB5 - MODE5[1:0]=11: Output mode, max speed 50 MHz. CNF5[1:0]=00: General purpose output push-pull
   GPIOA->BSRR |=  GPIO_BSRR_BR11;  // Вкл. питание LCD

   // Software reset I2C
   I2C1->CR1 |= I2C_CR1_SWRST;
   I2C1->CR1 &= ~I2C_CR1_SWRST;

   I2C1->CR1 &= ~I2C_CR1_PE;
   I2C1->CR2 = I2C_CR2_FREQ_5 | I2C_CR2_FREQ_2;			//I2C1 Clock 36Mhz
//   I2C1->CR2 = I2C_CR2_FREQ_1 | I2C_CR2_FREQ_0;			//I2C1 Clock 2Mhz
	 I2C1->TRISE = 37; //37;
   I2C1->CCR = 180;																	//I2C1 Clk = 36Mhz/180/2 = 100kHz
//   I2C1->CCR = 1800;																	//I2C1 Clk = 72Mhz/1800/2 = 10kHz

   // Включение I2C
   I2C1->CR1 |= I2C_CR1_PE | I2C_CR1_ACK;
   I2C1->OAR1 = (1 << 14);

}

/*
void I2C1_PCF2119_HW_Init(uint8_t addr){

  /// Сбрасываем LCD-дисплей внешним ресетом (Power-On Reset -> POR)
  Delay(100);
  GPIOA->BSRR |=  GPIO_BSRR_BS11;  // Откл. питания LCD
  Delay(10);
  GPIOB->BSRR |=  GPIO_BSRR_BS5;  // Вкл. POR
  Delay(100);
  GPIOB->BSRR |=  GPIO_BSRR_BR5;  // Откл. POR
  Delay(400);
  GPIOA->BSRR |=  GPIO_BSRR_BR11; // Вкл. питания LCD
  Delay(100);


  I2C1_Start();
  I2C1_SendAddr(addr);
  I2C1_SendByte(0x00);
  I2C1_SendByte(0x34);
  Delay(10);

  I2C1_Start();
  I2C1_SendAddr(addr);
  I2C1_SendByte(0x00);
  I2C1_SendByte(0x34);
  Delay(10);

  I2C1_Start();
  I2C1_SendAddr(addr);
  I2C1_SendByte(0x00);
  I2C1_SendByte(0x34);
  Delay(10);

  I2C1_Start();
  I2C1_SendAddr(addr);
  I2C1_SendByte(0x00);
  I2C1_SendByte(0x34);
  I2C1_SendByte(0x08);
  I2C1_SendByte(0x01);
  I2C1_SendByte(0x04);
  I2C1_Stop();
}

*/
