#include "stm32f10x.h"
#include "i2c_pcf2119.h"

void I2C1_Start(void)
{
   I2C1->CR1 |= I2C_CR1_START;
   while( !(I2C1->SR1 & I2C_SR1_SB) );
}


void I2C1_Stop(void)
{
   while( !(I2C1->SR1 & I2C_SR1_BTF) );
   I2C1->CR1 |= I2C_CR1_STOP;
}


void I2C1_SendAddr(uint8_t address)
{
   I2C1->DR = address;
   while( !(I2C1->SR1 & I2C_SR1_ADDR) );
   (void)  I2C1->SR1; //need for clearing some bits in SR1 register
   (void)  I2C1->SR2; //need for clearing some bits in SR1 register
}


void I2C1_SendByte(uint8_t byte)
{
   while( !(I2C1->SR1 & I2C_SR1_TXE) );
   I2C1->DR = byte;
}


void I2C1_wr_string(uint8_t addr, uint8_t row, uint8_t pos, char* data){
/*
Display Position    1   2   3   4   5   6   7   8   9   10  11  12
DD RAM Address      02  03  04  05  06  07  08  09  0A  0B  0C  0D
DD RAM Address      42  43  44  45  46  47  48  49  4A  4B  4C  4D
*/
  uint8_t DDRAM_address = 0;
  if (row == 1) { DDRAM_address = 1 + pos;}
  if (row == 2) { DDRAM_address = 0x41 + pos;}
  DDRAM_address |= (1 << 7);

	I2C1_Start();
	I2C1_SendAddr(addr);
	I2C1_SendByte(0x00);
	I2C1_SendByte(DDRAM_address); //Set_DDRAM address
	I2C1_Stop();

	I2C1_Start();
	I2C1_SendAddr(addr);
	I2C1_SendByte(0x40);  //CO=0, RS=1

  uint8_t i=0;
  while(data[i]){
    I2C1_SendByte((data[i++] | (1 << 7)));
  }
	I2C1_Stop();
}


uint32_t I2C1_PCF2119_read_ddram_char(uint8_t addr, uint8_t row, uint8_t pos){

  uint8_t DDRAM_address = 0;
  if (row == 1) { DDRAM_address = 1 + pos;}
  if (row == 2) { DDRAM_address = 0x41 + pos;}
  DDRAM_address |= (1 << 7);

	I2C1_Start();
	I2C1_SendAddr(addr);
	I2C1_SendByte(0x00);
	I2C1_SendByte(DDRAM_address); //Set_DDRAM address
	I2C1_Stop();

	I2C1_Start();
	I2C1_SendAddr(addr);
	I2C1_SendByte(0x40);  //CO=0, RS=1

	I2C1_Start();
	I2C1_SendAddr(addr | 0x01); //Адрес addr + бит R/W=1
	I2C1->CR1 &= ~I2C_CR1_ACK;							//NACK
	while(!(I2C1->SR1 & I2C_SR1_RXNE));
	I2C1_Stop();
	return (I2C1->DR &= 0x7F);
}

/**
* Чтение бита BF (Busy flag). ret=1 -> busy
*/
uint8_t I2C1_PCF2119_BF_read(uint8_t addr){
	I2C1_Start();
  I2C1_SendAddr(addr);
  I2C1_SendByte(0x00);  //CO=0, RS=0
	I2C1_Stop();

	I2C1_Start();
	I2C1_SendAddr(addr | 0x01); //Адрес addr + бит R/W=1
	I2C1->CR1 &= ~I2C_CR1_ACK;							//NACK
	while(!(I2C1->SR1 & I2C_SR1_RXNE));
	I2C1_Stop();
	uint8_t ret = (I2C1->DR >> 7);
	return ret;
}


uint8_t I2C1_PCF2119_AC_read(uint8_t addr){
	I2C1_Start();
  I2C1_SendAddr(addr);
  I2C1_SendByte(0x00);  //CO=0, RS=0
	I2C1_Stop();

	I2C1_Start();
	I2C1_SendAddr(addr | 0x01); //Адрес addr + бит R/W=1
	I2C1->CR1 &= ~I2C_CR1_ACK;							//NACK
	while(!(I2C1->SR1 & I2C_SR1_RXNE));
	I2C1_Stop();
	uint8_t ret = (I2C1->DR & 0x7F);
	return ret;
}


void I2C1_PCF2119_SW_Init(uint8_t addr){
  // PCF2119, setting of proper display modes
  I2C1_Start();
  I2C1_SendAddr(addr);
  I2C1_SendByte(0x00);  // MSB (Continuation bit Co) = 0, more than one byte may follow. Bit6, RS=0, next byte is command byte
 	I2C1_SendByte(0x25);  // 2 lines x 16, 1/18 duty, extended instruction set. Next byte will be another command.
 	I2C1_SendByte(0x06);  // Set display configuration to right to left, column 80 to 1. Row data displ. top to bottom,1 to 16.
 	I2C1_SendByte(0x08);  // Set to character mode, full display, icon blink disabled
  I2C1_SendByte(0x40);  // Set voltage multiplier to 2
  I2C1_SendByte(0xA5);  // Set Vlcd and store in register VA. VLCD nom = Vx * 0.08 + 1.82. Vx = (3.3-1.82)/0.08 = 19[dec]= 13[hex].
                        // Но когда я подставил 0x93 - символы вообще не горели. Поэтому экспериментально поставил 0xAA (AA=3.3/0.8)
                        // также нашёл в даташите на экран инфу: Driving voltage = 4.85V. 4,85-1,82=3,03. 3,03/0,08=37
                        // 37[dec]=0x25.  0x25+0x80=0xA5
  I2C1_SendByte(0x24);  // Change from extended instruction set to basic instruction set
  I2C1_SendByte(0x0C);  // Display control: set display on, cursor off, no blink
  I2C1_SendByte(0x06);  // Entry mode set, increase DDRAM after access, no shift
  I2C1_SendByte(0x02);  // Return home, set DDRAM address 0 in address counter
  I2C1_SendByte(0x01);  // Clear entire display, set DDRAM address to 0 in address counter

  ClearDisplay(addr);
}


void I2C1_PCF2119_contrast(uint8_t addr, uint8_t val){
  I2C1_Start();
  I2C1_SendAddr(addr);
  I2C1_SendByte(0x00);
  I2C1_SendByte(0x25);
  I2C1_SendByte(0x80 | val);

  I2C1_SendByte(0x24);  // Change from extended instruction set to basic instruction set
  I2C1_SendByte(0x0C);  // Display control: set display on, cursor off, no blink
  I2C1_SendByte(0x06);  // Entry mode set, increase DDRAM after access, no shift
  I2C1_SendByte(0x02);  // Return home, set DDRAM address 0 in address counter
  I2C1_Stop();
}

	/// Очистка экрана.
	// т.к. на моём дисплее прошит набор символов "R", то стандартный пробел 20h там заменён на другой символ.
	// Поэтому стандартная функция очистки экрана (I2C1_SendByte(0x01)) тут не работает.
void ClearDisplay(uint8_t addr) {
  I2C1_Start();
  I2C1_SendAddr(addr);
  I2C1_SendByte(0x00);
  I2C1_SendByte(0x24);  // Go to basic instruction set
  I2C1_SendByte(0x08);  // Display Off
  I2C1_SendByte(0x80);  // Set_DDRAM = 0
  I2C1_Stop();

  I2C1_Start();
  I2C1_SendAddr(addr);
  I2C1_SendByte(0x40);
  for (uint8_t q = 0; q <= 127; q++){
    I2C1_SendByte(160); // Пустой символ
  }
  I2C1_Stop();

  I2C1_Start();
  I2C1_SendAddr(addr);
  I2C1_SendByte(0x00);
  I2C1_SendByte(0x24);  // Go to basic instruction set
  I2C1_SendByte(0x40);  // Set_CGRAM = 0
  I2C1_Stop();

  I2C1_Start();
  I2C1_SendAddr(addr);
  I2C1_SendByte(0x40);
  for (uint8_t q = 0; q <= 127; q++){
    I2C1_SendByte(0); // Обнуляем все символы
  }
  I2C1_Stop();

  I2C1_Start();
  I2C1_SendAddr(addr);
  I2C1_SendByte(0x00);
  I2C1_SendByte(0x24);  // Go to basic instruction set
  I2C1_SendByte(0x0C);  // Display On
  I2C1_Stop();
}

