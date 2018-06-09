#ifndef I2C_H
#define I2C_H

void ClearDisplay(uint8_t addr);
uint32_t I2C1_PCF2119_read_ddram_char(uint8_t addr, uint8_t row, uint8_t pos);
uint8_t I2C1_PCF2119_BF_read(uint8_t addr);
uint8_t I2C1_PCF2119_AC_read(uint8_t addr);
void I2C1_wr_string(uint8_t addr, uint8_t row, uint8_t pos, char* data);
void I2C1_PCF2119_SW_Init(uint8_t addr);
void I2C1_PCF2119_Cursor_switch(uint8_t addr, uint8_t stat);
void I2C1_PCF2119_Cursor_shift(uint8_t addr, uint8_t stat);
void I2C1_PCF2119_Cursor_set_pos(uint8_t addr, uint8_t row, uint8_t pos);
void I2C1_PCF2119_contrast(uint8_t addr, uint8_t val);

#endif
