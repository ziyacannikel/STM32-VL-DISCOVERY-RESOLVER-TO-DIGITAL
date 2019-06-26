#ifndef MY_FUNCTIONS
#define MY_FUNCTIONS

char *convert(char name, double value);
void USART_Puts(USART_TypeDef* USARTx , volatile char *s);
void i2c_ini(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_address_direction(uint8_t address);
void i2c_transmit(uint8_t veri);
void i2c_write( uint8_t address, uint8_t data ,_Bool  secim);
void usart_init(void);
void init_adc1(void);
void timer_init(void);
char asc_convert(char value);
#endif
