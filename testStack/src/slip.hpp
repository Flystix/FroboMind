#ifndef SLIP_HPP_
#define SLIP_HPP_

#define BUFFER_SIZE			64
#define STA					0xC0
#define END					0xCF
#define ESC					0xDB
#define ESC_END				0xDC
#define ESC_STA				0xDD
#define ESC_ESC				0xDE

void init_fifo(void);
void print_fifo(void);
void send_pkg(void*, uint32_t);
uint32_t receive_pkg(void*, uint32_t);
void send_char(uint8_t);
uint8_t receive_char(void);

//int slip(const void* data_pkg, int len, uint8_t* slip_pkg);

#endif /* SLIP_HPP_ */
