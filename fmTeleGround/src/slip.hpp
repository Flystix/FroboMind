#ifndef SLIP_HPP_
#define SLIP_HPP_

#define BUFFER_SIZE			64
#define STA					0xC1
#define END					0xCF
#define ESC					0xDB
#define ESC_STA				0xDD
#define ESC_END				0xDC
#define ESC_ESC				0xDE

void slip_pkg(void* data_in, std::string* str_out, uint32_t len);
void slip_pkg(void* data_in, int fd, uint32_t len);
uint32_t unslip_pkg(std::string* str_in, void* data_out, uint32_t len);
uint32_t unslip_pkg(int fd_in, void* data_out, uint32_t len);

//int slip(const void* data_pkg, int len, uint8_t* slip_pkg);

#endif /* SLIP_HPP_ */

