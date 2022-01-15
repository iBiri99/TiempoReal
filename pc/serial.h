#ifndef SERIAL_H_
#define SERIAL_H_

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

void set_blocking (int fd, int should_block);
int set_interface_attribs (int fd, int speed, int parity);

#endif //SERIAL_H_
