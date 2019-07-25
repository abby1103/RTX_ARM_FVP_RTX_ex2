/*
 * uart_ARRC.h
 *
 *  Created on: 2019/3/25
 *      Author: me249
 */

#ifndef UART_ARRC_H_
#define UART_ARRC_H_

struct buffer {
  char string [500];
  int index;
};

void uart_thread(void const *argument);

#endif /* UART_ARRC_H_ */

