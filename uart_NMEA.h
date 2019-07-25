#ifndef __uart_NMEA_H
#define __uart_NMEA_H

struct buffer {
  char string [500];
  int index;
};

void uart_thread1(void const *argument);

#endif // __uart_NMEA_H
