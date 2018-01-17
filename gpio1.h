// gp4020_io.h headers for gp4020_io.c
// Copyright (C) 2005  Andrew Greenberg
// Distributed under the GNU GENERAL PUBLIC LICENSE (GPL) Version 2 (June 1991).
// See the "COPYING" file distributed with this software for more information.

#ifndef __GPIO1_H
#define __GPIO1_H

#define GPIO1_BASE (volatile unsigned int*)0xFF220010

void gpio1_set(unsigned int pin);
void gpio1_clr(unsigned int pin);


#endif
