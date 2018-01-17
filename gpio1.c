// gp4020_io.c hardware I/O for GP4020
// Copyright (C) 2005  Andrew Greenberg
// Distributed under the GNU GENERAL PUBLIC LICENSE (GPL) Version 2 (June 1991).
// See the "COPYING" file distributed with this software for more information.
//#include <cyg/kernel/kapi.h>
//#include "gp4020_io.h"
#include "gpio1.h"
//==========================================================================
// 32 bit access functions
//==========================================================================
void gpio1_set(unsigned int pin)
{
	*GPIO1_BASE |= (0x01 << pin);
}
void gpio1_clr(unsigned int pin)
{
	*GPIO1_BASE &= ~(0x01 << pin);
}

