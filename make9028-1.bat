@echo off
armcc -D__CMSIS_RTOS -D__MICROLIB -D__EVAL -D__FPU_PRESENT --apcs=interwork --thumb --cpu Cortex-A9 -g -O0 --md -I C:\Users\joe\Desktop\CMSIS_RTOS_RTX/RTOS/RTX/SRC -I C:\Users\joe\Desktop\CMSIS_RTOS_RTX/RTOS/RTX/INC -I C:\Users\joe\Desktop\CMSIS_RTOS_RTX/Include -I . -I ../INC -I C:\altera\14.1\embedded\ip\altera\hps\altera_hps\hwlib\include    -c -o RTX_ex2.o RTX_ex2.c
