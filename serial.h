
#ifndef __SERIAL_H
#define __SERIAL_H

typedef unsigned           char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

#define   __RO     volatile const
#define   __WO     volatile
#define   __RW     volatile

/* UART - Register Layout Typedef, see cyclone v HPS data sheet*/
typedef struct 
{
  	__RW uint32_t rbr_thr_dll;             
  	__RW uint32_t ier_dlh;    
  	__RO uint32_t iir;           
 	__WO uint32_t fcr;             
  	__RW uint32_t lcr;        
  	__RW uint32_t mcr;          
  	__RO uint32_t lsr;           
  	__RO uint32_t msr;           
  	__RW uint32_t scr;          
  	__RW uint32_t srbr;             
  	__RW uint32_t sthr;           
  	__RW uint32_t far;           
  	__RO uint32_t tfr;            
  	__WO uint32_t rfw;            
  	__RO uint32_t usr;            
  	__RO uint32_t tfl;          
  	__RO uint32_t rfl;          
  	__WO uint32_t srr;           
  	__RW uint32_t srts;         
  	__RW uint32_t sbcr;          
  	__RW uint32_t sdmam;     
  	__RW uint32_t sfe;      
  	__RW uint32_t srt;      
  	__RW uint32_t stet;      
  	__RW uint32_t htx;        
  	__WO uint32_t dmasa;        
  	__RO uint32_t cpr;        
  	__RO uint32_t ucv;        
  	__RO uint32_t ctr;
} UART_Type;

#define UART0_BASE      (0xFFC02000)  
#define UART0           ((UART_Type*)UART0_BASE)

#define UART0_CLK       100000000 //100MHz

extern void SER_Init(void);
extern void SER_Enable(void);
extern void SER_Disable(void);
extern void SER_GetChar (char* c);
extern void SER_PutChar(char c);
extern void SER_PutString(char s[]);
extern void SER_Set_baud_rate(uint32_t baud_rate);
extern void interrupt_SER(void);

#endif /* SERIAL_H_ */
