#ifndef __DOP_H
#define __DOP_H

typedef struct {
	double VDOP; //sqrt(ZDOP^2)
    double HDOP; //sqrt(XDOP^2+YDOP^2)
    double PDOP; //sqrt(XDOP^2+YDOP^2+VDOP^2)
    double GDOP; //sqrt(XDOP^2+YDOP^2+VDOP^2+TDOP^2)
    int	   Num_of_SV;
} DOP;

DOP CALCULATE_DOP( unsigned short NumSV );

#endif // __DOP_H
