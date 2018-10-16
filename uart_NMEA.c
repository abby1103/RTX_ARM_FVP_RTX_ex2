#include "cmsis_os.h"

#include <stdio.h>
#include "uart_NMEA.h"
#include "altera_avalon_uart_lwhal.h"
#include "uart_putstring.h"
#include "position.h"
#include "constants.h"
#include "position.h"
#include "measure.h"
#include "time.h"
#include "tracking.h"
#include <math.h>
#include "constants.h"

struct buffer gga_buf;
struct buffer gsa_buf;
struct buffer rmc_buf;
struct buffer vtg_buf;


void uart_thread(void const *argument)
{
    while(1){

        osSignalWait(0x0006, osWaitForever);

        time_t          std_time;
        std_time = get_standard_time();

        //*******      

        //*******           LLH                 ***********//
        double          lat_DD, lon_DD;
        double          lat_DMM, lon_DMM;
        double          height;

        lat_DD = receiver_llh.lat * RADTODEG;
        lon_DD = receiver_llh.lon * RADTODEG;
        height = receiver_llh.hgt;

        //*******           DD-->DMM            ***********//
        double a, b, c, d;

        a = (int) lat_DD;    //take Integer
        b = (int) lon_DD;
        c = lat_DD - a;      //Pure decimal
        d = lon_DD - b;


        lat_DMM = a * 100 + c * 60;
        lon_DMM = b * 100 + d * 60;
        //*******           DD-->DMM END            ***********//

        //*******           N or S                  ***********//
        char SIGN_N_S;
        if (lat_DMM > 0) {
            SIGN_N_S = 'N';
        }
        else if (lat_DMM < 0) {
            SIGN_N_S = 'S';
            lat_DMM = lat_DMM * (-1);
        }
        else {
            SIGN_N_S = ' ';
        }
        //*******           N or S END              ***********//

        //*******           E or W                  ***********//
        char SIGN_E_W;
        if (lon_DMM > 0) {
            SIGN_E_W = 'E';
        }
        else if (lon_DMM < 0) {
            SIGN_E_W = 'W';
            lon_DMM = lon_DMM * (-1);
        }
        else {
            SIGN_E_W = ' ';
        }
        //*******           E or W END              ***********//

        //*******           velocity                ***********//
        double East_v = -sin(receiver_llh.lon) * receiver_pvt_velocity.vx + cos(receiver_llh.lon) * receiver_pvt_velocity.vy;
        double North_v = ( -sin(receiver_llh.lat) * cos(receiver_llh.lon) * receiver_pvt_velocity.vx ) - ( sin(receiver_llh.lat) * sin(receiver_llh.lon) * receiver_pvt_velocity.vy ) + ( cos(receiver_llh.lat) * receiver_pvt_velocity.vz );
        double Up_V = ( cos(receiver_llh.lat) * cos(receiver_llh.lon) * receiver_pvt_velocity.vx) + ( cos(receiver_llh.lat) * sin(receiver_llh.lon) * receiver_pvt_velocity.vy ) + ( sin(receiver_llh.lat) * receiver_pvt_velocity.vz );

        double horizontal_v = sqrt( East_v * East_v + North_v * North_v )*3.6;  // unit:km/hr
        double horizontal_v_knot = horizontal_v * 1.852;                        // unit = knot
        double vertical_v = Up_V *3.6;                                          // km/kr
        double vertical_v_knot = vertical_v * 1.852;                            // knot
        double v = sqrt(horizontal_v * horizontal_v + vertical_v * vertical_v); // km/kr

        double angle_hv = acos( (North_v * 3.6) / horizontal_v) * RADTODEG ;    // horizontal Track angle to turn north in degrees
        double angle_vv  = acos( horizontal_v / v) * RADTODEG ;                  // vertical Track angle to turn north in degrees


        //********  position status ACTIVE or VOID  ************//
        char status;
        if(receiver_pvt.valid)
            status = 'A';
        else
            status = 'V';
        


        //********          nmea_buf                ************//

        //GPGGA  
        print_buffer( &gga_buf,"$GPGGA,%02d%02d%06.3f,%09.4f,%c,%010.4f,%c,%d,%.1f,%.1f,M,%.1f,M,,*"
                    ,std_time.hours,std_time.minutes,std_time.seconds
                    ,lat_DMM,SIGN_N_S,lon_DMM,SIGN_E_W
                    ,receiver_DOP.Num_of_SV,receiver_DOP.HDOP
                    ,height,0.0); /*Height of geoid not ok*/

        //checksum of gga 
        unsigned int sum = 0;
        int i1 = 1;
        
        while( (unsigned int)gga_buf.string[i1] != '*'){
            sum ^= (unsigned int)gga_buf.string[i1];
            i1 ++;
        }

        print_buffer( &gga_buf,"%x\r\n",sum);




        //GPGSA
        print_buffer( &gsa_buf,"$GPGSA,%c,%d,"
                    ,'A',3);

        int i2;
        for(i2 = 0; i2 < receiver_DOP.Num_of_SV; i2++ ){
        	print_buffer( &gsa_buf,"%02hu,",sat_position[i2].prn);
        }

        print_buffer( &gsa_buf,"%.1f,%.1f,%.1f*"
        			,receiver_DOP.PDOP,receiver_DOP.HDOP,receiver_DOP.VDOP);

        //checksum of gsa 
        sum = 0;
        i1 = 1;  
        while( (unsigned int)gsa_buf.string[i1] != '*'){
            sum ^= (unsigned int)gsa_buf.string[i1];
            i1 ++;
        }

        print_buffer( &gsa_buf,"%x\r\n",sum);

        //GPRMC
        print_buffer( &rmc_buf,"$GPRMC,%02d%02d%06.3f,%c,%09.4f,%c,%010.4f,%c,%05.1f,%05.1f,%02d%02d%02d,%05.1f,%c*"
                    ,std_time.hours,std_time.minutes,std_time.seconds
                    ,status
                    ,lat_DMM,SIGN_N_S,lon_DMM,SIGN_E_W
                    ,horizontal_v_knot,angle_hv
                    ,std_time.days,std_time.months,std_time.years
                    ,0.0,' '); //Magnetic Variation not ok
        //checksum of rmc 
        sum = 0;
        i1 = 1;  
        while( (unsigned int)rmc_buf.string[i1] != '*'){
            sum ^= (unsigned int)rmc_buf.string[i1];
            i1 ++;
        }

        print_buffer( &rmc_buf,"%x\r\n",sum);

        //GPVTG
        print_buffer( &vtg_buf,"$GPVTG,%05.1f,%05.1f,%05.1f,N,%06.1f,K*"
                    ,angle_hv,0.0 //Magnetic Variation not oK
                    ,horizontal_v_knot,horizontal_v);
        //checksum of vtg 
        sum = 0;
        i1 = 1;  
        while( (unsigned int)vtg_buf.string[i1] != '*'){
            sum ^= (unsigned int)vtg_buf.string[i1];
            i1 ++;
        }

        print_buffer( &vtg_buf,"%x\r\n",sum);

        
        //**********            uart                ***********//

        /*print GPGGA*/
        altera_avalon_uart_lwhal_putstring(&gga_buf,(void*)(0xFF200400));	/*print*/
        /*print GPGSA*/
        altera_avalon_uart_lwhal_putstring(&gsa_buf,(void*)(0xFF200400));	/*print*/
        /*print GPRMC*/
        altera_avalon_uart_lwhal_putstring(&rmc_buf,(void*)(0xFF200400));	/*print*/
        /*print GPVTG*/
        altera_avalon_uart_lwhal_putstring(&vtg_buf,(void*)(0xFF200400));	/*print*/

    }
    

}
