#include <stdio.h>
#include <math.h>
#include <stdlib.h> 
#include "position.h"
#include "DOP.h"
//#include "measure.h" 



//DOP CALCULATE_DOP( unsigned short NumSV , xyz_t *ecef_temp){
DOP CALCULATE_DOP( unsigned short NumSV ){

    //user East-North-Up (ENU) coordinates
    double EASTi = 0;
    double NORTHi = 0;
    double UPi = receiver_llh.hgt;
    //Satellite : Convert ECEF coordinates to East-North-Up (ENU) coordinates 
    double EASTs[12];
    double NORTHs[12];
    double UPs[12];

    double sealevel = 0; //Altitude of sealevel is zero meter
    double a = 6378137;  //WGS84 ellopsoid constants 
    double es = 0.081819190842622; //WGS84 ellopsoid constants
    double N = a /sqrt( 1 -  es*es  *   sin(receiver_llh.lat)*sin(receiver_llh.lat)  ); //Prime vertical radius of curvature
 
    double xLocalRef = (N + sealevel) * cos(receiver_llh.lat) * cos(receiver_llh.lon); //x ECEF coordinates for Local Reference Pt for ENU frame (meters)
    double yLocalRef = (N + sealevel) * cos(receiver_llh.lat) * sin(receiver_llh.lon); //y ECEF coordinates for Local Reference Pt for ENU frame (meters)
    double zLocalRef = ( ( (1 - es*es ) * N) + sealevel ) * sin(receiver_llh.lat); //z ECEF coordinates for Local Reference Pt for ENU frame (meters)

    
    int num;
    for (num = 0; num < NumSV; num++ ){

        double East = -sin(receiver_llh.lon) * (sat_position[num].x - xLocalRef) + cos(receiver_llh.lon) * (sat_position[num].y - yLocalRef);
        double North = ( -sin(receiver_llh.lat) * cos(receiver_llh.lon) * (sat_position[num].x - xLocalRef) ) - ( sin(receiver_llh.lat) * sin(receiver_llh.lon) * (sat_position[num].y - yLocalRef) ) + ( cos(receiver_llh.lat) * (sat_position[num].z - zLocalRef) );
        double Up = ( cos(receiver_llh.lat) * cos(receiver_llh.lon) * (sat_position[num].x - xLocalRef) ) + ( cos(receiver_llh.lat) * sin(receiver_llh.lon) * (sat_position[num].y - yLocalRef) ) + ( sin(receiver_llh.lat) * (sat_position[num].z - zLocalRef) );
        EASTs[num] = East;
        NORTHs[num] = North;
        UPs[num] = Up;
    
    }


    /*//Determine Line of Sight between Target and Satellite 
    double mag_user = sqrt(pow(ecef_temp->x , 2) + pow(ecef_temp->y , 2) + pow(ecef_temp->z , 2)); //Distance of user from Earth center (meter)
    double XtoSV[31];
    double YtoSV[31];
    double ZtoSV[31];
    double mag_usertoSV[31]; //Distance from user to Satellite
    double AngleFromUser[31];
    int Los;

    for(int i0;i0<=NumSV;i0++){
        XtoSV[i0] = sat_position[i0].x- ecef_temp->x;
        YtoSV[i0] = sat_position[i0].y- ecef_temp->y;
        ZtoSV[i0] = sat_position[i0].z- ecef_temp->z;
        mag_usertoSV[i0] = sqrt( pow( XtoSV[i0] , 2) + pow( YtoSV[i0] , 2) + pow( ZtoSV[i0] , 2) ); //Distance from user to Satellite
        AngleFromUser[i0] = acos(((XtoSV[i0]*ecef_temp->x) + (YtoSV[i0]*ecef_temp->y) +(ZtoSV[i0]*ecef_temp->z))/(mag_usertoSV[i0]*mag_user));
        

        if (AngleFromUser[i0] < (pi/2-(10*pi/180)) && sat_position[i0].valid == 0) //the obstruction angle= 10 deg
            Los = 1; //There is Line of Sight
        else
            Los = 0; //No Line of Sight

        LOS[i0] = Los;
    }
    //End of Determination of Line of Sight*/


    int i2;
    double SV[NumSV][3];
        
    for(i2 = 0; i2 < NumSV; i2++){   
    //Assigning coordinates to respective visible satellites
        SV[i2][0] = EASTs[i2];
        SV[i2][1] = NORTHs[i2];
        SV[i2][2] = UPs[i2];
        
    } 
     
    int i3;
    double r[12]=0, Dx[12]=0, Dy[12]=0, Dz[12]=0, Dt[12]=0;

    for(i3=0; i3<NumSV; i3++){
    //Calculate pseudo-ranges from target position to visible satellites
        r[i3] = sqrt( (SV[i3][0] - EASTi)*(SV[i3][0] - EASTi) + (SV[i3][1] - NORTHi)*(SV[i3][1] - NORTHi) + (SV[i3][2] - UPi)*(SV[i3][2] - UPi) );
    //Calculate directional derivatives for East, North, Up and Time
        Dx[i3] = SV[i3][0] / r[i3];
        Dy[i3] = SV[i3][1] / r[i3];
        Dz[i3] = (SV[i3][2] - receiver_llh.hgt) / r[i3];
        Dt[i3] = -1;
    } 

    //Produce the Covariance Matrix from the Directional Derivatives
    int i4, i5;
    double Alp[12][4];
    for (i4=0; i4<12; i4++){
        for (i5=0; i5<4; i5++){
            Alp[i4][i5] = 0; //Initialize Alp
        }
    }
    for (i4=0; i4<NumSV; i4++){
        Alp[i4][0] = Dx[i4];
        Alp[i4][1] = Dy[i4];
        Alp[i4][2] = Dz[i4];
        Alp[i4][3] = Dt[i4];
    } 

    //Transpose Alp to get Brv
    int i6, i7;
    double Brv[4][12];

    for (i6=0; i6<4; i6++){
        for (i7=0; i7<12; i7++){
            Brv[i6][i7] = 0; //Initialize Brv
        }
    }

    for (i6=0; i6<4; i6++){
        i7 = 0;
        while (i7<NumSV){
            Brv[i6][i7] = Alp[i7][i6];
            i7++;
        }  
    }

    //Matrix multiplication of Brv and Alp
    int i8, i9, i10;
    double Chl[4][4];
    for (i8=0; i8<4; i8++){
        for (i9=0; i9<4; i9++){
            Chl[i8][i9] = 0; //Initialize Chl 
        }
    }
    for (i8=0; i8<4; i8++){
        for (i9=0; i9<4; i9++){
            for (i10=0; i10<12; i10++)
                Chl[i8][i9] = Chl[i8][i9] + Brv[i8][i10]*Alp[i10][i9];
        }
    } 
    //calculate |Chl|

    int i11;
    double det_Chl = 0;

    for (i11 = 0; i11 < 4; i11++){

    	det_Chl = det_Chl + pow(-1,i11)*Chl[0][i11]*(Chl[1][(i11+1)%4] * Chl[2][(i11+2)%4] * Chl[3][(i11+3)%4]
    	                                           + Chl[1][(i11+2)%4] * Chl[2][(i11+3)%4] * Chl[3][(i11+1)%4]
    	                                           + Chl[1][(i11+3)%4] * Chl[2][(i11+1)%4] * Chl[3][(i11+2)%4]
    	                                           - Chl[1][(i11+3)%4] * Chl[2][(i11+2)%4] * Chl[3][(i11+1)%4]
    	                                           - Chl[1][(i11+2)%4] * Chl[2][(i11+1)%4] * Chl[3][(i11+3)%4]
    	                                           - Chl[1][(i11+1)%4] * Chl[2][(i11+3)%4] * Chl[3][(i11+2)%4]);

    }


    //Transpose Chl to get Drv
        int i12, i13;
        double Drv[4][4];

        for (i12 = 0; i12 < 4; i12++){
            for (i13=0; i13<4; i13++){
                Drv[i12][i13] = 0; //Initialize Drv
            }
        }

        for (i12 = 0; i12 < 4; i12++){
            i13 = 0;
            while (i13<4){
                Drv[i12][i13] = Chl[i13][i12];
                i13++;
            }  
        }

    //Inverse Matrix 
    double D[4][4];

    D[0][0] = (Drv[1][1] * Drv[2][2] * Drv[3][3] + Drv[1][2] * Drv[2][3] * Drv[3][1] + Drv[1][3] * Drv[2][1] * Drv[3][2]  - Drv[1][3] * Drv[2][2] * Drv[3][1]  - Drv[1][2] * Drv[2][1] * Drv[3][3]  - Drv[1][1] * Drv[2][3] * Drv[3][2]) / det_Chl;
/*    D[0][1] = (Drv[1][3] * Drv[2][2] * Drv[3][0] + Drv[1][2] * Drv[2][0] * Drv[3][3] + Drv[1][0] * Drv[2][3] * Drv[3][2]  - Drv[1][0] * Drv[2][2] * Drv[3][3]  - Drv[2][0] * Drv[3][2] * Drv[1][3]  - Drv[3][0] * Drv[1][2] * Drv[2][3]) / det_Chl;
    D[0][2] = (Drv[1][0] * Drv[2][1] * Drv[3][3] + Drv[1][3] * Drv[2][0] * Drv[3][1] + Drv[1][1] * Drv[2][3] * Drv[3][0]  - Drv[1][3] * Drv[2][1] * Drv[3][0]  - Drv[1][1] * Drv[2][0] * Drv[3][3]  - Drv[1][0] * Drv[2][3] * Drv[3][1]) / det_Chl;
    D[0][3] = (Drv[1][2] * Drv[2][1] * Drv[3][0] + Drv[1][1] * Drv[2][0] * Drv[3][2] + Drv[1][0] * Drv[2][2] * Drv[3][1]  - Drv[1][0] * Drv[2][1] * Drv[3][2]  - Drv[1][1] * Drv[2][2] * Drv[3][0]  - Drv[1][2] * Drv[2][0] * Drv[3][1]) / det_Chl;
*/
 //   D[1][0] = (Drv[0][3] * Drv[2][2] * Drv[3][1] + Drv[0][2] * Drv[2][1] * Drv[3][3] + Drv[0][1] * Drv[2][3] * Drv[3][2]  - Drv[0][1] * Drv[2][2] * Drv[3][3]  - Drv[0][2] * Drv[2][3] * Drv[3][1]  - Drv[0][3] * Drv[2][1] * Drv[3][2]) / det_Chl;
    D[1][1] = (Drv[0][0] * Drv[2][2] * Drv[3][3] + Drv[0][2] * Drv[2][3] * Drv[3][0] + Drv[0][3] * Drv[2][0] * Drv[3][2]  - Drv[0][3] * Drv[2][2] * Drv[3][0]  - Drv[0][2] * Drv[2][0] * Drv[3][3]  - Drv[0][0] * Drv[2][3] * Drv[3][2]) / det_Chl;
 /*   D[1][2] = (Drv[0][3] * Drv[2][1] * Drv[3][0] + Drv[0][1] * Drv[2][0] * Drv[3][3] + Drv[0][0] * Drv[2][3] * Drv[3][1]  - Drv[0][0] * Drv[2][1] * Drv[3][3]  - Drv[0][1] * Drv[2][3] * Drv[3][0]  - Drv[0][3] * Drv[2][0] * Drv[3][1]) / det_Chl;
    D[1][3] = (Drv[0][0] * Drv[2][1] * Drv[3][2] + Drv[0][1] * Drv[2][2] * Drv[3][0] + Drv[0][2] * Drv[2][0] * Drv[3][1]  - Drv[0][2] * Drv[2][1] * Drv[3][0]  - Drv[0][1] * Drv[2][0] * Drv[3][2]  - Drv[0][0] * Drv[2][2] * Drv[3][1]) / det_Chl;
*/
 //   D[2][0] = (Drv[0][1] * Drv[1][2] * Drv[3][3] + Drv[0][2] * Drv[1][3] * Drv[3][1] + Drv[0][3] * Drv[1][1] * Drv[3][2]  - Drv[0][3] * Drv[1][2] * Drv[3][1]  - Drv[0][2] * Drv[1][1] * Drv[3][3]  - Drv[0][1] * Drv[1][3] * Drv[3][2]) / det_Chl;
//    D[2][1] = (Drv[0][3] * Drv[1][2] * Drv[3][0] + Drv[0][2] * Drv[1][0] * Drv[3][3] + Drv[0][0] * Drv[1][3] * Drv[3][2]  - Drv[0][0] * Drv[1][2] * Drv[3][3]  - Drv[0][2] * Drv[1][3] * Drv[3][0]  - Drv[0][3] * Drv[1][0] * Drv[3][2]) / det_Chl;
    D[2][2] = (Drv[0][0] * Drv[1][1] * Drv[3][3] + Drv[0][1] * Drv[1][3] * Drv[3][0] + Drv[0][3] * Drv[1][0] * Drv[3][1]  - Drv[0][3] * Drv[1][1] * Drv[3][0]  - Drv[0][1] * Drv[1][0] * Drv[3][3]  - Drv[0][0] * Drv[1][3] * Drv[3][1]) / det_Chl;
 //   D[2][3] = (Drv[0][2] * Drv[1][1] * Drv[3][0] + Drv[0][1] * Drv[1][0] * Drv[3][2] + Drv[0][0] * Drv[1][2] * Drv[3][1]  - Drv[0][0] * Drv[1][1] * Drv[3][2]  - Drv[0][1] * Drv[1][2] * Drv[3][0]  - Drv[0][2] * Drv[1][0] * Drv[3][1]) / det_Chl;

 /*   D[3][0] = (Drv[0][3] * Drv[1][2] * Drv[2][1] + Drv[0][2] * Drv[1][1] * Drv[2][3] + Drv[0][1] * Drv[1][3] * Drv[2][2]  - Drv[0][1] * Drv[1][2] * Drv[2][3]  - Drv[0][2] * Drv[1][3] * Drv[2][1]  - Drv[0][3] * Drv[1][1] * Drv[2][2]) / det_Chl;
    D[3][1] = (Drv[0][0] * Drv[1][2] * Drv[2][3] + Drv[0][2] * Drv[1][3] * Drv[2][0] + Drv[0][3] * Drv[1][0] * Drv[2][2]  - Drv[0][3] * Drv[1][2] * Drv[2][0]  - Drv[0][2] * Drv[1][0] * Drv[2][3]  - Drv[0][0] * Drv[1][3] * Drv[2][2]) / det_Chl;
    D[3][2] = (Drv[0][3] * Drv[1][1] * Drv[2][0] + Drv[0][1] * Drv[1][0] * Drv[2][3] + Drv[0][0] * Drv[1][3] * Drv[2][1]  - Drv[0][0] * Drv[1][1] * Drv[2][3]  - Drv[0][1] * Drv[1][3] * Drv[2][0]  - Drv[0][3] * Drv[1][0] * Drv[2][1]) / det_Chl;
*/
    D[3][3] = (Drv[0][0] * Drv[1][1] * Drv[2][2] + Drv[0][1] * Drv[1][2] * Drv[2][0] + Drv[0][2] * Drv[1][0] * Drv[2][1]  - Drv[0][2] * Drv[1][1] * Drv[2][0]  - Drv[0][1] * Drv[1][0] * Drv[2][2]  - Drv[0][0] * Drv[1][2] * Drv[2][1]) / det_Chl;

/*    //XDOP YDOP VDOP TDOP
    double XDOP, YDOP, VDOP, TDOP;
    XDOP = sqrt( D[0][0] );
    YDOP = sqrt( D[1][1] );
    VDOP = sqrt( D[2][2] );
    TDOP = sqrt( D[3][3] );
*/
    // HDOP PDOP GDOP 
    DOP NEWDOP;
    NEWDOP.VDOP = sqrt( D[2][2] );
    NEWDOP.HDOP = sqrt( D[0][0] + D[1][1] );
    NEWDOP.PDOP = sqrt( D[0][0] + D[1][1] + D[2][2] );
    NEWDOP.GDOP = sqrt( D[0][0] + D[1][1] + D[2][2] + D[3][3] );
    NEWDOP.Num_of_SV = NumSV ;
    /*
    DOP NEWDOP;
        NEWDOP.HDOP =  add;
        NEWDOP.PDOP =  mus;
        NEWDOP.GDOP =  det_Chl;
        NEWDOP.Num_of_SV = NumSV ;
	*/
    
    return NEWDOP;
}
