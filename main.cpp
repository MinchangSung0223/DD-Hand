
/* 
* TCP Example For KIST Monkey Experiment 
* TCP_HYUSPA.cpp
* Created on: Mar 2, 2020
*     Author: Sunhong Kim
*/

#include "Poco/Net/Net.h"
#include "Poco/Net/StreamSocket.h"
#include "Poco/Net/SocketAddress.h"
#include "Poco/Dynamic/Var.h"
#include "Poco/Exception.h"
#include "Poco/Timer.h"
#include "Poco/Stopwatch.h"
#include "Poco/Thread.h"
#include "Poco/DateTime.h"
#include "Poco/Net/ServerSocket.h"
#include "Poco/Net/SocketAddress.h"
#include "Poco/Timespan.h"
#include "Poco/NumericString.h"
#include <iostream>
#include <time.h>
#include <signal.h>
using namespace Poco;
using namespace Poco::Dynamic;
using Poco::Net::SocketAddress;
using Poco::Net::StreamSocket;
using Poco::Net::Socket;
using Poco::Timer;
using Poco::TimerCallback;
using Poco::Thread;
using Poco::Stopwatch;
using namespace std;




union RecvData
{
    unsigned char byte[10];
    unsigned int cmd[5];
};


//float T_kp[4] ={10,10,10,10};
float T_kp[4] ={10,10,15,10};
float I_kp[4] = {10,10,15,10};
float M_kp[4] = {10,10,15,10};
float R_kp[4] = {8,10,5,10};
float P_kp[2] = {10,10};
//float T_kd[4] = {0,0,0,0};
//float I_kd[4] = {0,0,0,0};
//float M_kd[4] = {0,0,0,0};
//float R_kd[4] = {0,0,0,0};


//float T_kd[4] = {0.1,0.1,0.1,0.1};
float T_kd[4] ={0.1,0.1,0.1,0.1};
float I_kd[4] = {0.1,0.1,0.1,0.1};
float M_kd[4] =  {0.1,0.1,0.1,0.1};
float R_kd[4] = {0.1,0.1,0.05,0.1};//{0.1,0.1,0.1,0.1};
float P_kd[2] =  {0.05,0.05};


float T_ki[4] = {0,0,0,0};
float I_ki[4] = {0,0,0,0};
float M_ki[4] = {0,0,0,0};
float R_ki[4] = {0,0,0,0};
float P_ki[2] = {0,0};


uint16_t T_target[4] = {20074, 27948, 41607, 15328};
uint16_t I_target[4] = {21482,27061,35691,16452};
uint16_t M_target[4] = {25610,24106,34374,16104};
uint16_t R_target[4] = {24967,25692,2876,15877};
uint16_t P_target[2] = {24858,24464};

uint16_t T_min_torque[4] = {12000,12000,8000,12000};
uint16_t I_min_torque[4] = {12000,12000,8000,12000};
uint16_t M_min_torque[4] =  {12000,12000,8000,12000};
uint16_t R_min_torque[4] =  {12000,12000,8000,12000};
uint16_t P_min_torque[2] = {10000,8000};





float T_vel[4] = {0,0,0,0};
float I_vel[4] = {0,0,0,0};
float M_vel[4] = {0,0,0,0};
float R_vel[4] = {0,0,0,0};
float P_vel[2] ={0,0};

float preT_vel[4] = {0,0,0,0};
float preI_vel[4] = {0,0,0,0};
float preM_vel[4] = {0,0,0,0};
float preR_vel[4] = {0,0,0,0};
float preP_vel[2] ={0,0};

float T = 0.002;
float tau =  0.01;


//
//uint16_t T_target[4] = {0x000, 0x000, 0x000, 0x000};
//uint16_t I_target[4] = {0x000, 0x000, 0x000, 0x000};
//uint16_t M_target[4] = {0x000, 0x000, 0x000, 0x000};
//uint16_t R_target[4] = {0x000, 0x000, 0x000, 0x000};

volatile uint16_t T_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
volatile uint16_t I_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
volatile uint16_t M_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
volatile uint16_t R_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
volatile uint16_t P_pos[2] = {0x8000,0x8000};

uint16_t preT_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preI_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preM_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preR_pos[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preP_pos[2] = {0x8000,0x8000};

uint16_t preT_err[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preI_err[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preM_err[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preR_err[4] = {0x8000, 0x8000, 0x8000, 0x8000};
uint16_t preP_err[2] = {0x8000,0x8000};
                      //T_torque[2] = offset+1000;-
                //T_torque[1] = offset+7500;+
                //T_torque[0] = offset+12000;+
                //T_torque[3] = offset+9500;+
                
uint16_t fric_offsetT[4] = {7000,3000,1000,3000};
uint16_t fric_offsetI[4] = {4000,3000,1000,3000};
uint16_t fric_offsetM[4] = {7000,3000,1000,3000};
uint16_t fric_offsetR[4] = {2000,3000,500,3000};
uint16_t fric_offsetP[2] = {7000,8000};



int T_Err[4] = {0,0,0,0};
int I_Err[4] = {0,0,0,0};
int M_Err[4] = {0,0,0,0};
int R_Err[4] = {0,0,0,0};
int P_Err[2] =  {0,0};


int T_sumErr[4] = {0,0,0,0};
int I_sumErr[4] = {0,0,0,0};
int M_sumErr[4] = {0,0,0,0};
int R_sumErr[4] = {0,0,0,0};
int P_sumErr[2] =  {0,0};







float T_torque[4] = {0x8000, 0x8000, 0x8000, 0x8000};
float I_torque[4] = {0x8000, 0x8000, 0x8000, 0x8000};
float M_torque[4] = {0x8000, 0x8000, 0x8000, 0x8000};
float R_torque[4] = {0x8000, 0x8000, 0x8000, 0x8000};
float P_torque[4] = {0x8000,0x8000};

uint16_t offset = 0x8000;
uint8_t dataDivide_flag = 0; // 0: motor , 1: sensor

bool rxSuccess_flag = false;
bool prePos_flag = false;
bool kill_flag = false;
bool torque_flag = false;
const std::string hostname = "169.254.186.72"; //STEP2 IP Address

const Poco::UInt16 PORT = 7;
StreamSocket ss;
enum {
    SIZE_HEADER = 52,
    SIZE_COMMAND = 4,
    SIZE_HEADER_COMMAND = 56,
    SIZE_DATA_MAX = 37,
    SIZE_DATA_ASCII_MAX = 37
};

void ctrlchandler(int)
{
    kill_flag = 1;
    usleep(1000000);

    exit(EXIT_SUCCESS);
}

void killhandler(int)
{

    kill_flag = 1;
    usleep(1000000);
    exit(EXIT_SUCCESS);
}





class HYUControl: public Poco::Runnable{
    
    virtual void run(){
        unsigned char writeBuff[37];
        writeBuff[0] = 0x00;
        for(int i=0 ; i<18 ; i++){
            writeBuff[i*2+1] = 0x80;
            writeBuff[i*2+2] = 0x00;
        }

        ss.sendBytes(writeBuff,37,0);
	int toggle = 1;
	unsigned int count = 0;
        while(!kill_flag){
		
            unsigned char receiveBuff[37];
	    ss.receiveBytes(receiveBuff,37,0);
            usleep(2000); 
            for(int i=0 ; i<4 ; i++){ // 32/2/4 -> 32/8 -> 4
                T_pos[i] = ((receiveBuff[i*2] << 8) & 0xFF00) | (receiveBuff[i*2+1] & 0x00FF);
                I_pos[i] = ((receiveBuff[i*2+8] << 8) & 0xFF00) | (receiveBuff[i*2+9] & 0x00FF);
                M_pos[i] = ((receiveBuff[i*2+16] << 8) & 0xFF00) | (receiveBuff[i*2+17] & 0x00FF);
                R_pos[i] = ((receiveBuff[i*2+24] << 8) & 0xFF00) | (receiveBuff[i*2+25] & 0x00FF);
                
                T_vel[i] = (preT_pos[i]-T_pos[i])/T;
                I_vel[i] = (preI_pos[i]-I_pos[i])/T;
                M_vel[i] = (preM_pos[i]-M_pos[i])/T;
                R_vel[i] = (preR_pos[i]-R_pos[i])/T;
                
            }
	    for(int i =0;i<2;i++){
	    	P_pos[i] = ((receiveBuff[i*2+32] << 8) & 0xFF00) | (receiveBuff[i*2+33]& 0x00FF);
                P_vel[i] = (preP_pos[i]-P_pos[i])/T;
	    }
            for(int i=0; i<4; i++){
                T_Err[i] = T_target[i] - T_pos[i];
                I_Err[i] = I_target[i] - I_pos[i];
                M_Err[i] = M_target[i] - M_pos[i];
                R_Err[i] = R_target[i] - R_pos[i];
                
        	T_sumErr[i] = T_sumErr[i]+ T_Err[i]*T;   
        	I_sumErr[i] = I_sumErr[i]+ I_Err[i]*T;   
        	M_sumErr[i] = M_sumErr[i]+ M_Err[i]*T;   
        	R_sumErr[i] = R_sumErr[i]+ R_Err[i]*T;   
        	     
            }
	    for(int i = 0;i<2;i++){
		P_Err[i] = P_target[i] - P_pos[i];
		P_sumErr[i] = P_sumErr[i]+ P_Err[i]*T;   
        	
	    
	    }
                cout << "\x1B[2J\x1B[H";
/*
	    cout<<"T_Err : "<<T_Err[0]<<","<<T_Err[1]<<","<<T_Err[2]<<","<<T_Err[3] <<std::endl;
            cout<<"T_sumErr : "<<T_sumErr[0]<<","<<T_sumErr[1]<<","<<T_sumErr[2]<<","<<T_sumErr[3] <<std::endl;
	    cout<<"T_pos : "<<T_pos[0]<<","<<T_pos[1]<<","<<T_pos[2]<<","<<T_pos[3] <<std::endl;
    	    cout<<"T_vel : "<<T_vel[0]<<","<<T_vel[1]<<","<<T_vel[2]<<","<<T_vel[3] <<std::endl;
*/
/*
	    cout<<"I_Err : "<<I_Err[0]<<","<<I_Err[1]<<","<<I_Err[2]<<","<<I_Err[3] <<std::endl;
            cout<<"I_sumErr : "<<I_sumErr[0]<<","<<I_sumErr[1]<<","<<I_sumErr[2]<<","<<I_sumErr[3] <<std::endl;
	    cout<<"I_pos : "<<I_pos[0]<<","<<I_pos[1]<<","<<I_pos[2]<<","<<I_pos[3] <<std::endl;
    	    cout<<"I_vel : "<<I_vel[0]<<","<<I_vel[1]<<","<<I_vel[2]<<","<<I_vel[3] <<std::endl;
*/
/*
	    cout<<"R_Err : "   <<R_Err[0]<<","   <<R_Err[1]<<","   <<R_Err[2]<<","   <<R_Err[3] <<std::endl;
            cout<<"R_sumErr : "<<R_sumErr[0]<<","<<R_sumErr[1]<<","<<R_sumErr[2]<<","<<R_sumErr[3] <<std::endl;
	    cout<<"R_pos : "   <<R_pos[0]<<",   "<<R_pos[1]<<","   <<R_pos[2]<<","   <<R_pos[3] <<std::endl;
    	    cout<<"R_vel : "   <<R_vel[0]<<","   <<R_vel[1]<<","   <<R_vel[2]<<","   <<R_vel[3] <<std::endl;
*/
/*
	    cout<<"M_Err : "   <<M_Err[0]<<","   <<M_Err[1]<<","   <<M_Err[2]<<","   <<M_Err[3] <<std::endl;
            cout<<"M_sumErr : "<<M_sumErr[0]<<","<<M_sumErr[1]<<","<<M_sumErr[2]<<","<<M_sumErr[3] <<std::endl;
	    cout<<"M_pos : "   <<M_pos[0]<<",   "<<M_pos[1]<<","   <<M_pos[2]<<","   <<M_pos[3] <<std::endl;
    	    cout<<"M_vel : "   <<M_vel[0]<<","   <<M_vel[1]<<","   <<M_vel[2]<<","   <<M_vel[3] <<std::endl;
*/

/*
	    cout<<"P_Err : "   <<P_Err[0]<<","   <<P_Err[1]<<","   <<P_Err[2]<<","   <<P_Err[3] <<std::endl;
            cout<<"P_sumErr : "<<P_sumErr[0]<<","<<P_sumErr[1]<<","<<P_sumErr[2]<<","<<P_sumErr[3] <<std::endl;
	    cout<<"P_pos : "   <<P_pos[0]<<",   "<<P_pos[1]<<","   <<P_pos[2]<<","   <<P_pos[3] <<std::endl;
    	    cout<<"P_vel : "   <<P_vel[0]<<","   <<P_vel[1]<<","   <<P_vel[2]<<","   <<P_vel[3] <<std::endl;

*/

	    cout<<"T_pos : "<<T_pos[0]<<","<<T_pos[1]<<","<<T_pos[2]<<","<<T_pos[3] <<std::endl;
	    cout<<"I_pos : "<<I_pos[0]<<","<<I_pos[1]<<","<<I_pos[2]<<","<<I_pos[3] <<std::endl;
	    cout<<"M_pos : "   <<M_pos[0]<<",   "<<M_pos[1]<<","   <<M_pos[2]<<","   <<M_pos[3] <<std::endl;
	    cout<<"R_pos : "   <<R_pos[0]<<",   "<<R_pos[1]<<","   <<R_pos[2]<<","   <<R_pos[3] <<std::endl;

	    cout<<"P_pos : "   <<P_pos[0]<<",   "<<P_pos[1]<<","   <<P_pos[2]<<","   <<P_pos[3] <<std::endl;
	    
            for(int i=0; i<4; i++){

               T_torque[i] = (float)(T_kp[i]*(T_Err[i])+T_kd[i]*(float)(T_vel[i]))+(float)(T_ki[i]*T_sumErr[i])+offset; //offset = 0x8000
               I_torque[i] = (float)(I_kp[i]*(I_Err[i])+I_kd[i]*(float)(I_vel[i]))+(float)(I_ki[i]*I_sumErr[i])+offset; //offset = 0x8000
               M_torque[i] = (float)(M_kp[i]*(M_Err[i])+M_kd[i]*(float)(M_vel[i]))+(float)(M_ki[i]*M_sumErr[i])+offset; //offset = 0x8000
               R_torque[i] = (float)(R_kp[i]*(R_Err[i])+R_kd[i]*(float)(R_vel[i]))+(float)(R_ki[i]*R_sumErr[i])+offset; //offset = 0x8000
                if(T_torque[i]<0+T_min_torque[i]) T_torque[i] = T_min_torque[i];
                if(I_torque[i]<0+I_min_torque[i]) I_torque[i] = I_min_torque[i];
                if(M_torque[i]<0+M_min_torque[i]) M_torque[i] = M_min_torque[i];
                if(R_torque[i]<0+R_min_torque[i]) R_torque[i] = R_min_torque[i];

                if(T_torque[i]>65534-T_min_torque[i]) T_torque[i] = 65534-T_min_torque[i];
                if(I_torque[i]>65534-I_min_torque[i]) I_torque[i] = 65534-I_min_torque[i];
                if(M_torque[i]>65534-M_min_torque[i]) M_torque[i] = 65534-M_min_torque[i];
                if(R_torque[i]>65534-R_min_torque[i]) R_torque[i] = 65534-R_min_torque[i];
                
                if(abs(T_torque[i]-offset)<abs(fric_offsetT[i])){
                	if((T_torque[i]-offset)<0){
                		T_torque[i] = offset-fric_offsetT[i];
                	}
                	else if((T_torque[i]-offset)>=0){
                		T_torque[i] = offset+fric_offsetT[i];
                	}
                }
                
                if(abs(I_torque[i]-offset)<abs(fric_offsetI[i])){
                	if((I_torque[i]-offset)<0){
                		I_torque[i] = offset-fric_offsetI[i];
                	}
                	else if((I_torque[i]-offset)>=0){
                		I_torque[i] = offset+fric_offsetI[i];
                	}
                }
                if(abs(M_torque[i]-offset)<abs(fric_offsetM[i])){
                	if((M_torque[i]-offset)<0){
                		M_torque[i] = offset-fric_offsetM[i];
                	}
                	else if((M_torque[i]-offset)>=0){
                		M_torque[i] = offset+fric_offsetM[i];
                	}
                }
                if(abs(R_torque[i]-offset)<abs(fric_offsetR[i])){
                	if((R_torque[i]-offset)<0){
                		R_torque[i] = offset-fric_offsetR[i];
                	}
                	else if((R_torque[i]-offset)>=0){
                		R_torque[i] = offset+fric_offsetR[i];
                	}
                }    
                if(torque_flag == false){          
			T_torque[i] = offset;
			I_torque[i] = offset;
			M_torque[i] = offset;
			R_torque[i] = offset;
		}
		
            }
            for(int i = 0;i<2;i++){
                P_torque[i] = (float)(P_kp[i]*(P_Err[i])+P_kd[i]*(float)(P_vel[i]))+(float)(P_ki[i]*P_sumErr[i])+offset; //offset = 0x8000
		if(P_torque[i]<0+T_min_torque[i])P_torque[i] = T_min_torque[i];
		if(P_torque[i]>65534-P_min_torque[i])P_torque[i]=65534-P_min_torque[i];
                if(abs(P_torque[i]-offset)<abs(fric_offsetP[i])){
                	if((P_torque[i]-offset)<0){
                		P_torque[i] = offset-fric_offsetP[i];
                	}
                	else if((P_torque[i]-offset)>=0){
                		P_torque[i] = offset+fric_offsetP[i];
                	}
                }
                if(torque_flag == false)
                	P_torque[i] =offset;         

	    }
	  //   cout<<"T_torque : "<<T_torque[0]-0x8000<<","<<T_torque[1]-0x8000<<","<<T_torque[2]-0x8000<<","<<T_torque[3] -0x8000<<std::endl;
	  //   cout<<"I_torque : "<<I_torque[0]-0x8000<<","<<I_torque[1]-0x8000<<","<<I_torque[2]-0x8000<<","<<I_torque[3] -0x8000<<std::endl;
	  //   cout<<"T_torque : "<<T_torque[0]-0x8000<<","<<T_torque[1]-0x8000<<","<<T_torque[2]-0x8000<<","<<T_torque[3] -0x8000<<std::endl;
	  //   cout<<"R_torque : "<<R_torque[0]-0x8000<<","<<R_torque[1]-0x8000<<","<<R_torque[2]-0x8000<<","<<R_torque[3] -0x8000<<std::endl;
	     //cout<<"P_torque : "<<P_torque[0]-0x8000<<","<<P_torque[1]-0x8000<<","<<P_torque[2]-0x8000<<","<<P_torque[3] -0x8000<<std::endl;
	     
	     
            for(int i=0 ; i<4 ; i++){
                preT_pos[i] = T_pos[i];
                preI_pos[i] = I_pos[i];
                preM_pos[i] = M_pos[i];
                preR_pos[i] = R_pos[i];

            }
	    for(int i=0;i<2;i++){
	    	preP_pos[i] = P_pos[i];

	    }


            unsigned char TIMR_Duty[37];
            TIMR_Duty[0] = 0x00; // ID
            for(int i=0; i<4; i++){
                TIMR_Duty[i*2+1] = ((int)T_torque[i] >> 8) & 0x00FF;
                TIMR_Duty[i*2+2] = (int)T_torque[i] & 0x00FF;

                TIMR_Duty[i*2+9] = ((int)I_torque[i] >> 8) & 0x00FF;
                TIMR_Duty[i*2+10] = (int)I_torque[i] & 0x00FF;

                TIMR_Duty[i*2+17] = ((int)M_torque[i] >> 8) & 0x00FF;
                TIMR_Duty[i*2+18] = (int)M_torque[i] & 0x00FF;

                TIMR_Duty[i*2+25] = ((int)R_torque[i] >> 8) & 0x00FF;
                TIMR_Duty[i*2+26] = (int)R_torque[i] & 0x00FF;
            }
            for(int i =0;i<2;i++){
	    	TIMR_Duty[i*2+33] =  ((int)P_torque[i] >> 8 ) & 0x00FF;
		TIMR_Duty[i*2+34] = (int)P_torque[i] & 0x00FF;
	    }

            ss.sendBytes(TIMR_Duty,37,0);

        }
        
        
        
        
        
        // finish 
		for(int i=0; i<4; i++){
			T_torque[i]= offset ;
			I_torque[i]= offset ;
			M_torque[i]= offset ;
			R_torque[i]= offset ;
		}for(int i=0;i<2;i++){
			P_torque[i]= offset ;
		}

	    unsigned char TIMR_Duty[37];
	    TIMR_Duty[0] = 0x00; // ID
	    for(int i=0; i<4; i++){
		TIMR_Duty[i*2+1] = ((int)T_torque[i] >> 8) & 0x00FF;
		TIMR_Duty[i*2+2] = (int)T_torque[i] & 0x00FF;

		TIMR_Duty[i*2+9] = ((int)I_torque[i] >> 8) & 0x00FF;
		TIMR_Duty[i*2+10] = (int)I_torque[i] & 0x00FF;

		TIMR_Duty[i*2+17] = ((int)M_torque[i] >> 8) & 0x00FF;
		TIMR_Duty[i*2+18] = (int)M_torque[i] & 0x00FF;

		TIMR_Duty[i*2+25] = ((int)R_torque[i] >> 8) & 0x00FF;
		TIMR_Duty[i*2+26] = (int)R_torque[i] & 0x00FF;
	    }
	    for(int i =0;i<2;i++){
	    	TIMR_Duty[i*2+33] =  ((int)P_torque[i] >> 8 ) & 0x00FF;
		TIMR_Duty[i*2+34] = (int)P_torque[i] & 0x00FF;
	    }

	    ss.sendBytes(TIMR_Duty,37,0);
            unsigned char receiveBuff[37];
	    ss.receiveBytes(receiveBuff,37,0);
            usleep(2000); 
            
            for(int i =0;i<5;i++){
	    	    ss.sendBytes(TIMR_Duty,37,0);
		    ss.receiveBytes(receiveBuff,37,0);
		    usleep(2000); 
            }


	    usleep(100000);
         ss.close();
        

    }

};
union Data
        {
    unsigned char byte[SIZE_DATA_MAX];
        };

int main(int argc, char **argv)
{




    if(argc>=2){
	std::cout<<argv[1]<<std::endl;
	if(atoi(argv[1]) == 1)
		torque_flag = true;
	else
		torque_flag = false;
    }
    else{
    	return 0;
    }
    
    
    
    
    Data data_rev;


    signal(SIGINT, ctrlchandler);
    signal(SIGTERM, killhandler);

    HYUControl hyu;
    Poco::Thread thread;


    try
    {
        cout << "Trying to connect Hand server..." << endl;
        ss.connect(SocketAddress(hostname, PORT));
        Timespan timeout(1, 0);
        while (ss.poll(timeout, Poco::Net::Socket::SELECT_WRITE) == false)
        {
            cout << "Connecting to Hand server..." << endl;
        }


        cout << "Trying to connect Hand server..." << endl;

        thread.start(hyu);
        int count = 0;

        





        Poco::Net::SocketAddress server_addr(9911);
        Poco::Net::ServerSocket server_sock(server_addr);
        Poco::Net::Socket::SocketList connectedSockList;

        connectedSockList.push_back(server_sock);
        RecvData data_rev;
            cout << "TCP Server is on" << endl;
        
        while (true){
            Poco::Net::Socket::SocketList readList(connectedSockList.begin(), connectedSockList.end());
            Poco::Net::Socket::SocketList writeList(connectedSockList.begin(), connectedSockList.end());
            Poco::Net::Socket::SocketList exceptList(connectedSockList.begin(), connectedSockList.end());
            Poco::Timespan timeout(1);
            auto count = Poco::Net::Socket::select(readList, writeList, exceptList, timeout);
            if (count == 0)
                continue;

            Poco::Net::Socket::SocketList delSockList;
            for (auto& readSock : readList)
                {
                    if (server_sock == readSock)
                    {
                        auto newSock = server_sock.acceptConnection();
                        connectedSockList.push_back(newSock);
                        std::cout << "NEW CLIENT CONNECTED" << std::endl;
                    }
                    else
                    {
                        char buffer[10] = { 0 };
                        auto n = ((Poco::Net::StreamSocket*)&readSock)->receiveBytes(buffer, 10);
                        if (n > 0)
                        {
                            std::cout << "RECEIVED MESSEGE: " <<buffer << std::endl;
                            memcpy(data_rev.byte, buffer, SIZE_DATA_MAX);
                            std::cout << data_rev.cmd[0]<<std::endl;
                            if ( data_rev.cmd[0] == 1){
                                 std::cout << "MOTION 1 " <<buffer << std::endl;

                                T_target[0] = 20074;
                                T_target[1] = 27948;
                                T_target[2] = 41607;
                                T_target[3] = 15328;

                                I_target[0] = 21482;
                                I_target[1] = 27061;
                                I_target[2] = 35691;
                                I_target[3] = 16452;
                                
                                M_target[0] = 25610;
                                M_target[1] = 24106;
                                M_target[2] = 34374;
                                M_target[3] = 16104;
                                
                                R_target[0] = 24967;
                                R_target[1] = 25692;
                                R_target[2] = 2876;
                                R_target[3] = 15877;
                                
                                P_target[0] = 24858;
                                P_target[1] = 24464;

                                                            
                            }
                            else if ( data_rev.cmd[0] == 3){
                                 std::cout << "MOTION 3  "<< data_rev.cmd[0]<<buffer << std::endl;
                                T_target[0] = 19931;
                                T_target[1] = 27875;
                                T_target[2] = 36623;
                                T_target[3] = 14998;

                                I_target[0] = 23430;
                                I_target[1] = 28226;
                                I_target[2] = 36480;
                                I_target[3] = 15966;
                                
                                M_target[0] = 25737;
                                M_target[1] = 25587;
                                M_target[2] = 34420;
                                M_target[3] = 16702;
                                
                                R_target[0] = 23250;
                                R_target[1] = 29299;
                                R_target[2] = 3111;
                                R_target[3] = 16053;
                                
                                P_target[0] = 24866;
                                P_target[1] = 53274;
                                


                            }
                            else if ( data_rev.cmd[0] == 2){
                                 std::cout << "MOTION 2 "<< data_rev.cmd[0]<<buffer << std::endl;
                                T_target[0] = 21000;
                                T_target[1] = 26194;
                                T_target[2] = 40564;
                                T_target[3] = 14220;

                                I_target[0] = 22103;
                                I_target[1] = 27062;
                                I_target[2] = 38630;
                                I_target[3] = 15958;
                                
                                M_target[0] = 25737;
                                M_target[1] = 24325;
                                M_target[2] = 38630;
                                M_target[3] = 15958;
                                
                                R_target[0] = 20413;
                                R_target[1] = 29227;
                                R_target[2] = 10612;
                                R_target[3] = 15983;
                                
                                P_target[0] = 24859;
                                P_target[1] = 53273;

                            }

                            else if ( data_rev.cmd[0] == 4){
                                 std::cout << "MOTION 4 "<< data_rev.cmd[0]<<buffer << std::endl;
                                 T_target[0] = 24518;
                                T_target[1] = 26434;
                                T_target[2] = 38169;
                                T_target[3] = 14343;

                                I_target[0] = 16876;
                                I_target[1] = 32498;
                                I_target[2] = 33184;
                                I_target[3] = 17268;
                                
                                M_target[0] = 18083;
                                M_target[1] = 30353;
                                M_target[2] = 31385;
                                M_target[3] = 15292;
                                
                                R_target[0] = 25662;
                                R_target[1] = 30616;
                                R_target[2] = 7276;
                                R_target[3] = 17338;
                                
                                P_target[0] = 25442;
                                P_target[1] = 52961;
                            }

                            else if ( data_rev.cmd[0] == 5){
                                 std::cout << "MOTION 5 "<< data_rev.cmd[0]<<buffer << std::endl;
                                   T_target[0] = 20074;
                                T_target[1] = 27948;
                                T_target[2] = 41607;
                                T_target[3] = 15328;

                                I_target[0] = 21482;
                                I_target[1] = 27061;
                                I_target[2] = 35691;
                                I_target[3] = 16452;
                                
                                M_target[0] = 25610;
                                M_target[1] = 24106;
                                M_target[2] = 34374;
                                M_target[3] = 16104;
                                
                                R_target[0] = 24967;
                                R_target[1] = 25692;
                                R_target[2] = 2876;
                                R_target[3] = 15877;
                                
                                P_target[0] = 24858;
                                P_target[1] = 24464;
                            }
                            else if ( data_rev.cmd[0] == 6){
                                 std::cout << "MOTION 6 "<< data_rev.cmd[0]<<buffer << std::endl;
                                T_target[0] = 20074;
                                T_target[1] = 27948;
                                T_target[2] = 41607;
                                T_target[3] = 15328;

                                I_target[0] = 21482;
                                I_target[1] = 27061;
                                I_target[2] = 35691;
                                I_target[3] = 16452;
                                
                                M_target[0] = 25610;
                                M_target[1] = 24106;
                                M_target[2] = 34374;
                                M_target[3] = 16104;
                                
                                R_target[0] = 24967;
                                R_target[1] = 25692;
                                R_target[2] = 2876;
                                R_target[3] = 15877;
                                
                                P_target[0] = 24858;
                                P_target[1] = 24464;
                            }
                            std::cout << "-------------------------" <<std::endl;
                        }
                        else
                        {
                            std::cout << "클라이언트와 연결이 끊어졌습니다." << std::endl;
                            delSockList.push_back(readSock);
                        }
                    }
                }
         
                for (auto& delSock : delSockList)
                {
                    // 삭제할 소켓을 검색한다.
                    auto delIter = std::find_if(connectedSockList.begin(),
                        connectedSockList.end(),
                        [&delSock](auto& sock)
                        {
                            return delSock == sock ? true : false;
                        }
                    );
         
                    if (delIter != connectedSockList.end())
                    {
                        connectedSockList.erase(delIter);
                        std::cout << "connectedSockList 에서 socket 제거" << std::endl;
                    }
                }
        }






    }
    catch (Poco::Exception& exc)
    {
        cout << "Fail to connect server..." << exc.displayText() << endl;
    }
    ss.close();
    thread.join();


    return 0;
}



