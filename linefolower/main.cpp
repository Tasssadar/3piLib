/*
 * Line_Folower01.cpp
 * Created: 20.12.2019 22:27:22
 * Author : Šimon Skládaný, David Skrob
 */ 

#include "3piLibPack.h"

#define Motor_ol 0,50
#define Motor_or 50,0 // fakt musis zatacet
#define Motor_or_low 50,60
#define Motor_or_low 60,50 // pomala zatacka,ale projedes rychle 
#define Motor_rov 50,50

#define time 100
#define rmot 50
#define lmot 20

void run(void)
{
	buzzer.start();
	delay(100);
	buzzer.stop();
	display.printToXY("press", 1, 0);
	display.printToXY("Button_B", 0, 1);
	
	waitForButton(BUTTON_B);
	display.clear();
	display.printNumToXY(getBatteryVoltage(), 2,0);
	display.printToXY("mV", 6,0);
	delay(1000);
	cal_round();
	
	bool white_line = true;
	
	int senzor0 = 0;
	int senzor1 = 0;
	int senzor2 = 0;
	int senzor3 = 0;
	int senzor4 = 0;
	int roz_r = 0
	int roz_l = 0
	int bila = 0;
	
	while (1) 
    {
		
		int t = 0;
		
		while(1)
		{
			
			senzor0 = getSensorValue(0);
			senzor1 = getSensorValue(1);
			senzor2 = getSensorValue(2);
			senzor3 = getSensorValue(3);
			senzor4 = getSensorValue(4);
			
			bila = senzor0 + senzor1 + senzor2 + senzor3 + senzor4;
			
			int position = getLinePos(white_line = false);
			//rs232.sendNumber(position);
			//rs232.send("\n");	
			
			if(position < 1024)//zatoè vlevo (nefunguje s0 proto 2000 jinak 1000)
			{
				setMotorPower(Motor_ol);
				
			}
			else if(position < 3072) //èára pod tebou
			{
				setMotorPower(Motor_rov);	
			}
			
			else  // zatoè vpravo pokud je position < 4000
			{
				setMotorPower(Motor_or);
				
			}	
			
			if(bila < 15) // vyjel si z èáry
			{
				setMotorPower(125,125);
				delay (400);
				for(int i;i!=4 ;i++){
					setMotorPower(-125,-125);
					delay(100);
					setMotorPower(rmot,lmot);
					delay(time);
					setMotorPower(-rmot,-lmot);
					delay(time);
					setMotorPower(lmot,rmot);
					delay(time);
					setMotorPower(-lmot,-rmot);
					delay(time);
					time+100;
				}
				
				
			}
			
			
		}
    }
}

