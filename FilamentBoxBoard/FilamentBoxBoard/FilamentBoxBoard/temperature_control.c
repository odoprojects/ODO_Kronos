/*
 * temperture_control.c
 *
 * Created: 13.08.2016 18:30:42
 *  Author: Madman
 */ 
#include "temperature_control.h"
#include "hardware.h"
#include "modbus.h"
#include <util/delay.h>



TTemperatureController tempController[ANALOG_INPUTS];
TFan fan[FAN_COUNT];
TautotunePIDs autotunepidtable;
//extern char bufor[];

uint8_t autotuneIndex = 255;
short temptable_generic1[GENERIC_THERM_NUM_ENTRIES][2];



volatile uint16_t PIDTune_status = 0;

// wskaŸnik do funkcji callback dla zdarzenia UART_RX_STR_EVENT()
static void (*temperature_event_callback)();


// funkcja do rejestracji funkcji zwrotnej w zdarzeniu UART_RX_STR_EVENT()
void register_manage_temperature_event_callback(void (*callback)()) {
	temperature_event_callback = callback;
}

// Zdarzenie do obliczenia indeksu w buforze cyklicznym z ramk¹ do wykonania
void MANAGE_TEMPETATURE_EVENT() {
	if( temperature_event_callback ) {	
		(*temperature_event_callback)();
	}
}


#define NUMTEMPS_1 28
// Epcos B57560G0107F000
const short temptable_1[NUMTEMPS_1][2] PROGMEM =
{
    {0,4000},{92,2400},{105,2320},{121,2240},{140,2160},{162,2080},{189,2000},{222,1920},{261,1840},{308,1760},
    {365,1680},{434,1600},{519,1520},{621,1440},{744,1360},{891,1280},{1067,1200},{1272,1120},
    {1771,960},{2357,800},{2943,640},{3429,480},{3760,320},{3869,240},{3912,200},{3948,160},{4077,-160},{4094,-440}
};
#define NUMTEMPS_2 21
const short temptable_2[NUMTEMPS_2][2] PROGMEM =
{
    {1*4, 848*8},{54*4, 275*8}, {107*4, 228*8}, {160*4, 202*8},{213*4, 185*8}, {266*4, 171*8}, {319*4, 160*8}, {372*4, 150*8},
    {425*4, 141*8}, {478*4, 133*8},{531*4, 125*8},{584*4, 118*8},{637*4, 110*8},{690*4, 103*8},{743*4, 95*8},{796*4, 86*8},
    {849*4, 77*8},{902*4, 65*8},{955*4, 49*8},{1008*4, 17*8},{1020*4, 0*8} //safety
};

#define NUMTEMPS_3 28
const short temptable_3[NUMTEMPS_3][2] PROGMEM =
{
    {1*4,864*8},{21*4,300*8},{25*4,290*8},{29*4,280*8},{33*4,270*8},{39*4,260*8},{46*4,250*8},{54*4,240*8},{64*4,230*8},{75*4,220*8},
    {90*4,210*8},{107*4,200*8},{128*4,190*8},{154*4,180*8},{184*4,170*8},{221*4,160*8},{265*4,150*8},{316*4,140*8},{375*4,130*8},
    {441*4,120*8},{513*4,110*8},{588*4,100*8},{734*4,80*8},{856*4,60*8},{938*4,40*8},{986*4,20*8},{1008*4,0*8},{1018*4,-20*8}
};

#define NUMTEMPS_4 20
const short temptable_4[NUMTEMPS_4][2] PROGMEM =
{
    {1*4, 430*8},{54*4, 137*8},{107*4, 107*8},{160*4, 91*8},{213*4, 80*8},{266*4, 71*8},{319*4, 64*8},{372*4, 57*8},{425*4, 51*8},
    {478*4, 46*8},{531*4, 41*8},{584*4, 35*8},{637*4, 30*8},{690*4, 25*8},{743*4, 20*8},{796*4, 14*8},{849*4, 7*8},{902*4, 0*8},
    {955*4, -11*8},{1008*4, -35*8}
};
// ATC 104GT
#define NUMTEMPS_8 34
const short temptable_8[NUMTEMPS_8][2] PROGMEM =
{
    {0,8000},{69,2400},{79,2320},{92,2240},{107,2160},{125,2080},{146,2000},{172,1920},{204,1840},{222,1760},{291,1680},{350,1600},
    {422,1520},{511,1440},{621,1360},{755,1280},{918,1200},{1114,1120},{1344,1040},{1608,960},{1902,880},{2216,800},{2539,720},
    {2851,640},{3137,560},{3385,480},{3588,400},{3746,320},{3863,240},{3945,160},{4002,80},{4038,0},{4061,-80},{4075,-160}
};
#define NUMTEMPS_9 67 // 100k Honeywell 135-104LAG-J01
const short temptable_9[NUMTEMPS_9][2] PROGMEM =
{
    {1*4, 941*8},{19*4, 362*8},{37*4, 299*8}, //top rating 300C
    {55*4, 266*8},{73*4, 245*8},{91*4, 229*8},{109*4, 216*8},{127*4, 206*8},{145*4, 197*8},{163*4, 190*8},{181*4, 183*8},{199*4, 177*8},
    {217*4, 171*8},{235*4, 166*8},{253*4, 162*8},{271*4, 157*8},{289*4, 153*8},{307*4, 149*8},{325*4, 146*8},{343*4, 142*8},{361*4, 139*8},
    {379*4, 135*8},{397*4, 132*8},{415*4, 129*8},{433*4, 126*8},{451*4, 123*8},{469*4, 121*8},{487*4, 118*8},{505*4, 115*8},{523*4, 112*8},
    {541*4, 110*8},{559*4, 107*8},{577*4, 105*8},{595*4, 102*8},{613*4, 99*8},{631*4, 97*8},{649*4, 94*8},{667*4, 92*8},{685*4, 89*8},
    {703*4, 86*8},{721*4, 84*8},{739*4, 81*8},{757*4, 78*8},{775*4, 75*8},{793*4, 72*8},{811*4, 69*8},{829*4, 66*8},{847*4, 62*8},
    {865*4, 59*8},{883*4, 55*8},{901*4, 51*8},{919*4, 46*8},{937*4, 41*8},
    {955*4, 35*8},{973*4, 27*8},{991*4, 17*8},{1009*4, 1*8},{1023*4, 0}  //to allow internal 0 degrees C
};
#define NUMTEMPS_10 20 // 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
const short temptable_10[NUMTEMPS_10][2] PROGMEM =
{
    {1*4, 704*8},{54*4, 216*8},{107*4, 175*8},{160*4, 152*8},{213*4, 137*8},{266*4, 125*8},{319*4, 115*8},{372*4, 106*8},{425*4, 99*8},
    {478*4, 91*8},{531*4, 85*8},{584*4, 78*8},{637*4, 71*8},{690*4, 65*8},{743*4, 58*8},{796*4, 50*8},{849*4, 42*8},{902*4, 31*8},
    {955*4, 17*8},{1008*4, 0}
};
#define NUMTEMPS_11 31 // 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
const short temptable_11[NUMTEMPS_11][2] PROGMEM =
{
    {1*4, 936*8},{36*4, 300*8},{71*4, 246*8},{106*4, 218*8},{141*4, 199*8},{176*4, 185*8},{211*4, 173*8},{246*4, 163*8},{281*4, 155*8},
    {316*4, 147*8},{351*4, 140*8},{386*4, 134*8},{421*4, 128*8},{456*4, 122*8},{491*4, 117*8},{526*4, 112*8},{561*4, 107*8},{596*4, 102*8},
    {631*4, 97*8},{666*4, 92*8},{701*4, 87*8},{736*4, 81*8},{771*4, 76*8},{806*4, 70*8},{841*4, 63*8},{876*4, 56*8},{911*4, 48*8},
    {946*4, 38*8},{981*4, 23*8},{1005*4, 5*8},{1016*4, 0}
};
#define NUMTEMPS_12 31 // 100k RS thermistor 198-961 (4.7k pullup)
const short temptable_12[NUMTEMPS_12][2] PROGMEM =
{
    {1*4, 929*8},{36*4, 299*8},{71*4, 246*8},{106*4, 217*8},{141*4, 198*8},{176*4, 184*8},{211*4, 173*8},{246*4, 163*8},{281*4, 154*8},{316*4, 147*8},
    {351*4, 140*8},{386*4, 134*8},{421*4, 128*8},{456*4, 122*8},{491*4, 117*8},{526*4, 112*8},{561*4, 107*8},{596*4, 102*8},{631*4, 97*8},{666*4, 91*8},
    {701*4, 86*8},{736*4, 81*8},{771*4, 76*8},{806*4, 70*8},{841*4, 63*8},{876*4, 56*8},{911*4, 48*8},{946*4, 38*8},{981*4, 23*8},{1005*4, 5*8},{1016*4, 0*8}
};
#if CPU_ARCH == ARCH_AVR
#define NUMTEMPS_13 19
const short temptable_13[NUMTEMPS_13][2] PROGMEM =
{
    {0,0},{908,8},{942,10*8},{982,20*8},{1015,8*30},{1048,8*40},{1080,8*50},{1113,8*60},{1146,8*70},{1178,8*80},{1211,8*90},{1276,8*110},{1318,8*120}
    ,{1670,8*230},{2455,8*500},{3445,8*900},{3666,8*1000},{3871,8*1100},{4095,8*2000}
};
#else
#define NUMTEMPS_13 9
const short temptable_13[NUMTEMPS_13][2] PROGMEM =
{
	{0,0},{1365,8},{1427,10*8},{1489,20*8},{2532,8*230},{2842,8*300},{3301,8*400},{3723,8*500},{4095,8*600}
};
#endif
#define NUMTEMPS_14 46
const short temptable_14[NUMTEMPS_14][2] PROGMEM = {
	{1*4,8*938}, {31*4,8*314}, {41*4,8*290}, {51*4,8*272}, {61*4,8*258}, {71*4,8*247}, {81*4,8*237}, {91*4,8*229}, {101*4,8*221}, {111*4,8*215}, {121*4,8*209},
	{131*4,8*204}, {141*4,8*199}, {151*4,8*195}, {161*4,8*190}, {171*4,8*187}, {181*4,8*183}, {191*4,8*179}, {201*4,8*176}, {221*4,8*170}, {241*4,8*165}, 
	{261*4,8*160}, {281*4,8*155}, {301*4,8*150}, {331*4,8*144}, {361*4,8*139}, {391*4,8*133}, {421*4,8*128}, {451*4,8*123}, {491*4,8*117}, {531*4,8*111}, 
	{571*4,8*105}, {611*4,8*100}, {681*4,8*90}, {711*4,8*85}, {811*4,8*69}, {831*4,8*65}, {881*4,8*55}, 
	{901*4,8*51},  {941*4,8*39}, {971*4,8*28}, {981*4,8*23}, {991*4,8*17}, {1001*4,8*9}, {1021*4,8*-27},{1023*4,8*-200}
};
	
#if NUM_TEMPS_USERTHERMISTOR0 > 0
const short temptable_5[NUM_TEMPS_USERTHERMISTOR0][2] PROGMEM = USER_THERMISTORTABLE0 ;
#endif
#if NUM_TEMPS_USERTHERMISTOR1 > 0
const short temptable_6[NUM_TEMPS_USERTHERMISTOR1][2] PROGMEM = USER_THERMISTORTABLE1 ;
#endif
#if NUM_TEMPS_USERTHERMISTOR2 > 0
const short temptable_7[NUM_TEMPS_USERTHERMISTOR2][2] PROGMEM = USER_THERMISTORTABLE2 ;
#endif
const short * const temptables[14] PROGMEM = {(short int *)&temptable_1[0][0],(short int *)&temptable_2[0][0],(short int *)&temptable_3[0][0],(short int *)&temptable_4[0][0]
#if NUM_TEMPS_USERTHERMISTOR0 > 0
        ,(short int *)&temptable_5[0][0]
#else
        ,0
#endif
#if NUM_TEMPS_USERTHERMISTOR1 > 0
        ,(short int *)&temptable_6[0][0]
#else
        ,0
#endif
#if NUM_TEMPS_USERTHERMISTOR2 > 0
        ,(short int *)&temptable_7[0][0]
#else
        ,0
#endif
        ,(short int *)&temptable_8[0][0]
        ,(short int *)&temptable_9[0][0]
        ,(short int *)&temptable_10[0][0]
        ,(short int *)&temptable_11[0][0]
        ,(short int *)&temptable_12[0][0]
        ,(short int *)&temptable_13[0][0]
        ,(short int *)&temptable_14[0][0]
                                             };
const uint8_t temptables_num[14] PROGMEM = {NUMTEMPS_1,NUMTEMPS_2,NUMTEMPS_3,NUMTEMPS_4,NUM_TEMPS_USERTHERMISTOR0,NUM_TEMPS_USERTHERMISTOR1,NUM_TEMPS_USERTHERMISTOR2,NUMTEMPS_8,
                                 NUMTEMPS_9,NUMTEMPS_10,NUMTEMPS_11,NUMTEMPS_12,NUMTEMPS_13,NUMTEMPS_14
};



float constrain(float y, float a, float b){
	if(y < a) {
		return a;
	}
	else if(b < y) {
		return b;
	}
	else
	return y;
}

void createGenericTable(short table[GENERIC_THERM_NUM_ENTRIES][2],int minTemp, int maxTemp, float beta, float r0, float t0, float r1, float r2)
{
	t0+=273.15f;
	float rs,vs;
	if(r1==0)
	{
		rs = r2;
		vs = GENERIC_THERM_VREF;
	}
	else
	{
		vs =(float)(GENERIC_THERM_VREF*r1)/(r1+r2);
		rs = (r2*r1)/(r1+r2);
	}
	float k = r0*exp(-beta/t0);
	float delta = (maxTemp-minTemp)/(GENERIC_THERM_NUM_ENTRIES-1.0f);
	for(uint8_t i=0; i<GENERIC_THERM_NUM_ENTRIES; i++)
	{
		float t = maxTemp-i*delta;
		float r = exp(beta/(t+272.65))*k;
		float v = 4092*r*vs/((rs+r)*GENERIC_THERM_VREF);
		int adc = (int)(v);
		t *= 8;
		if(adc>4092) adc=4092;
		table[i][0] = (adc>>(ANALOG_REDUCE_BITS));
		table[i][1] = (int)t;
	}
}

void disableAllHeater()
{
	
	for(uint8_t i = 0; i <= ANALOG_INPUTS; i++)
	{
		tempController[i].targetTemperatureC = 0;
		tempController[i].targetTemperature = 0;
		pwm_pos[tempController[i].pwmIndex] = 0;
	}

	autotuneIndex = 255;
}


int8_t autotunePID(struct TemperatureController *temperatureCont,  float temp,  uint8_t controllerId, int maxCycles, uint8_t storeValues)
{

	PIDTune_status |= PIDTUNE_AUTOTUNE_START;
	modbus_puts_s(PSTR(" Pid Tuning in progress."));
	
	float currentTemp;
	int cycles=0;
	uint8_t heating = 1;

	uint32_t temp_millis = millis;
	uint32_t t1=millis;
	uint32_t t2=millis;
	int32_t t_high = 0;
	int32_t t_low;

	int32_t bias=temperatureCont->pidMax>>1;
	int32_t d = temperatureCont->pidMax>>1;
	float Ku, Tu;
	float Kp=0, Ki=0, Kd=0;
	float maxTemp=20, minTemp=20;
	
    if(maxCycles < 5)
		maxCycles = 5;
    if(maxCycles > 20)
		maxCycles = 20;
	disableAllHeater();
	
		
	autotuneIndex = controllerId;
	if (controllerId == NUM_EXTRUDER)
	{
		pwm_pos[BED0] = tempController[BED0].pidMax;
		pwm_pos[BED1] = tempController[BED1].pidMax;
		pwm_pos[BED2] = tempController[BED2].pidMax;
		pwm_pos[BED3] = tempController[BED3].pidMax;
		
	}else
	{
		pwm_pos[temperatureCont->pwmIndex] =  temperatureCont->pidMax;
	}

	if(controllerId<NUM_EXTRUDER)
	{
		fan[controllerId].coolerPWM = fan[controllerId].coolerSpeed;
		fan[0].coolerPWM = fan[0].coolerSpeed;
	}
	for(;;)
	{
		
		updateCurrentTemperature(temperatureCont);
		currentTemp = temperatureCont->currentTemperatureC;
		UART_RX_STR_EVENT(bufor);
		//MANAGE_TEMPETATURE_EVENT();
		unsigned long time = millis;
		

		maxTemp=((maxTemp>currentTemp )? maxTemp : currentTemp);
		minTemp=((minTemp<currentTemp )? minTemp : currentTemp);
		
		if(heating == 1 && currentTemp > temp)   
		{
			if(time - t2 > (controllerId < NUM_EXTRUDER ? 2500 : 1500))
			{
				heating=0;
				if (controllerId == NUM_EXTRUDER)
				{
					pwm_pos[BED0] = (bias - d);
					pwm_pos[BED1] = (bias - d);
					pwm_pos[BED2] = (bias - d);
					pwm_pos[BED3] = (bias - d);
					
				}else
				{
					pwm_pos[temperatureCont->pwmIndex] = (bias - d);
				}
				t1=time;
				t_high=t1 - t2;
				maxTemp=temp;
			}
		}
		if(heating == 0 && currentTemp < temp)
		{
			if(time - t1 > (controllerId < NUM_EXTRUDER ? 5000 : 3000))
			{
				heating=1;
				t2=time;
				t_low=t2 - t1; // half wave length
				if(cycles > 0)
				{
					bias += (d*(t_high - t_low))/(t_low + t_high);
					bias = (int32_t)constrain(bias, 20 ,temperatureCont->pidMax-20);
					if(bias > temperatureCont->pidMax/2) d = temperatureCont->pidMax - 1 - bias;
					else d = bias;

					//modbus_puts_var_int(PSTR(" bias: "),bias*100);
					//modbus_puts_var_int(PSTR(" d: "),d*100);
					//modbus_puts_var_int(PSTR(" minTemp: "),minTemp*100);
					//modbus_puts_var_int(PSTR(" maxTemp: "),maxTemp*100);

// 					modbus_puts_var_float(PSTR(" bias: "),bias,2);
// 					modbus_puts_var_float(PSTR(" d: "),d,2);
// 					modbus_puts_var_float(PSTR(" minTemp: "),minTemp,2);
// 					modbus_puts_var_float(PSTR(" maxTemp: "),maxTemp,2);

					if(cycles > 2)
					{
						Ku = (4.0*d)/(3.14159*(maxTemp-minTemp));
						Tu = ((float)(t_low + t_high)/1000.0);

						//modbus_puts_var_int(PSTR(" Ku: "),Ku*100);
						//modbus_puts_var_int(PSTR(" Tu: "),Tu*100);
						
// 						modbus_puts_var_float(PSTR(" Ku: "),Ku,2);
// 						modbus_puts_var_float(PSTR(" Tu: "),Tu,2);

						Kp = 0.6*Ku;
						Ki = 2*Kp/Tu;
						Kd = Kp*Tu*0.125;
						//modbus_puts_s(PSTR(" Classic PID"));
						
						modbus_puts_var_int(PSTR(" Kp: "),Kp*100);
						modbus_puts_var_int(PSTR(" Ki: "),Ki*100);
						modbus_puts_var_int(PSTR(" Kd: "),Kd*100);
// 						modbus_puts_var_float(PSTR(" Kp: "),Kp,2);
// 						modbus_puts_var_float(PSTR(" Ki: "),Ki,2);
// 						modbus_puts_var_float(PSTR(" Kd: "),Kd,2);
					}
				}
				if (controllerId == NUM_EXTRUDER)
				{
					pwm_pos[BED0] = (bias + d);
					pwm_pos[BED1] = (bias + d);
					pwm_pos[BED2] = (bias + d);
					pwm_pos[BED3] = (bias + d);
					
				}else
				{
					pwm_pos[temperatureCont->pwmIndex] = (bias + d);
				}
				cycles++;
				modbus_puts_var_int(PSTR(" cycles: "),cycles);
				minTemp=temp;
			}
		}
		if(currentTemp > (temp + 20))
		{
			PIDTune_status |= PIDTUNE_TAPIDFAILEDHIGH;
			modbus_puts_s(PSTR(" PID Autotune failed! Temperature too high"));			
			disableAllHeater();
			return 1;
		}
		if(time - temp_millis > 10000)
		{
			temp_millis = time;
			modbus_puts_s(PSTR(" Pid Tuning in progress."));
		}
		if(((time - t1) + (time - t2)) > (10L*60L*1000L*2L))   // 20 Minutes   (10L*60L*1000L*2L)
		{
			PIDTune_status |= PIDTUNE_TAPIDFAILEDTIMEOUT;
			modbus_puts_s(PSTR(" PID Autotune failed! timeout"));			
			disableAllHeater();
			return 1;
		}
		if(cycles > maxCycles)
		{
			PIDTune_status |= PIDTUNE_TAPIDFINISHED;
			PIDTune_status &= ~(PIDTUNE_AUTOTUNE_START);
			modbus_puts_s(PSTR(" PID Autotune finished !"));	
			disableAllHeater();
			//if(storeValues)
			//{
				modbus_puts_s(PSTR("Eeprom PID saved"));
				temperatureCont->pidDGain = Kp;
				temperatureCont->pidIGain = Ki;
				temperatureCont->pidDGain = Kd;
				temperatureCont->heatManager = HTR_PID;
				storeDataIntoEEPROM();
			//}
			return 1;
		}
	}
}

void initTempControler(struct TemperatureController *temperatureCont, int index){
	
	temperatureCont->pwmIndex = index;
	temperatureCont->analogIndex = index;
	
}


//const uint8_t temptables_num[12] PROGMEM = {NUMTEMPS_1,NUMTEMPS_2,NUMTEMPS_3,NUMTEMPS_4,NUM_TEMPS_USERTHERMISTOR0,NUM_TEMPS_USERTHERMISTOR1,NUM_TEMPS_USERTHERMISTOR2,NUMTEMPS_8,
//	NUMTEMPS_9,NUMTEMPS_10,NUMTEMPS_11,NUMTEMPS_12
//};

void init_temperature_control(){
	createGenericTable(temptable_generic1,GENERIC_THERM1_MIN_TEMP, GENERIC_THERM1_MAX_TEMP, GENERIC_THERM1_BETA, GENERIC_THERM1_R0, GENERIC_THERM1_T0, GENERIC_THERM1_R1, GENERIC_THERM1_R2);
	for(uint8_t tmpContr = 0; tmpContr<ANALOG_INPUTS; tmpContr++){
		initTempControler(&tempController[tmpContr],tmpContr);
	}	
}



void setTargetTemperature(struct TemperatureController *temperatureCont, uint16_t tar)
{
	temperatureCont->targetTemperatureC = (float)tar/100.0f;
	int temp = TEMP_FLOAT_TO_INT(temperatureCont->targetTemperatureC);
	uint8_t type = temperatureCont->sensorType;
	switch(temperatureCont->sensorType)
	{
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
		case 204:
	    {
		    if (type == 204)
		    {
			    type = 8;
		    }
		    type--;
			uint8_t num = pgm_read_uint8_t(&temptables_num[type])<<1;
			uint8_t i=2;
			const short *temptable = (const short *)pgm_read_word(&temptables[type]); //pgm_read_word(&temptables[type]);
			short oldraw = pgm_read_word(&temptable[0]);
			short oldtemp = pgm_read_word(&temptable[1]);
			short newraw,newtemp;
			while(i<num)
			{
				newraw = pgm_read_word(&temptable[i++]);
				newtemp = pgm_read_word(&temptable[i++]);
				if (newtemp < temp)
				{
					temperatureCont->targetTemperature = (1023<<(2-ANALOG_REDUCE_BITS))- oldraw + (int32_t)(oldtemp-temp)*(int32_t)(oldraw-newraw)/(oldtemp-newtemp);
					return 1;
				}
				oldtemp = newtemp;
				oldraw = newraw;
			}
			// Overflow: Set to last value in the table
			temperatureCont->targetTemperature = (1023<<(2-ANALOG_REDUCE_BITS))-newraw;
			break;
		}
		case 50: // user defined PTC thermistor
		case 51:
		case 52:
		{
			type-=46;
			uint8_t num = pgm_read_uint8_t(&temptables_num[type])<<1;
			uint8_t i=2;
			const short *temptable = (const short *)pgm_read_word(&temptables[type]); //pgm_read_word(&temptables[type]);
			short oldraw = pgm_read_word(&temptable[0]);
			short oldtemp = pgm_read_word(&temptable[1]);
			short newraw,newtemp;
			while(i<num)
			{
				newraw = pgm_read_word(&temptable[i++]);
				newtemp = pgm_read_word(&temptable[i++]);
				if (newtemp > temp)
				{
					temperatureCont->targetTemperature = oldraw + (int32_t)(oldtemp-temp)*(int32_t)(oldraw-newraw)/(oldtemp-newtemp);
					return 1;
				}
				oldtemp = newtemp;
				oldraw = newraw;
			}
			// Overflow: Set to last value in the table
			temperatureCont->targetTemperature = newraw;
			break;
		}
		case 60: // HEATER_USES_AD8495 (Delivers 5mV/degC)
		temperatureCont->targetTemperature = (int)((int32_t)temp * (1024<<(2-ANALOG_REDUCE_BITS))/ 1000);
		break;
		case 100: // HEATER_USES_AD595
		temperatureCont->targetTemperature = (int)((int32_t)temp * (1024<<(2-ANALOG_REDUCE_BITS))/ 500);
		break;
		#ifdef SUPPORT_MAX6675
		case 101:  // defined HEATER_USES_MAX6675
		temperatureCont->targetTemperature = temp * 4;
		break;
		#endif
		#ifdef SUPPORT_MAX31855
		case 102:  // defined HEATER_USES_MAX31855
		temperatureCont->targetTemperature = temp * 4;
		break;
		#endif
		case 97:
		case 98:
		case 99:
		case 203:
		{
			if (type == 203)
			{
				type = 97;
			}
			uint8_t i=2;
			const short *temptable;
			#ifdef USE_GENERIC_THERMISTORTABLE_1
			if(type == 97)
			temptable = (const short *)temptable_generic1;
			#endif
			short oldraw = temptable[0];
			short oldtemp = temptable[1];
			short newraw,newtemp;
			while(i<GENERIC_THERM_NUM_ENTRIES*2)
			{
				newraw = temptable[i++];
				newtemp = temptable[i++];
				if (newtemp < temp)
				{
					temperatureCont->targetTemperature = (1023<<(2-ANALOG_REDUCE_BITS))- oldraw + (int32_t)(oldtemp-temp)*(int32_t)(oldraw-newraw)/(oldtemp-newtemp);
					return 1;
				}
				oldtemp = newtemp;
				oldraw = newraw;
			}
			// Overflow: Set to last value in the table
			temperatureCont->targetTemperature = (1023<<(2-ANALOG_REDUCE_BITS))-newraw;
			break;
		}
	}
}

int8_t updateCurrentTemperature(struct TemperatureController *temperatureCont){

	uint8_t type = temperatureCont->sensorType;
    switch(type)
    {
		case 0:
			temperatureCont->currentTemperature = 25;
			break;
	    #if ANALOG_INPUTS>0
	    case 1:
	    case 2:
	    case 3:
	    case 4:
	    case 5:
	    case 6:
	    case 7:
	    case 8:
	    case 9:
	    case 10:
	    case 11:
	    case 12:
	    case 97:
	    case 98:
	    case 99:
		case 203:
		case 204:
	    temperatureCont->currentTemperature = (1023<<(2-ANALOG_REDUCE_BITS))-(osAnalogInputValues[temperatureCont->analogIndex]>>(ANALOG_REDUCE_BITS)); // Convert to 10 bit result
	    break;
	    case 50: // User defined PTC table
	    case 51:
	    case 52:
	    case 60: // HEATER_USES_AD8495 (Delivers 5mV/degC)
	    case 100: // AD595
	    temperatureCont->currentTemperature = (osAnalogInputValues[temperatureCont->analogIndex]>>(ANALOG_REDUCE_BITS));
	    break;
	    #endif
	    default:
	    temperatureCont->currentTemperature = 4095; // unknown method, return high value to switch heater off for safety
    }
    int currentTemperature = temperatureCont->currentTemperature;
    switch(type)
    {
		case 0:
			temperatureCont->currentTemperatureC = 25;
			break;		
	    case 1:
	    case 2:
	    case 3:
	    case 4:
	    case 5:
	    case 6:
	    case 7:
	    case 8:
	    case 9:
	    case 10:
	    case 11:
	    case 12:
		case 14:
		case 204:
	    {	
			if (type == 204)
			{
				type = 8;
			}				
		    type--;
		    uint8_t num = pgm_read_uint8_t(&temptables_num[type])<<1;
		    uint8_t i=2;
		    const int16_t *temptable = (const int16_t *)pgm_read_word(&temptables[type]); //pgm_read_word_near(&temptables[type]);
			//const short *temptable = (const short *)pgm_read_word(&temptables[type]);
		    int16_t oldraw = pgm_read_word(&temptable[0]);
		    int16_t oldtemp = pgm_read_word(&temptable[1]);
			int16_t newraw,newtemp = 0;
		    currentTemperature = (1023<<(2-ANALOG_REDUCE_BITS))-currentTemperature;
		    while(i<num)
		    {
			    newraw = pgm_read_word(&temptable[i++]);
			    newtemp = pgm_read_word(&temptable[i++]);
			    if (newraw > currentTemperature)
			    {
				    temperatureCont->currentTemperatureC = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature-oldraw)*(float)(newtemp-oldtemp)/(newraw-oldraw));
					return 1;
			    }
			    oldtemp = newtemp;
			    oldraw = newraw;
		    }
		    temperatureCont->currentTemperatureC = TEMP_INT_TO_FLOAT(newtemp);
		}
	    break;
		case 13:
	    case 50: // User defined PTC thermistor
	    case 51:
	    case 52:
	    {
		    type-=46;
		    uint8_t num = pgm_read_uint8_t(&temptables_num[type])<<1;
		    uint8_t i=2;
		    const short *temptable = (const short *)pgm_read_word(&temptables[type]); //pgm_read_word_near(&temptables[type]);
		    short oldraw = pgm_read_word(&temptable[0]);
		    short oldtemp = pgm_read_word(&temptable[1]);
		    short newraw,newtemp;
		    while(i<num)
		    {
			    newraw = pgm_read_word(&temptable[i++]);
			    newtemp = pgm_read_word(&temptable[i++]);
			    if (newraw > currentTemperature)
			    {
					
				    temperatureCont->currentTemperatureC = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature-oldraw)*(float)(newtemp-oldtemp)/(newraw-oldraw));

				    return 1;
			    }
			    oldtemp = newtemp;
			    oldraw = newraw;
		    }
		    temperatureCont->currentTemperatureC = TEMP_INT_TO_FLOAT(newtemp);
		    break;
		}
		case 97:
		case 98:
		case 99:
		case 203:
		{
			if (type == 203)
			{
				type = 97;
			}
			uint8_t i=2;
			const short *temptable;
			#ifdef USE_GENERIC_THERMISTORTABLE_1
			if(type == 97)
				temptable = (const short *)temptable_generic1;
			#endif

			short oldraw = temptable[0];
			short oldtemp = temptable[1];
			short newraw,newtemp;
			currentTemperature = (1023<<(2-ANALOG_REDUCE_BITS))-currentTemperature;

			while(i<GENERIC_THERM_NUM_ENTRIES*2)
			{
				newraw = temptable[i++];
				newtemp = temptable[i++];
				if (newraw > currentTemperature)
				{
					temperatureCont->currentTemperatureC = TEMP_INT_TO_FLOAT(oldtemp + (float)(currentTemperature-oldraw)*(float)(newtemp-oldtemp)/(newraw-oldraw));
					float t =  temperatureCont->currentTemperatureC;
					return 1;
				}
				oldtemp = newtemp;
				oldraw = newraw;
			}
			temperatureCont->currentTemperatureC = TEMP_INT_TO_FLOAT(newtemp);
			break;
		}
	}
}

void manageTemperatures(){
	static uint8_t TempErrors = 0;
	for(uint8_t controller = 0; controller<ANALOG_INPUTS; controller++)
	{
		updateCurrentTemperature(&tempController[controller]);
		
		if(controller<NUM_EXTRUDER)
		{
			if(tempController[controller].currentTemperatureC<EXTRUDER_FAN_COOL_TEMP && tempController[controller].targetTemperatureC<EXTRUDER_FAN_COOL_TEMP)
			fan[controller].coolerPWM = 0;
			else
			fan[controller].coolerPWM = fan[controller].coolerSpeed;
		}
		if((tempController[controller].currentTemperatureC < MIN_DEFECT_TEMPERATURE || tempController[controller].currentTemperatureC > MAX_DEFECT_TEMPERATURE))   // no temp sensor or short in sensor, disable heater
		{
			TempErrors++;
#if defined (DEBUG_PRINT)
			uart_puts("error1");
			uart_putc('\n');
#endif
		}
		if(TempErrors) continue;
		uint8_t on = tempController[controller].currentTemperature>=tempController[controller].targetTemperature ? 0 : 1;

#ifdef TEMP_PID
		tempController[controller].tempArray[tempController[controller].tempPointer++] = tempController[controller].currentTemperatureC;
		tempController[controller].tempPointer &= 3;
		if(tempController[controller].heatManager == 1)
		{
			uint8_t output;
			float error = tempController[controller].targetTemperatureC - tempController[controller].currentTemperatureC;
			if(tempController[controller].targetTemperatureC<20.0f) output = 0; // off is off, even if damping term wants a heat peak!
			else if(error>PID_CONTROL_RANGE)
			output = tempController[controller].pidMax;
			else if(error<-PID_CONTROL_RANGE)
			output = 0;
			else
			{
				float pidTerm = tempController[controller].pidPGain * error;	
				tempController[controller].tempIState = constrain(tempController[controller].tempIState+error,tempController[controller].tempIStateLimitMin,tempController[controller].tempIStateLimitMax);
				pidTerm += tempController[controller].pidIGain * tempController[controller].tempIState*0.1;
				float dgain = tempController[controller].pidDGain * (tempController[controller].tempArray[tempController[controller].tempPointer]-tempController[controller].currentTemperatureC)*3.333f;
				pidTerm += dgain;			
				#if SCALE_PID_TO_MAX==1
				pidTerm = (pidTerm  * tempController[controller].pidMax)*0.0039062;
				#endif
				output = (uint8_t)constrain((int)pidTerm, 0, tempController[controller].pidMax);
			}
			pwm_pos[tempController[controller].pwmIndex] = output;
		}
		else if(tempController[controller].heatManager == 3)     // deat-time control
		{
			uint8_t output;
			float error = tempController[controller].targetTemperatureC - tempController[controller].currentTemperatureC;
			if(tempController[controller].targetTemperatureC<20.0f)
			output = 0; // off is off, even if damping term wants a heat peak!
			else if(error>PID_CONTROL_RANGE)
			output = tempController[controller].pidMax;
			else if(error < -PID_CONTROL_RANGE)
			output = 0;
			else
			{
				float raising = 3.333 * (tempController[controller].currentTemperatureC - tempController[controller].tempArray[tempController[controller].tempPointer]); // raising dT/dt, 3.33 = reciproke of time interval (300 ms)
				tempController[controller].tempIState = 0.25 * (3.0 * tempController[controller].tempIState + raising); // damp raising
				output = (tempController[controller].currentTemperatureC + tempController[controller].tempIState * tempController[controller].pidPGain > tempController[controller].targetTemperatureC ? 0 : tempController[controller].pidDriveMax);
			}
			pwm_pos[tempController[controller].pwmIndex] = output;
		}
		else
		#endif
		if(tempController[controller].heatManager == 2)    // Bang-bang with reduced change frequency to save relais life
		{
			uint32_t time = millis;
			if (time - tempController[controller].lastTemperatureUpdate > HEATED_BED_SET_INTERVAL)
			{
				pwm_pos[tempController[controller].pwmIndex] = (on ? 255 : 0);
				tempController[controller].lastTemperatureUpdate = time;
			}
		}
		else     // Fast Bang-Bang fallback
		{
			pwm_pos[tempController[controller].pwmIndex] = (on ? 255 : 0);
		}
	}
}