 /*
 * File:   SPWM.c
 * Author: Devanshu
 *
 * Created on August 8, 2017, 12:42 PM
 */
//
// DSPIC30F4011 Configuration Bit Settings
// 'C' source line Configuration statements
// FOSC

//**********************************************************************************************************************************************************
//                                                                 CONFIGURATIONS
//**********************************************************************************************************************************************************
#pragma config FPR = XT_PLL16           // Primary Oscillator Mode (XT w/PLL 16x)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_PWMPIN      // PWM Output Pin Reset (Control with HPOL/LPOL bits)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
//// Use project enums instead of #define for ON and OFF.
//*******************************************************************************************************************************************************
//                                                      HEADER FILES
//******************************************************************************************************************************************************
#include <xc.h>
#include <dsp.h>
#include <libpic30.h>
#include <p30F4011.h>
#include<stdio.h>
#include<math.h>
//*******************************************************************************************************************************************************
////                                                    MACROS
//*******************************************************************************************************************************************************
typedef signed int SFRAC16;

//#define PIDRefrenceVal 410
#define _trapISR __attribute__((interrupt,no_auto_psv))

#define BattSetVoltage 96
#define KVA_Val 2
#define LcdParametersDisplayNo 7

#define IpVoltMultFactor 0.42060        //Claibrated
#define IpCurrentMultFactor 0.013314
#define OpVoltMultFactor 0.454866       //Calibrated
#define OpCurrentMultFactor 0.013314
#define BattVoltMultFactor 0.50208868
#define BattChargCurrentMultFactor 0.009696
#define BattDisChargCurrentMultFactor 0.009696


#define BlankFirstRow() Lcd_Out(1, 0,"                ")
#define BlankSecondRow() Lcd_Out(2, 0, "              ")
#define Transtech "TRANSTECH SYSTEM";

#define Fault "      FAULT     "
#define HighOPVoltage "HIGH O/P VOLTAGE"
#define LowOPVoltage "LOW O/P VOLTAGE "
#define HighOpCurrent "HIGH O/P CURRENT"

#define BattVoltage "BATTERY VOLTAGE:"
#define BattLow "   BATTERY LOW  "
#define BattHigh "  BATTERY HIGH  "
#define BattChargCurrent "BAT CHAR CURRENT:"
#define BattDisChargCurrent "BATT DISCHAR CURRENT:"

#define MainsVoltageLow "MAINS VOLT LOW"
#define MainsVoltageHigh "MAINS VOLT HIGH"
#define MainsCurrentHigh "MAINS CURRENT HIGH"
#define MainsCurrent "MAINS CURRENT: "
#define MainsVoltage "MAINS VOLTAGE: "

#define OPVolatge "OUTPUT VOLTAGE: "
#define OPCurrent "OUTPUT CURRENT: " 

#define OPFrequency "OUTPUT FREQUENCY:"

#define OpPower "  OUTPUT POWER "
#define IpPower "   INPUT POWER  "
#define PercentLoad "   LOAD PERCENT   "

#define ADC_ON() ADCON1bits.ADON=1
#define ADC_OFF() ADCON1bits.ADON=0
#define ADC_StartSampling() ADCON1bits.SAMP=1

#define EnablePwmInterrupt() IEC2bits.PWMIE=1
#define EnableTimer2Interrupt() _T2IE=1
#define EnableTimer3Interrupt() _T3IE=1
#define EnableTimer5Interrupt() _T5IE=1
#define EnableTimer4Interrupt() _T4IE=1;

#define Lcd_delay 2

#define T2_Period 29490    //prescalar 0, 1 ms delay
#define T3_Period 65535    //Prescalar 256, 0.5688s delay
#define T4_Period 14745    //prescalar 0, 0.5 ms delay
#define T5_Period 65535     //Prescalar 256, 0.5688 delay



#define TMR2_StartTimer() T2CONbits.TON=1
#define TMR3_StartTimer() T3CONbits.TON=1
#define TMR4_StartTimer() T4CONbits.TON=1
#define TMR5_StartTimer() T5CONbits.TON=1

#define TMR2_StopTimer() T2CONbits.TON=0
#define TMR3_StopTimer() T3CONbits.TON=0
#define TMR4_StopTimer() T4CONbits.TON=0
#define TMR5_StopTimer() T5CONbits.TON=0

#define PWM1_StopPwm() PTCONbits.PTEN=0
#define PWM1_StartPwm() PTCONbits.PTEN=1

#define ANA0_PWM_Control 0
#define ANA1_Op_HighLow 1
#define ANA2_Op_Feedback 2
#define ANA3_Op_Overcurrent 3 
#define ANA4_Ip_Voltage 4
#define ANA5_Ip_Current 5
#define ANA6_Batt_Voltage 6
#define ANA7_Batt_Charging_Current 7 
#define ANA8_Batt_Discharging_Current 8



#define LCD_D7 LATFbits.LATF3
#define LCD_D6 LATFbits.LATF2
#define LCD_D5 LATFbits.LATF1
#define LCD_D4 LATFbits.LATF0
#define LCD_EN LATEbits.LATE4
#define LCD_RS LATEbits.LATE5
//RS not needed since lcd used in 



#define Op_High_Indicator LATFbits.LATF4
#define Op_Low_Indicator LATFbits.LATF5
#define Op_Overcurrent_Indicator LATFbits.LATF6
#define Overall_Fault_Indicator LATDbits.LATD2


#define AutoScroll PORTCbits.RC14
#define Mains_High_Indicator LATEbits.LATE8
#define Main_Low_Indicator  LATDbits.LATD1
//LATE0- Sq_wave O/p


#define Batt_Low_Indicator LATDbits.LATD0
//#define Batt_High_Indicator 
#define BattBuzzerIndicator LATEbits.LATE2

#define Charger_Card LATDbits.LATD3

#define SFloat_To_SFrac16(Float_Value)	\
        ((Float_Value < 0.0) ? (SFRAC16)(32768 * (Float_Value) - 0.5) \
        : (SFRAC16)(32767 * (Float_Value) + 0.5))
//**********************************************************************************************************************************************************************
//                                                GLOBAL VARIABLES
//*****************************************************************************************************************************************************8


volatile float BattAvgChargingAnalogCurrent,BattAvgDisChargingAnalogCurrent,BattAvgAnalogVolatge=0,IpAvgAnalogCurrent=0
,OpAvgAnalogVoltage=0,OpAvgAnalogCurrent=0,IpAvgAnalogVolatge=0;
volatile static unsigned int i=0,k=0,temp=0,prevtemp=0,PIDRefrenceVal=545,PIDSetpoint=0,SoftstartCount=0,DelayBetLcdParm=0,
        OpAnalogVoltage[25],OpAnalogCurrent[25],
        IpAnalogVolatge[25],IpAnalogCurrent[25],
        BattChargingAnalogCurrent[25],BattDisChargingAnalogCurrent[25],BattAnalogVolatge[25],BattTripVolatge=0,BattBuzzerVolatge=0,BattHighVolatge=0,OverCurrentVolatge=0;
volatile static long unsigned int DutyCycle=0,result=0;
const unsigned int LookUpTable[128]={400, 501, 601, 701, 801, 901, 1000, 1099, 1198, 1296, 1393, 1490, 1586, 1681, 
1775, 1869, 1961, 2052, 2143, 2232, 2320, 2407, 2493, 2577, 2660, 2742, 2822, 2901, 2978, 3053, 3127, 3199, 3269, 
3337, 3404, 3469, 3532, 3593, 3652, 3709, 3763, 3816, 3867, 3915, 3962, 4006, 4048, 4087, 4125, 4160, 4193, 4223, 
4251, 4277, 4300, 4321, 4339, 4355, 4369, 4380, 4388, 4395, 4398, 4400, 4398, 4395, 4388, 4380, 4369, 4355, 4339,
4321, 4300, 4277, 4251, 4223, 4193, 4160, 4125, 4087, 4048, 4006, 3962, 3915, 3867, 3816, 3763, 3709, 3652, 3593,
3532, 3469, 3404, 3337, 3269, 3199, 3127, 3053, 2978, 2901, 2822, 2742, 2660, 2577, 2493, 2407, 2320, 2232, 2143,
2052, 1961, 1869, 1775, 1681, 1586, 1490, 1393, 1296, 1198, 1099, 1000, 901, 801, 701, 601, 501, 400, 300};

volatile char FlagOHigh=0,FlagOLow=0,FlagOOverCurrent=0,FlagBattLow=0,FlagFaultIndicator=0,FlagIHigh=0,FlagILow=0,
        FlagIOverCurrent=0,flag=0,softstart=1,
        OpVoltReadyForAvg=0,OpCurrentReadyForAvg=0,
        IpVoltReadyForAvg=0,IpCurrentReadyForAvg=0,
        BattVoltReadyForAvg=0,BattChargCurrReadyForAvg=0,BattDisChargCurrReadyForAvg=0,
        count=0,ShortPressCount=0,FaultDisplayFlag=0,FlagSecondtime=0,BuzzerFlag=0;

//int _EEDATA(32) fooArrayInDataEE[] = {0,1,2,3,4,5,6,7,8,9,0xA,0xB,0xC,0xD,0xE,0xF};

/*Declare variables to be stored in RAM*/
//int fooArray1inRAM[] = {0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF, 0xABCD, 0xBCDE,
  //                     0xCDEF, 0xDEFA, 0x0000, 0x1111, 0x2222, 0x3333, 0x4444, 0x5555};

//int fooArray2inRAM[16];
 
/*
Variable Declaration required for each PID controller in your application
*/
/* Declare a PID Data Structure named, fooPID */
tPID fooPID;
/* The fooPID data structure contains a pointer to derived coefficients in X-space and */
/* pointer to controler state (history) samples in Y-space. So declare variables for the */
/* derived coefficients and the controller history samples */
fractional abcCoefficient[3] __attribute__ ((section (".xbss, bss, xmemory")));
fractional controlHistory[3] __attribute__ ((section (".ybss, bss, ymemory")));
/* The abcCoefficients referenced by the fooPID data structure */
/* are derived from the gain coefficients, Kp, Ki and Kd */
/* So, declare Kp, Ki and Kd in an array */
fractional kCoeffs[] = {0,0,0};


//*********************************************************************************************************************************************************
//                                                   FUNCTION DECLARATIONS
//**********************************************************************************************************************************************************************
void InitializePWM1(void);
void InitializeADC(void);
//void InitializeInterrupt(void);

void TMR2Intialize(void);
void TMR3Intialize(void);
void TMR5Intialize(void);
void TMR4Intialize(void);

void CnpinInitialize();

int GetAdcValue(char channel);
void CheckOutputCurrent(void);
void CheckOutputVoltage(void);
void CheckOutputVoltageHigh(void);
void CheckInputVoltage(void);
void CheckInputCurrent(void);
void CheckIpConditions(void);
void CheckFaultConditions(void);
void CheckBattVolatge(void);
void CheckBattChargCurrent(void);
void CheckBattDischargCurrent(void);

void delay(unsigned int);
void Lcd_Chr_Cp(char out_char);
void Lcd_Chr(char row, char column, char out_char);
void Lcd_Out_Cp(char *text);
void Lcd_Out(char row, char column, char *text);
void Lcd_Cmd(unsigned char out_char);
void Lcd_Initialize(void);

void ConfigurePorts(void);

void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char *res, int afterpoint);

//void INTx_IO_Init(void);

void Display(unsigned char KeypadVal);

float ArrayAvgFunc(volatile unsigned int *ArrayBasepointer);

//void __attribute__((__interrupt__)) _INT0Interrupt(void){
//    IFS0bits.INT0IF = 0;
//    TMR3_StartTimer();
//    FlagBattLow=1;
//    _INT0EP=1;
//}
void __attribute__((interrupt, no_auto_psv)) _PWMInterrupt(void){
       _PWMIF=0;
        i++;
        if(i==128)
        {
            i=0;
            LATEbits.LATE0=~LATEbits.LATE0;
            temp=GetAdcValue(ANA0_PWM_Control);
            if((temp>(prevtemp+6))||(temp<(prevtemp-6))){
                if(softstart==1){
                    PIDRefrenceVal=temp;
                }
                else{
                    PIDSetpoint=temp;
                    fooPID.controlReference=PIDSetpoint;
                }
                prevtemp=temp;
            }
            if(softstart==1){
                fooPID.controlReference=(++PIDSetpoint);
                if(PIDSetpoint>PIDRefrenceVal){
                    softstart=0;
                    PIDSetpoint=PIDRefrenceVal;
                    fooPID.controlReference=PIDSetpoint;
                }
            }
            DelayBetLcdParm++;
        }
        if(i%6==0)
        {
            temp=GetAdcValue(ANA1_Op_HighLow);
                fooPID.measuredOutput=temp;
                PID(&fooPID);
                result=fooPID.controlOutput+PIDSetpoint;
            if(result>1024) 
            {
                result=1024;
                fooPID.controlOutput=(1024-PIDSetpoint);
            }
        }
        k=i+1; 
        if(k==128)
        {
            k=0;
        } 
        DutyCycle=((LookUpTable[k]*result)>>10);
        if(Overall_Fault_Indicator){
            PDC1=0;
        }
        else{
            PDC1=DutyCycle;
            
        }
        if(i%10==0){
        if(softstart==1){
            CheckOutputVoltageHigh();   
        }
        else{
            SoftstartCount++;
            if(SoftstartCount>1280){
                CheckOutputVoltage();
            }
            else{
                CheckOutputVoltageHigh();
            }
        }
        //CheckFaultConditions();
        CheckOutputCurrent();
        CheckInputCurrent();
        
        }
        if(i%64==0){
            CheckBattVolatge();
            CheckInputVoltage();
            CheckIpConditions();
            CheckBattChargCurrent();
            CheckBattDischargCurrent();
        }
}

//void _ISR _T2Interrupt(void){
//    unsigned int static count=0;
//    _T2IF=0;
//    if(count==0){
//        CheckOutputCurrent();
//    }
//    else if(count==1){
//        CheckInputVoltage();
//    }
//    else if(count==2){
//        CheckInputCurrent();
//    }
//    else if(count==4){
//        if(softstart==1){
//            CheckOutputVoltageHigh();   
//        }
//        else{
//            CheckOutputVoltage();
//        }
//        count=0;
//    }
//    CheckIpConditions();
//    CheckFaultConditions();
//    count++;
//    TMR2=0;
// }
 void _ISR _T3Interrupt(void){
     static unsigned int count=0;
    _T3IF=0;
    if(count==1){
        Overall_Fault_Indicator=1;
        PDC1=0;
        //PWM1_StopPwm();
        //TMR4_StopTimer();
        _INT0IE=0;
        IEC0bits.INT0IE = 0;
        TMR3_StopTimer();
        count=0;
    }
    TMR3=0;
    count++;
 }
 void _ISR _T5Interrupt(void){
     static unsigned int count=0;
    _T5IF=0;
    if(count==4){
        Charger_Card=1;
        TMR5_StopTimer();
        count=0;
    }
    TMR5=0;
    count++;
 }
//void _ISR _T4Interrupt(void){
//    _T4IF=0;
//    temp=GetAdcValue(ANA1_Op_HighLow);
//    fooPID.measuredOutput=temp;
//    PID(&fooPID);
//    result=fooPID.controlOutput+PIDSetpoint;
//    if(result>1024) 
//    {
//        result=1024;
//        fooPID.controlOutput=(1024-PIDSetpoint);
//    }
//    TMR4=0;
// }


//*********************************************************************************************************************************************************
//                                                             MAIN CODE
// *********************************************************************************************************************************************************** 
int main(void) 
{
    char *str,BlankConst=0;
    ConfigurePorts();
    InitializePWM1();
    InitializeADC();                                                 
    TMR2Intialize();
    TMR3Intialize();
    TMR4Intialize();
    TMR5Intialize();
    Lcd_Initialize();
    CnpinInitialize();
    BattTripVolatge=(BattSetVoltage*0.88*1.9904)-8; //0.816
    BattBuzzerVolatge=(BattSetVoltage*0.9*1.9904)-8; //0.836
    BattHighVolatge=(BattSetVoltage*1.26*1.991);
    OverCurrentVolatge=(KVA_Val*1000/230)/OpCurrentMultFactor;
    str=Transtech;
    Lcd_Out(1,0,str);
    ShortPressCount=0;
    count=0;
    delay(10000);
   // Op_High_Indicator=1;
/*
Step 1: Initialize the PID data structure, fooPID
*/
    fooPID.abcCoefficients = &abcCoefficient[0];    /*Set up pointer to derived coefficients */
    fooPID.controlHistory = &controlHistory[0];     /*Set up pointer to controller history samples */
    PIDInit(&fooPID);                               /*Clear the controler history and the controller output */
	kCoeffs[0] = Q15(0.7);
	kCoeffs[1] = Q15(0.1);
	kCoeffs[2] = Q15(0.09);
    PIDCoeffCalc(&kCoeffs[0], &fooPID);             /*Derive the a,b, & c coefficients from the Kp, Ki & Kd */
    fooPID.controlReference =0;           /*Set the Reference Input for your controller */
    PDC1=0;
    Overall_Fault_Indicator=0;
    EnablePwmInterrupt();
    EnableTimer3Interrupt();
    EnableTimer5Interrupt();
   // TMR2_StartTimer();
    //TMR4_StartTimer();
    //EnableTimer4Interrupt();
    //EnableTimer2Interrupt();
    //INTx_IO_Init();
//    _prog_addressT EE_addr;
//    int temp = 0;
//
//    /* initialize a variable to represent the Data EEPROM address */
//    _init_prog_address(EE_addr, fooArrayInDataEE);
//    
//    /*Copy array "fooArrayinDataEE" from DataEEPROM to "fooArray2inRAM" in RAM*/
//    _memcpy_p2d16(fooArray2inRAM, EE_addr, _EE_ROW);
//
//    /*Erase a row in Data EEPROM at array "fooArrayinDataEE" */
//    _erase_eedata(EE_addr, _EE_ROW);
//    _wait_eedata();
//
//    /*Write a row to Data EEPROM from array "fooArray1inRAM" */
//    _write_eedata_row(EE_addr, fooArray1inRAM);
//    _wait_eedata();
    while(1){
        FaultDisplayFlag=(Overall_Fault_Indicator||Charger_Card);
        BattAvgAnalogVolatge=ArrayAvgFunc(BattAnalogVolatge);
            if(AutoScroll==1&&(!FaultDisplayFlag)){
                if(BlankConst==1){
                    BlankSecondRow();
                    BlankConst=0;
                }
                if((DelayBetLcdParm%100)==0){
                    //BlankSecondRow();
                    Display(count);
                    if(DelayBetLcdParm>1000){
                        DelayBetLcdParm=0;
                        count++;
                        if(count>=LcdParametersDisplayNo){
                            count=0;
                        }                
                    }
                }
                FlagSecondtime=0;            }
            else{  
                if(FaultDisplayFlag&&(!FlagSecondtime)){
                    ShortPressCount=20;
                    FlagSecondtime=1;
                    BlankConst=1;
                }
                Display(ShortPressCount/2);
            }  
        if(_CNIF==1){
            ShortPressCount=ShortPressCount+1;
            BlankSecondRow();
            if(FaultDisplayFlag){
                if(ShortPressCount>=22){
                    ShortPressCount=0;
                }
                if(ShortPressCount==(LcdParametersDisplayNo*2)){
                    ShortPressCount=20;
                    Lcd_Cmd(0x01);
                }
            }
            else{
                if(ShortPressCount>=(LcdParametersDisplayNo*2)){
                ShortPressCount=0;
                }
            }
            _CNIF=0;
            PORTC;
        }
    }
    return -1;
}
//**********************************************************************************************************************************************************************************
//                                                          FUNCTION DEFINITIONS
//**********************************************************************************************************************************************************************************

void Lcd_Chr_Cp(char out_char){
    char Temp=0;
    LCD_RS=1;
    Temp=out_char;
    Temp=Temp>>4;
    Temp&=0x0F;
    LATF&=0xF0;
    LATF|=Temp;
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
    
    Temp=out_char;
    Temp&=0x0F;
    LATF&=0xF0;
    LATF|=Temp;
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
}
void Lcd_Chr( char row, char column, char out_char){
    char CommVal=0;
    if(row==1){
        CommVal=0x80;
    }
    else{
        CommVal=0xC0;
    }
    CommVal=CommVal+column;
    Lcd_Cmd(CommVal);
    Lcd_Chr_Cp(out_char);
}
void Lcd_Out_Cp(char *text){
    unsigned int j=0;
    for(;text[j]!=0;j++)
    Lcd_Chr_Cp(text[j]);
}
void Lcd_Out(char row,char column, char *text){
    unsigned int j=0;
    unsigned char CommVal=0;
    if(row==1){
        CommVal=0x80;
    }
    else{
        CommVal=0xC0;
    }
    CommVal=CommVal+column;
    Lcd_Cmd(CommVal);
    for(;text[j]!=0;j++)
    Lcd_Chr_Cp(text[j]);
}
void Lcd_Cmd(unsigned char out_char){
    unsigned char Temp=0;
    LCD_RS=0;
    Temp=out_char;
    Temp&=0xF0;
    Temp=Temp>>4;
    Temp&=0x0F;
    LATF&=0xF0;
    LATF|=Temp;
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
    
    Temp=out_char;
    Temp&=0x0F;
    LATF&=0xF0;
    LATF|=Temp;
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
 
}
void Lcd_Initialize(void){
    delay(Lcd_delay+10);
   // lcd_init_write(0x30);   //Special Sequence:Write Function Set.
    LATF|=0x03;
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
    delay(Lcd_delay);
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
    delay(Lcd_delay);
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
    delay(Lcd_delay);
    LATF&=0xF0;
    LATF|=0x02;
    LCD_EN=1;
    delay(Lcd_delay);
    LCD_EN=0;
    Lcd_Cmd(0x28);                                                // Clear Screen & Returns the Cursor Home
    delay(Lcd_delay);
    Lcd_Cmd(0x0E);      
    delay(Lcd_delay);      
    Lcd_Cmd(0x01);           //Inc cursor to the right when writing and don?t shift screen
    delay(Lcd_delay);
    Lcd_Cmd(0x06);
    delay(Lcd_delay);
    Lcd_Cmd(0x80);
    delay(Lcd_delay);
   // Lcd_Cmd(0xC0);
    delay(Lcd_delay);
    Lcd_Cmd(0x0E);
}
void ConfigurePorts(){
    TRISB=0xFFF;            //Setting as input for analog function
    TRISE=0;
    //TRISEbits.TRISE8=0;  //INTO as i/p
    TRISEbits.TRISE2=0;   //BattBuzzer
    TRISF=0;
    TRISD=0;
    TRISC=0;
    TRISCbits.TRISC13=1;        //Parameter increment for lcd
    TRISCbits.TRISC14=1;        //AutoScroll I/p
    LATF=0;
    Overall_Fault_Indicator=1;
    LATB=0;
   // LATE=0;
    LATC=0;
    LATD=0;
   // LATBbits.LATB7=1;
}

int GetAdcValue(char channel){
    ADCHS=channel;
    ADC_StartSampling();
    while(!ADCON1bits.DONE);
    return ADCBUF0;
}

void reverse(char *str, int len)
{
    int l=0, j=len-1, temp;
    while (l<j)
    {
        temp = str[l];
        str[l] = str[j];
        str[j] = temp;
        l++; j--;
    }
}
int intToStr(int x, char str[], int d)
{
    int l = 0;
    while (x)
    {
        str[l++] = (x%10) + '0';
        x = x/10;
    }
 
    // If number of digits required is more, then
    // add 0s at the beginning
    while (l < d)
        str[l++] = '0';
 
    reverse(str, l);
    str[l] = '\0';
    return l;
}

void ftoa(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;
 
    // Extract floating part
    float fpart = n - (float)ipart;
 
    // convert integer part to string
    int i = intToStr(ipart, res, 0);
 
    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot
 
        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);
 
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

void InitializePWM1()
{
    INTCON1=0x0000;
    INTCON2=0x0000;
    IFS2bits.PWMIF=0;
    IEC2bits.PWMIE=0;
    PTCON=0x2000;
    PTPER=2303;
    PWMCON1=0x0F10;
    PWMCON2=0x0000;
    PDC1=0;
    PTCONbits.PTEN=1;
    _PWMIP=7;
}
void InitializePWM2(){
    INTCON1=0x0000; 
    INTCON2=0x0000;
    IFS2bits.PWMIF=0;
    IEC2bits.PWMIE=0;
    PTCON=0x2000;
    PTPER=2303;
    PWMCON1=0x0F10;
    PWMCON2=0x0000;
    PDC1=0;
    PTCONbits.PTEN=1;
    _PWMIP=7;
}

void InitializeADC(){
    ADPCFG=0xFE00;
    ADCON2bits.VCFG=0;
    ADCON3bits.ADCS=9;
    ADCON2bits.CHPS=0;
    ADCON1bits.SIMSAM=1;
    ADCSSL=0x0000;
    ADCHS=0x0000;
    ADCON1bits.SSRC=7;
    ADCON1bits.ASAM=0;
    ADCON1bits.DONE=0;
    ADCON1bits.ADON=0;
    ADCON1bits.SAMP=1;
    ADCON3bits.SAMC=1;
    ADCON1bits.FORM=0;
    ADCON2bits.BUFS=0;
    ADCON2bits.SMPI=0;
    ADCON2bits.BUFM=0;
    ADCON2bits.ALTS=0;
    ADC_ON();
}
void TMR2Intialize(void){
    TMR2_StopTimer();
    T2CON=0;
    _T2IP=3;
    TMR2=0;
    T2CONbits.TCKPS0=0;
    T2CONbits.TCKPS1=0;
    PR2=T2_Period;
    _T2IF=0;
    _T2IE=0;
    
}
void TMR3Intialize(void){
    TMR3_StopTimer();
    T3CON=0;
    _T3IP=5;
    TMR3=0;
    T3CONbits.TCKPS0=1;
    T3CONbits.TCKPS1=1;
    PR3=T3_Period;
    _T3IF=0;
    _T3IE=0;
    
}
void TMR5Intialize(void){
    TMR5_StopTimer();
    T5CON=0;
    _T5IP=4;
    TMR5=0;
    T5CONbits.TCKPS0=1;
    T5CONbits.TCKPS1=1;
    PR5=T5_Period;
    _T5IF=0;
    _T5IE=0;
    
}
void TMR4Intialize(void){
    TMR4_StopTimer();
    T4CON=0;
    _T4IP=6;
    TMR4=0;
    T4CONbits.TCKPS0=0;
    T4CONbits.TCKPS1=0;
    PR4=T4_Period;
    _T4IF=0;
    _T4IE=0;
    
}
//void INTx_IO_Init(void)
//{
//        _INT0EP=0;
//        IFS0bits.INT0IF = 0;    /*Reset INT0 interrupt flag */
//        IEC0bits.INT0IE = 1;    /*Enable INT0 Interrupt Service Routine */
//
//        IFS1bits.INT1IF = 0;    /*Reset INT1 interrupt flag */
//        IEC1bits.INT1IE = 0;    /*Enable INT1 Interrupt Service Routine */
//
//        IFS1bits.INT2IF = 0;    /*Reset INT0 interrupt flag */
//        IEC1bits.INT2IE = 0;    /*Enable INT0 Interrupt Service Routine */
//        _INT0IP=3;
//}

void CheckOutputVoltage(void){
   volatile static unsigned int AnalogVal=0,l=0;
    AnalogVal=GetAdcValue(ANA2_Op_Feedback);
    OpAnalogVoltage[l]=AnalogVal;
    l++;
    if(l==24){
        l=0;
        OpVoltReadyForAvg=1;
    }
    if(AnalogVal>582){
        FlagOHigh=1;
        Op_High_Indicator=1;
    }
    else if(AnalogVal<542){
        FlagOHigh=0;
        if(Overall_Fault_Indicator==0){
            Op_High_Indicator=0;
        }
    }
        
    if(AnalogVal<445){
        FlagOLow=1;
        Op_Low_Indicator=1;
    }
    else if(AnalogVal>450) {
        FlagOLow=0;
        if(Overall_Fault_Indicator==0){
        Op_Low_Indicator=0;
        }
    }
}
void CheckOutputVoltageHigh(void){
   volatile static unsigned int AnalogVal=0,l=0;
    AnalogVal=GetAdcValue(ANA2_Op_Feedback);
    OpAnalogVoltage[l]=AnalogVal;
    l++;
    if(l==24){
        l=0;
        OpVoltReadyForAvg=1;
    }
    if(AnalogVal>582){
        FlagOHigh=1;
        Op_High_Indicator=1;
    }
    else if(AnalogVal<542){
        FlagOHigh=0;
        if(Overall_Fault_Indicator==0){
        Op_High_Indicator=0;
        }
    } 
}
void CheckOutputCurrent(void){
    volatile static unsigned int AnalogVal=0,l=0;
    AnalogVal=GetAdcValue(ANA3_Op_Overcurrent);
    OpAnalogCurrent[l]=AnalogVal;
    l++;
    if(l==24){
        l=0;
        OpCurrentReadyForAvg=1;
    }
    if(AnalogVal>OverCurrentVolatge)   {       
        FlagOOverCurrent=1;
        Op_Overcurrent_Indicator=1;
    }
    else {
        if(Overall_Fault_Indicator==0){
        Op_Overcurrent_Indicator=0;
        FlagOOverCurrent=0;
        }
    }
}
void CheckInputVoltage(void){
    volatile static unsigned int AnalogVal=0,l=0;
    AnalogVal=GetAdcValue(ANA4_Ip_Voltage);
    IpAnalogVolatge[l]=AnalogVal;
    l++;
    if(l==24){
        l=0;
        IpVoltReadyForAvg=1;
    }
    if(AnalogVal>622){
        FlagIHigh=1;
        Mains_High_Indicator=1;
    }
    else if(AnalogVal<583){
        Mains_High_Indicator=0;
        FlagIHigh=0;
    }
    
    if(AnalogVal<362){
        FlagILow=1;
        Main_Low_Indicator=1;
    }
    else if(AnalogVal>389){
        FlagILow=0;
        Main_Low_Indicator=0;
    }
}
void CheckInputCurrent(void){
    volatile static unsigned int AnalogVal=0,l=0;
    AnalogVal=GetAdcValue(ANA5_Ip_Current);
    IpAnalogCurrent[l]=AnalogVal;
    l++;
    if(l==24){
        l=0;
        IpCurrentReadyForAvg=1;
    }
    if(AnalogVal>512)   {       
        FlagIOverCurrent=1;
        
    }
    else {
        FlagIOverCurrent=0;
    }
}
void CheckIpConditions(void){
    if(FlagIHigh==1||FlagILow==1||FlagIOverCurrent==1){
        TMR5_StartTimer();
    }
    else{
            TMR5_StopTimer();
            Charger_Card=0;
    }
}
void CheckFaultConditions(void){
    if(FlagOHigh||FlagOLow||FlagOOverCurrent||FlagBattLow){
        TMR3_StartTimer();
        
    }
    else {
        TMR3_StopTimer();
        //Overall_Fault_Indicator=0;
    }
        BattBuzzerIndicator=BuzzerFlag;
}
void CheckBattVolatge(void){
    volatile static unsigned int AnalogVal=0,l=0;
    AnalogVal=GetAdcValue(ANA6_Batt_Voltage);
    BattAnalogVolatge[l]=AnalogVal;
    l++;
    if(l==25){
        l=0;
        BattVoltReadyForAvg=1;
    }
    if(BattAvgAnalogVolatge<BattBuzzerVolatge){
        BuzzerFlag=1;
    }
    else{
        BuzzerFlag=0;
    }
    if(BattAvgAnalogVolatge<BattTripVolatge){
        BuzzerFlag=0;
        FlagBattLow=1;
        Batt_Low_Indicator=1;
    }
    else{
        if(Overall_Fault_Indicator==0){
            FlagBattLow=0;
            Batt_Low_Indicator=0;
        }
    }
}
void CheckBattChargCurrent(void){
    volatile static unsigned int AnalogVal=0,l=0;
    AnalogVal=GetAdcValue(ANA7_Batt_Charging_Current);
    BattChargingAnalogCurrent[l]=AnalogVal;
    l++;
    if(l==24){
        l=0;
        BattChargCurrReadyForAvg=1;
    }
}
void CheckBattDischargCurrent(void){
    volatile static unsigned int AnalogVal=0,l=0;
    AnalogVal=GetAdcValue(ANA8_Batt_Discharging_Current);
    BattDisChargingAnalogCurrent[l]=AnalogVal;
    l++;
    if(l==24){
        l=0;
        BattDisChargCurrReadyForAvg=1;
    }
}

void delay(unsigned int j){
                unsigned int l=0,k=0;
                for(k=0;k<1000;k++){
                for(l=0;l<=j;l++);
                }
                
}

float ArrayAvgFunc(volatile unsigned int* ArrayBasepointer){
    unsigned char l=0;
    float avg=0;
    for(l=0;l<25;l++){
        avg=avg+ArrayBasepointer[l];
    }
    avg=avg/25;
    
    return avg;
}

void Display(unsigned char KeypadVal){
    char *str,Val[3];
    float ParameterVal=0,Volt=0,Curr=0;
   // Lcd_Cmd(0x01);
    
    switch (KeypadVal){
    case 0:
        if(OpVoltReadyForAvg){
            OpAvgAnalogVoltage=ArrayAvgFunc(OpAnalogVoltage);
            OpVoltReadyForAvg=0;
        }
        str=OPVolatge;
        Lcd_Out(1,0,str);
        ParameterVal=(unsigned int)(OpAvgAnalogVoltage*OpVoltMultFactor);
        intToStr(ParameterVal, Val, 3);
        Lcd_Out(2,6,Val);
        Lcd_Out(2,9," VAC");
        break;
        
    case 1:
        if(OpCurrentReadyForAvg){
            OpAvgAnalogCurrent=ArrayAvgFunc(OpAnalogCurrent);
            OpCurrentReadyForAvg=0;
        }
        str=OPCurrent;
        Lcd_Out(1,0,str);
        ParameterVal=(unsigned int)(OpAvgAnalogCurrent*OpCurrentMultFactor);
        intToStr(ParameterVal, Val, 3);
        Lcd_Out(2,6,Val);
        Lcd_Out(2,9," IAC");
        break;
        
    case 2: 
        if(IpVoltReadyForAvg){
            IpAvgAnalogVolatge=ArrayAvgFunc(IpAnalogVolatge);
            IpCurrentReadyForAvg=0;
        }
        str=MainsVoltage;
        Lcd_Out(1,0,str);
        ParameterVal=(unsigned int)(IpAvgAnalogVolatge*IpVoltMultFactor);
        intToStr(ParameterVal, Val, 3);
        Lcd_Out(2,6,Val);
        Lcd_Out(2,9," VAC");
        break;
        
    case 7:   
        if(IpCurrentReadyForAvg){
            IpAvgAnalogCurrent=ArrayAvgFunc(IpAnalogCurrent);
            IpCurrentReadyForAvg=0;
        }
        str=MainsCurrent;
        Lcd_Out(1,0,str);
        ParameterVal=(unsigned int)(IpAvgAnalogCurrent*IpCurrentMultFactor);
        intToStr(ParameterVal, Val, 3);
        Lcd_Out(2,6,Val);
        Lcd_Out(2,9," IAC");
        break;
        
    case 4:
       if(BattVoltReadyForAvg){
            BattAvgAnalogVolatge=ArrayAvgFunc(BattAnalogVolatge);
            BattVoltReadyForAvg=0;
        }
        str=BattVoltage;
        Lcd_Out(1,0,str);
        ParameterVal=(unsigned int)(BattAvgAnalogVolatge*BattVoltMultFactor)+4;
        intToStr(ParameterVal, Val, 3);
        Lcd_Out(2,6,Val);
        Lcd_Out(2,9," VDC");
        break;
        
    case 5:
        if(BattChargCurrReadyForAvg){
            BattAvgChargingAnalogCurrent=ArrayAvgFunc(BattChargingAnalogCurrent);
            BattChargCurrReadyForAvg=0;
        }
        str=BattChargCurrent;
        Lcd_Out(1,0,str);
        ParameterVal=(unsigned int)(BattAvgChargingAnalogCurrent*BattChargCurrentMultFactor);
        intToStr(ParameterVal, Val, 3);
        Lcd_Out(2,6,Val);
        Lcd_Out(2,9," IDC");
        break;
        
    case 6:    
        if(BattDisChargCurrReadyForAvg){
            BattAvgDisChargingAnalogCurrent=ArrayAvgFunc(BattDisChargingAnalogCurrent);
            BattDisChargCurrReadyForAvg=0;
        }
        str=BattDisChargCurrent;
        Lcd_Out(1,0,str);
        ParameterVal=(unsigned int)(BattAvgDisChargingAnalogCurrent*BattDisChargCurrentMultFactor);
        intToStr(ParameterVal, Val, 3);
        Lcd_Out(2,6,Val);
        Lcd_Out(2,9," IDC");
        break;
        
    case 3:
        str=OPFrequency;
        Lcd_Out(1,0,str);
        Lcd_Out(2,6,"49.9 Hz");
        break;
        
    case 8:
        str=OpPower;
        Lcd_Out(1,0,str);
        Volt=OpAvgAnalogVoltage*OpVoltMultFactor;
        Curr=OpAvgAnalogVoltage*OpCurrentMultFactor;
        Volt=Volt*Curr;
        ParameterVal=(Volt*0.001);
        ftoa(ParameterVal, Val, 3);
        Lcd_Out(2,6,Val);
        Lcd_Out(2,9," KVA");
        break;
        
    case 9:    
        str=IpPower;
        Lcd_Out(1,0,str);
        ParameterVal=IpAvgAnalogVolatge*IpAvgAnalogCurrent*0.001;
        ftoa(ParameterVal, Val, 3);
        Lcd_Out(2,6,Val);
        Lcd_Out(2,9," KVA");
        break;
    case 10:
                
                 Lcd_Out(1,0,Fault);
                 if(Overall_Fault_Indicator){
                    if(FlagOHigh){
                        Lcd_Out(2,0,HighOPVoltage);
                        delay(3000);
                    }
                    if(FlagOLow){
                        Lcd_Out(2,0,LowOPVoltage);
                        delay(3000);
                    }
                    if(FlagOOverCurrent){
                        Lcd_Out(2,0,HighOpCurrent);
                        delay(3000);
                    }
                    if(FlagBattLow){
                        Lcd_Out(2,0,BattLow);
                        delay(3000);
                    }
                 delay(3000);   
                }
                if(Charger_Card==1){
                    if(FlagIHigh){
                        //BlankSecondRow();
                        Lcd_Out(2,0,MainsVoltageHigh);
                        delay(3000);
                    }
                    if(FlagILow){
                        //BlankSecondRow();
                        Lcd_Out(2,0,MainsVoltageLow);
                        delay(3000);
                    }
                    if(FlagIOverCurrent){
                        //BlankSecondRow();
                        Lcd_Out(2,0,MainsCurrentHigh);
                        delay(3000);
                    }
                 delay(3000);   
                }
                 
                break;

    }
}
void CnpinInitialize(){
    TRISCbits.TRISC13=1;
    CNEN1bits.CN1IE=1;
    CNPU1bits.CN1PUE=1;
    _CNIF=0;
    IPC3bits.CNIP=3;
    _CNIE=0;
}


void _trapISR _OscillatorFail(void)
{       INTCON1bits.OSCFAIL = 0;
        Overall_Fault_Indicator=1;
        Lcd_Out(1,0,"DSP Error      ");
        Lcd_Out(2,0,"OcillatorFail  ");
        delay(1000);
		while(1);
		
}

void _trapISR _AddressError(void)
{       INTCON1bits.ADDRERR = 0;
        Overall_Fault_Indicator=1;
        Lcd_Out(1,0,"DSP Error       ");
		Lcd_Out(2,0,"AdressError     ");
        delay(1000);
		while(1);
}

void _trapISR _StackError(void)
{       INTCON1bits.STKERR = 0;
        Overall_Fault_Indicator=1;
        Lcd_Out(1,0,"DSP Error       ");
		Lcd_Out(2,0,"StackError      ");
        delay(1000);
		while(1);
}

void _trapISR _MathError(void)
{       INTCON1bits.MATHERR = 0;
        Overall_Fault_Indicator=1;
        Lcd_Out(1,0,"DSP Error       ");
		Lcd_Out(2,0,"MathError       ");
        delay(1000);
        while(1);
}

//***************************************************************************************************************************************************************
//                                               END OF CODE
//***************************************************************************************************************************************************************