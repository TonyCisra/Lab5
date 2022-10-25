/* Universidad del Valle
 * Programacion de microcontroladores
 * Laboratorio No. 5
 */
// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#define _XTAL_FREQ 500000


void setup(void);
void setupINTOSC(void);
void setupADC(void);
void setupPWM(void);    //Definimos las funciones

// VARIABLES
unsigned short CCPR = 0;    //Valor que entrara en el primer CCP
unsigned short CCPRo = 0;   //Valor que entrara en el segundo CCP
int x;
int x0;
int x1;
unsigned short y0;
unsigned short y1;          //VAriables para el map

uint8_t valTmr0 = 131;      //Valor del TImer 0
uint8_t valPot;             //Valor que entrega en 3er pot
uint8_t contador;           //Contador que terminara el 3er PWM

// Funciones
void setup(void);
unsigned short map(int val, int i_min, int i_max, short o_min, short o_max);

//**************************
// Interrupciones
//**************************

void __interrupt() isr(){
    
    if (ADIF) //Condicionamos cada interrupcion del ADC
    {
        if (ADCON0bits.CHS == 0b0000) //Vemos el AN0 si hay movimineto
        {
            CCPR = map(ADRESH, 0, 255, 7, 15); //Mapeamos la salida del AN0 condicionandola que sea de 7 a 15 (Valores que necesita el serv)
            CCPR1L = (uint8_t)(CCPR); //Metemos nuestra variable en el CCP1
            CCP1CONbits.DC1B = 0b11;  //Escribimos los bits menos significativos
        } 
        else if (ADCON0bits.CHS == 0b0001)//Vemos el AN1 si hay movimiento
        {
            CCPRo = map(ADRESH, 0, 255, 7, 15); //Mapeamos la salida del AN1 condicionandola que sea de 7 a 15 (Valores que necesita el servo)
            CCPR2L = (uint8_t)(CCPRo); //Metemos nuestra variable en el CCP1
            CCP2CONbits.DC2B1 = 1;
            CCP2CONbits.DC2B0 = 1;    //Escrbimos los bits menos sifinicativos
        }
        else
        {
            valPot = ADRESH;         //Metemos el valor del 3er pot en nuestra variable para el 3er PWM
        }
        ADIF =  0; //Limpiamos la bandera
    }
    if (T0IF) //Condicionamos cada intrrupcion del timer
    {
        contador ++;     //Aumentamos cada nuestra variables cada desborde
        if(contador < valPot)   //Condicionamos que si nuestro contador es menos al valor del POT 3
        {
            PORTDbits.RD0 = 1; //Encedemos nuestro led
        }
        else
        {
            PORTDbits.RD0 = 0; //Apagamos nuestro led
        } 
        TMR0 = valTmr0;         //Agregamos el valor del timer
        T0IF = 0;               //Limpiamos la bandera
    }
}

    

//**************************
// CÃ³digo Principal
//**************************
void main(void) {
    
    setup();
    setupADC();
    setupINTOSC();
    setupPWM();
    
    
    while(1){

        if(ADCON0bits.GO == 0){                 // Iniciamos el modulo ADC
            if(ADCON0bits.CHS == 0b0000)         // Vemos el canal 1
            {    
                ADCON0bits.CHS = 0b0001;         // Que sea el canal 2
            }
            else if(ADCON0bits.CHS == 0b0001)    // Vemos el canal 2
            {
                ADCON0bits.CHS = 0b0010;         // Que sea el canal 3
            }
            else
            {
                ADCON0bits.CHS = 0b0000;         //Que sea el canal 1
                __delay_us(1000);
            }
            ADCON0bits.GO = 1;
        }
    }   
            
    }
//**************************
// FunciÃ³n para configurar GPIOs
//**************************
void setup(void){
    ANSELH = 0;
    TRISD = 0;
    PORTD = 0;
    
    
    //Habilitamos la interrupcionese
    INTCONbits.GIE  = 1;
    INTCONbits.PEIE = 1;
    
    PIE1bits.ADIE     = 1;
    INTCONbits.TMR0IE = 1;
    
    PIR1bits.ADIF   = 0;
    INTCONbits.T0IF = 0;
    
    T0IF = 0;
    T0IE = 1;
    GIE  = 1;

}
//**************************
// FunciÃ³n para configurar PWM
//**************************
void setupINTOSC(void){
    OSCCONbits.IRCF = 0b011;       // 500 KHz
    OSCCONbits.SCS = 1;
}
//**************************
// FunciÃ³n para configurar ADC
//**************************
void setupADC(void){
    
    // Paso 1 Seleccionar puerto de entrada
    //TRISAbits.TRISA0 = 1;
    TRISA = TRISA | 0x111;
    ANSEL = ANSEL | 0x111;
    
    // Paso 2 Configurar mÃ³dulo ADC
    
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;       // Fosc/ 8
    
    ADCON1bits.VCFG1 = 0;       // Ref VSS
    ADCON1bits.VCFG0 = 0;       // Ref VDD
    
    ADCON1bits.ADFM = 0;        // Justificado hacia izquierda
    
    ADCON0bits.CHS = 0b0000;        // Canal AN0
    ADCON0bits.CHS = 0b0001;        // Canal AN1
    ADCON0bits.CHS = 0b0010;        // Canal AN2
    
    ADCON0bits.ADON = 1;        // Habilitamos el ADC
    __delay_us(100);
    
    //Configuracion del Timer0
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.T0SE = 0;
    OPTION_REGbits.PSA  = 0;
    OPTION_REGbits.PS   = 0b000;
    TMR0 = valTmr0;
    
}
//**************************
// FunciÃ³n para configurar PWM
//**************************
void setupPWM(void){
    // Paso 1
    TRISCbits.TRISC2 = 1;
    TRISCbits.TRISC1 = 1;
    
    // Paso 2
    PR2 = 155;      // Periodo de 20mS
    
    // Paso 3
    CCP1CON = 0b00001100;        // P1A como PWM 
    CCP2CON = 0b00001100;
    
   // Paso 4
    CCP1CONbits.DC1B = 0b11;        // CCPxCON<5:4>
    CCPR1L = 11;        // CCPR1L 
                        // CALCULO PARA 1.5mS de ancho de pulso
    CCP2CONbits.DC2B1 = 1;
    CCP2CONbits.DC2B0 = 1;          // CCPxCON<5:4>
    CCPR2L = 11;        // CCPR2L 
                        // CALCULO PARA 1.5mS de ancho de pulso
    // Paso 5
    TMR2IF = 0;
    T2CONbits.T2CKPS = 0b11;      // Prescaler de 1:16
    TMR2ON = 1;         // Encender timer 2
    
    // Paso 6
    while(!TMR2IF);
    TRISCbits.TRISC2 = 0;   // Habilitamos la salida del PWM
    TRISCbits.TRISC1 = 0;
    
}

// Función que hace una interpolación de datos
unsigned short map(int x, int x0, int x1, short y0, short y1){
    return (unsigned short)(y0 + ((float)(y1-y0)/(x1-x0)) * (x-x0));
}