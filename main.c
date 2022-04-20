/* 
 * File:   main.c
 * Author: josej
 *
 * Created on April 13, 2022, 11:21 AM
 */

#include <xc.h>
#include <stdint.h>

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

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 4000000

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
 uint16_t valor = 0;
 uint8_t cont = 0;
 uint16_t valores[3];    
 uint8_t display[3];    
 uint8_t i;             // Declarada
 uint8_t banderas = 0;  // Declarada e inicializada
 uint8_t tabla[10] = {0b11101101,
                      0b10100000,
                      0b11001110,
                      0b11101010,
                      0b10100011,
                      0b01101011,
                      0b01101111,
                      0b11100000,
                      0b11101111,
                      0b11100011};   
  uint8_t tabla2[10] = {0b11111101,
                        0b10110000,
                        0b11011110,
                        0b11111010,
                        0b10110011,
                        0b01111011,
                        0b01111111,
                        0b11110000,
                        0b11111111,
                        0b11110011};

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
void R_TMR0(void);
void obtener_valor(void);
void set_display(void);
void mostrar_valor(void);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PIR1bits.ADIF){              // Fue interrupción del ADC?
        if(ADCON0bits.CHS == 0){    // Verificamos AN0
            PORTB = ADRESH;         // Mostramos ADRESH en PORTB
        }
        
        else if (ADCON0bits.CHS == 1){  // Verificamos AN1
            cont = ADRESH;              // ADRESH en contador
        }
        PIR1bits.ADIF = 0;          // Limpiamos bandera de interrupción
    }
    
    if(INTCONbits.T0IF){
        mostrar_valor();
        R_TMR0();                 // Reset de TMR0
    }
    
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        if(ADCON0bits.GO == 0){             // No hay proceso de conversion

            if(ADCON0bits.CHS == 0b0000)    
                ADCON0bits.CHS = 0b0001;    // Cambio de canal
            else if(ADCON0bits.CHS == 0b0001)
                ADCON0bits.CHS = 0b0000;    // Cambio de canal
            __delay_us(40);                 // Tiempo de adquisición
            ADCON0bits.GO = 1;              // Iniciamos proceso de conversión
        } 
        valor = cont*2;
        if(valor>500){
            valor = 500;
        }
        obtener_valor();
        set_display();
    }
    return;
}

void R_TMR0(void){
     TMR0 = 6;                // Delay de 0.5 mS
     INTCONbits.T0IF = 0;
     return;
 }
 
 void obtener_valor(void){
     valores[0] = valor/100;                            // Obtener centenas del valor
     valores[1] = (valor-valores[0]*100)/10;            // Obtener decenas del valor
     valores[2] = valor-valores[0]*100-valores[1]*10;   // Obtener unidades del valor
 }
 
 void set_display(void){
     i = valores[0];
     display[0] = tabla2[i];      // Display con centenas
     i = valores[1];
     display[1]= tabla[i];       // Display con decenas
     i = valores[2];
     display[2] = tabla[i];      // Display con unidades
 }
 
 void mostrar_valor(void){
     //PORTD = 0;
     switch (banderas){
         case 0:
             PORTC = display[0];
             PORTDbits.RD0 = 1;
             PORTDbits.RD1 = 0;
             PORTDbits.RD2 = 0;
             banderas = 1;
             return;
         case 1:
             PORTC = display[1];
             PORTDbits.RD0 = 0;
             PORTDbits.RD1 = 1;
             PORTDbits.RD2 = 0;
             banderas = 2;
             return;
         case 2: 
             PORTC = display [2];
             PORTDbits.RD0 = 0;
             PORTDbits.RD1 = 0;
             PORTDbits.RD2 = 1;
             banderas = 0;
             return;
         default: 
             PORTC = 0;
             banderas = 0;
             return;
            
     }
 }

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
   
    ANSELH = 0;         // I/O digitales)
    ANSEL = 0b00000111; // AN0, AN1 y AN2 como entrada analógica
    
    
    TRISA = 0b00000111; // AN0, AN1 y AN2 como entrada
    PORTA = 0; 
    
    TRISB = 0;
    PORTB = 0;
    TRISC = 0;
    PORTC = 0;
    TRISD = 0;
    PORTD = 0;
    
    // Configuración reloj interno
    OSCCONbits.IRCF = 0b0110;   // 4MHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuración ADC
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time
    
    // Configuracion interrupciones
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    
    OPTION_REGbits.T0CS = 0;    // TIMR0 como temporizador
    OPTION_REGbits.T0SE = 0;  
    OPTION_REGbits.PSA = 0;     // Prescaler asignado a TMR0
    OPTION_REGbits.PS2 = 0;     // Prescaler = 2
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 0;
    TMR0 = 6;                 // Delay de 0.5 mS
    
}
