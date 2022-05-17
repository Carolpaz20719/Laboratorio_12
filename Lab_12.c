/*
 * File:   Lab_12.c
 * Author: Carolina Paz
 *
 * Created on 17 de mayo de 2022, 02:31 PM
 */

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

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 4000000             // Oscilador
#define address_POT 0                  // Constante para el address del potenciometro

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t bandera_write = 0;             // Variable para leer
uint8_t bandera_read = 0;              // Varibale para escribir
uint8_t val_POT = 0;                   // Variable para valor de potenciometro 
uint8_t bandera_sleep = 0;             // Varibale para dormir o despertar

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    
    if(PIR1bits.ADIF){                  // Fue interrupción del ADC
        if(ADCON0bits.CHS == 0){        // Verificamos sea AN0 el canal seleccionado
            val_POT = ADRESH;           // Guardar en ADRESH en val_POT
            PORTC = val_POT;            // Mostrar el val_POT en PORTC
        }
        PIR1bits.ADIF = 0;              // Limpiamos bandera de interrupción
    }
    
    else if (INTCONbits.RBIF){          // Fue interrupcion del Puerto B 
        if (PORTBbits.RB0 == 0){        // Fue RB0 quien activo interrupcion
            if (bandera_sleep ==0){     // Si esta despierto
                bandera_sleep =1; }     // Mandarlo a dormir
        }
        
        if (PORTBbits.RB1 == 0){        // Fue RB1 quien activo interrupcion
            if (bandera_sleep ==1){     // Si esta en modo sleep
                bandera_sleep =0; }     // Depertarlo
        }
        
        if (PORTBbits.RB2 == 0){        // Fue RB2 quien activo interrupcion
            bandera_sleep =0;           // Despertar
            bandera_write = 1; }        // Escribir el valor 
        INTCONbits.RBIF = 0;            // Limpar la bandera de interrupcion
    }
    
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        if(ADCON0bits.GO == 0){                 // No hay proceso de conversion
            ADCON0bits.GO = 1;}                 // Iniciamos proceso de conversión
       
        
        if (bandera_sleep==1){                  // Para bandera_sleep sea 1
            PIE1bits.ADIE =0 ;                  // Desactivar interrupcion del ADC
            SLEEP ();                           // Pasar a sleep
        }
        else if (bandera_sleep ==0){            // Para bandera_sleep sea 0
            PIE1bits.ADIE =1 ;                  // Activar las interrupciones
        }
        
        if(bandera_write == 1){                 // Guardar el valor
            write_EEPROM(address_POT,val_POT);  // Guardar el valor del POT
            __delay_ms(10);                     // delay
            bandera_write = 0;                  // Dejar de escribir
        }
        
        PORTD = read_EEPROM(address_POT);       // El PORTD va estar leyendo constantemente
        PORTE = 0b0001;                         // Enncender RE0
        __delay_ms(250);                        // Por 250ms
        PORTE = 0b0000;                         // Apagar RE0
        __delay_ms(250);                        // Por 250ms
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    
    // Configuración de entradas y salidas
    ANSEL = 0b01;               // AN0 como entrada analógica
    ANSELH = 0;                 // I/O digitales
    
    TRISC = 0;                  // PORTC como salida
    PORTC = 0;                  // Limpiar PORTC
         
    TRISD = 0;                  // PORTD como salida
    PORTD = 0;                  // Limpiar PORTD
    
    TRISE = 0;                  // PORTE como salida
    PORTE = 0;                  // Limpiar PORTE
    
        
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
    __delay_us(50);             // Sample time
    
    //Interrupcion del puerto B
    OPTION_REGbits.nRBPU = 0;   // Habilitamos resistencias de pull-up del PORTB

    WPUBbits.WPUB0  = 1;        // Habilitamos resistencia de pull-up de RB0
    WPUBbits.WPUB1  = 1;        // Resistencia resistencia de pull-up de RB1
    WPUBbits.WPUB2  = 1;        // Resistencia resistencia de pull-up de RB2
    INTCONbits.RBIE = 1;        // Habilitamos interrupciones del PORTB
    
    IOCBbits.IOCB0  = 1;        // Habilitamos interrupción por cambio de estado para RB0
    IOCBbits.IOCB1  = 1;        // Habilitamos interrupción por cambio de estado para RB1
    IOCBbits.IOCB2  = 1;        // Habilitamos interrupción por cambio de estado para RB2
    INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupción
    
    // Configuracion interrupciones
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    
}

// Reer en la EEPROM
uint8_t read_EEPROM(uint8_t address){
    EEADR = address;            // Guardamos el address en EEADR
    EECON1bits.EEPGD = 0;       // Lectura a la EEPROM
    EECON1bits.RD = 1;          // Obtenemos dato de la EEPROM
    return EEDAT;               // Regresamos dato 
}

// Escribir en la EEPROM
void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address;            // Guardamos el address en EEADR
    EEDAT = data;               // Guardar el dato que queremos mandar en EEDAT
    EECON1bits.EEPGD = 0;       // Escritura a la EEPROM
    EECON1bits.WREN = 1;        // Habilitamos escritura en la EEPROM
    
    INTCONbits.GIE = 0;         // Deshabilitamos interrupciones para escribir
    EECON2 = 0x55;              // Registro de control
    EECON2 = 0xAA;              // Registro de control
    
    EECON1bits.WR = 1;          // Iniciamos escritura
    
    EECON1bits.WREN = 0;        // Deshabilitamos escritura en la EEPROM
    INTCONbits.RBIF = 0;        // Limpiar bandera de interrupcion
    INTCONbits.GIE = 1;         // Habilitamos interrupciones
}