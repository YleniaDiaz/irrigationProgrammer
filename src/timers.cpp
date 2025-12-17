#include <Arduino.h>

// Programación del TIMER 1
void timer1_init() {
	//EICRA |= (1 << ISC11) ;
	//EIMSK |= (1 << INT1);
}

// Programación del TIMER 3
void timer3_init(){
	// Reiniciar todo para partir de un estado conocido
	TCCR3A = 0;
	TCCR3B = 0;
	TCNT3 = 0;
	OCR3A = 0;
	OCR3B = 0;
	OCR3C = 0;
	
	// Habilitar la salida OC3C (modo toggle); pin 3 de salida
	// Modo CTC (modo 4), TOP = OCR3A, N = 8
	
						// Modo Toggle en C 01
						// 2 canal A; 2 canal B; 2 canal C; 
	TCCR3A = B00000100; 	// 00 del final modo = 4 = 01 00
	TCCR3B = B00001010;		//Modo N = 8 = 010
						// Modo = 4 = 01
	
	// Falta calcular el TOP
	OCR3A = 9999;   // Definir el TOP par a 5ms
	OCR3C = 9999;  // Interrupción cuando llega al top

	pinMode(3, 1); 		//el pin 3 es de salida (1)
}

void timerPCINT0_init() {
	// Interrupciones Pin Change para Mando Rotatorio (PCINT0..3 están en Puerto B)
    PCICR |= (1 << PCIE0);				// Habilitar grupo PCIE0
    PCMSK0 |= (1 << PCINT4) | (1 << PCINT3) | (1 << PCINT2) | (1 << PCINT1) | (1 << PCINT0);			// B00001111 /  Habilitar pines PB3..0 (Arduino 50..53)

    // INT1 para PCA9555 (Pin 20)
    EICRA &= ~(1 << ISC10);  			// Configurar disparo por FLANCO DE BAJADA (Falling Edge) -> ISC11=1, ISC10=0
    EICRA |= (1 << ISC11);
    EIMSK |= (1 << INT1);						// EIMSK: Habilitar INT1	
}

// Programación del TIMER 1 OVF
void timer1OVF_init(){
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1C = 0;
	TIMSK1 = 0;
	
	// Modo 15 (Fast PWN, TOP =  OCR1A) N=1024
	TCCR1A = B00000011;
	TCCR1B = B00011101;
	TIMSK1 |= (1<<TOIE1); 			// OVERFLOW
	OCR1A = 7812;						// TOP
}
