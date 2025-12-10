/*  Práctica Lab4: Programador de riego
 * 
 * Descripción: Reloj RTC + programador de riego
 * 
 * Fichero: 		25-26_plab4_priego_base.pdsprj
 * Creado: 		04/12/2025
 * Autor:				Ylenia Díaz Trinidad
*/

// include de ficheros .h y declaración de nombres de funciones
#include <i2c_functions.h>
#include <timers.h>
#include <datos_priego.h>

// definición de macros
#define PRIGHT  30    		// pulsador right
#define PDOWN   31    		// "" down
#define PLEFT   32   			// "" left
#define PENTER 33    		// "" center
#define PUP     34    			// "" up
#define SPEAKER 37    		// speaker

#define D4    0xFE   			// 1111 1110 unidades
#define D3    0xFD    			// 1111 1101 decenas
#define D2    0xFB    			// 1111 1011 centenas
#define D1    0xF7    			// 1111 0111 millares
#define DOFF  0xFF    		// 1111 1111 apagado: todos los cátados comunes a "1"
#define DON   0xF0    		// 1111 0000   todos los cátados comunes a "0"

// PIN MODOS
#define MODO_MANUAL 50
#define MODO_PROGRAMACION 51
#define MODO_VER 52
#define MODO_AUTOMATICO 53

// Definición de las teclas del nuevo teclado
char teclado_map[][3]={ {'1','2','3'},
											{'4','5','6'},
											{'7','8','9'},
											{'*','0','#'}};
String buffer = "";
String dia[ ] = { "", "MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN" }; 	// day 1-7
String mes[ ]={ "", "JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};

// Códigos de 7 segmentos de los caracteres hexadecimales: 0-F
int tabla7seg [ ] = { 63, 06, 91, 79, 102, 109, 125, 39, 127, 103, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71, 0xFF }; // 0,1,2... E,F, todos encendidos

// 	PROGRAMADOR DE RIEGO
String smodo_riego[]={"MAN ", "PROG", "VER ", "AUTO"};
int modo_priego = 0;	// MAN modo de funcionamiento del programador de riego: 0-3
enum modo_priego {MAN=0, PROG, VER, AUTO};

// pulsadores o botones del programador de riego
volatile boolean p_left = true;
volatile boolean p_off = true;
volatile boolean p_on = true;
volatile boolean p_right = true;

// Estructura de datos para la programación semanal del riego
// LR[8][2] para poder usar índices 1-7 directamente (coincidiendo con RTC dayOfWeek)
// índice 0 no se usa para evitar restar (día - 1).
// LR [Día] [Línea] -> Día 1..7, Línea 0..1
linea_riego LR[8] [2];  		// Array de 7 días (de 1 a 8) y 2 líneas -> se puede ampliar para poner sesiones de riego al día LR[8] [2] [3]

void setup() {
	// CANALES SERIE

	// canal tx0/rx0
	Serial.begin(9600);
	while(!Serial);
	Serial.write(12); 	// clear screen
	delay(150);

	// canal tx3/rx3 (pantalla LCD)
	Serial3.begin(9600); //canal 3, 9600 baudios,8 bits, no parity, 1 stop bit
	while(!Serial3);
	Serial3.write(0xFE); Serial3.write(0x01); //Clear Screen
	delay(150);

	// puertos de E/S del turnomatic
	// PORTA: Segmentos a-f
	DDRA=0xFF;    // PORTA de salida
	PORTA=0xFF;    // activamos segmentos a-g
		
	// PORTL[7:4]: filas del teclado
	DDRL=0x0F;    // input;
	PORTL=0xFF;     // pull-up activos, cátodos/columnas teclado desactivadas 
		
	// PORTC: Pulsadores y altavoz
	DDRC=0x01;    //PC7:1 input: PC0: output-speaker
	PORTC= 0xFE;   // pull-up activos menos el speaker que es de salida

	// pines del I2C
	pinMode(LEE_SDA, INPUT);
	pinMode(LEE_SCL, INPUT);
	pinMode(ESC_SDA, OUTPUT);
	pinMode(ESC_SCL, OUTPUT);
	digitalWrite(ESC_SCL, HIGH);
	digitalWrite(ESC_SDA, HIGH); 

	// MANDO ROTATORIO

	// Conexiones de las salidas del switch rotatorio: MAN-PROG-VER-AUTO
	// estos pines se corresponden con PCINT3-PCINT0
	pinMode(MODO_MANUAL, INPUT_PULLUP);							// MAN  PCINT3
	pinMode(MODO_PROGRAMACION, INPUT_PULLUP);			// PROG  PCINT2
	pinMode(MODO_VER, INPUT_PULLUP);									// VER  PCINT1
	pinMode(MODO_AUTOMATICO, INPUT_PULLUP);				// AUTO   PCINT0

	// sincronización del mando rotatorio por interrupciones
	// habilitación de interrupciones PCINT0-3

	// Lectura del switch para definir el modo de funcionamiento del programador: variable modo_priego

	/// EXPANSOR I2C PCA9555
	// declaración del pin para que interrumpa el PCA9555  ->interrupción INT1 (pin 20)
	pinMode(20,INPUT_PULLUP);

	// Configuración interrupción externa INT1

	/// EXPANSOR PCA9555
	initPCA();

	// inicialización estructura de datos (cualquier cosa o leer de EEPROM)
	/*
		for (int j=0; j<7 ;j++){
			for(int i=0; i<2; i++){
				LR[j][i].estado = 2;	// 0:OFF 1: ON  2: hh:mm:ss y T
				LR[j][i].hi.hh = 11+i+j;
				LR[j][i].hi.mm = 12+i+j;
				LR[j][i].hi.ss = 13+i+j;
				LR[j][i].T.mm = 14+i+j;
				LR[j][i].T.ss = 15+i+j;
			}
		}
	*/
	// establecer día de la semana y hora o dejar el que toma por defecto
	
	test_LCD();

	// deshabilitar interrupciones
	cli();
	
    // Timers
    timer1_init(); 				// Timer 1: Refresco LCD (2Hz)
    timer3_init();					// Timer 3: Barrido teclado/display (200Hz)
	timerPCINT0_init(); 	// Timer Mando rotatorio
	timer1OVF_init(); 		// Timer 1 OVF: pantalla LCD
	
	// Habilitar interrupciones
	sei();
}

// Visualización entrelazada display-teclado 4x3 (turnomatic)
ISR(TIMER3_COMPC_vect){
	PORTL = DOFF;		//bloquea el display
	
	if (displayMode == 4) { 	// Modo domótica
		int anguloActual = angulo[servo];
		int anguloAbs = abs(anguloActual);
		
		switch (digit) {
			case 0: 
				PORTA = tabla_7seg[anguloAbs%10];  	//Visualización unidades de los grados
				PORTL  = D4;
				break;
			case 1: 
				PORTA = tabla_7seg[anguloAbs / 10%10]; 	//Visualización decenas de los grados
				PORTL  = D3;
				break;
			case 2: 
				if (anguloActual < 0) PORTA = negativeValue;	// Visualización - si el angulo es negativo
				else PORTA = 0x00;	// Apagar
				PORTL = D2;
				break;
			case 3:
				PORTA = tabla_7seg[servo]; 	// visualizar en millar el servo seleccionado
				PORTL  = D1;
				break;
		}
	} else { 	// Modo turnomatic
		switch (digit) {
			case 0: 
				if (displayMode == 1 ||  displayMode == 2) PORTA = tabla_7seg[count%10];  	//Visualización unidades
				else PORTA = 0x00;  // Apagar
				PORTL  = D4;
				break;
			case 1: 
				if (displayMode == 1  ||  displayMode == 2) PORTA = tabla_7seg[int(count / 10)%10]; 	//Visualización decenas
				else PORTA = 0x00;	// Apagar
				PORTL  = D3;
				break;
			case 2: 
				if (displayMode == 1 ) PORTA = tabla_7seg[int(count / 100)%10];	//Visualización centenas
				else if (displayMode == 3) PORTA = tabla_7seg[count%10];	// visualizar unidades
				else PORTA = 0x00;	// Apagar
				PORTL  = D2;
				break;
			case 3:
				if (displayMode == 3) PORTA = tabla_7seg[int(count / 10)%10]; 	// visualizar decenas
				else PORTA = 0x00;	// Apagar
				PORTL  = D1;
				break;
		}
		teclado(digit);
	}
	
	if (digit == 0 || digit == 1 || digit == 2) digit++;
	else digit = 0;
}

//  ISR del switch rotatorio: PCINT0-1-2-3, pines: 53-52-51-50, Modos: AUTO-VER-PROG-MANUAL
// PORTB[7:0] --> Pines: 13-12-11-10-50-51-52-53 (X-X-X-X-MANUAL-PROG-VER-AUTO)
ISR(PCINT0_vect) {
	if (digitalRead(MODO_MANUAL) == LOW) Serial.println("MODO MANUAL");
	if (digitalRead(MODO_PROGRAMACION) == LOW) Serial.println("MODO PROGAMACION");
	if (digitalRead(MODO_VER) == LOW) Serial.println("MODO VER");
	if (digitalRead(MODO_AUTOMATICO) == LOW) Serial.println("MODO AUTOMATICO");
	
	/*int val = PINB & 0x0F;
	// val = 0000 0111 -> MODO MAN
	// val = 0000 1011 -> MODO PROG
	// val = 0000 1101 -> MODO VER
	// val = 0000 1110 -> MODO AUTO
	switch(val) {
		case 7:				// Modo MAN
			Serial.println("MODO MANUAL");
			break;
		case 11:			// Modo PROG
			Serial.println("MODO PROGRAMACION");
			break;
		case 13:			// Modo VER
			Serial.println("MODO VER");
			break;
		case 14:			// Modo AUTO
			Serial.println("MODO AUTO");
			break;
		default:
			break;
	}*/
}

// INTERRUPCION PANTALLA
ISR(TIMER1_OVF_vect) {
	// setCursor(linea, columna);
	// Hay que tener en cuenta poner 0 cuando la hora/minutos/segundos sean menor que 10
	// Serial.print("D"+String(getDay()) + "         " + String(getHour()) + ":"  + String(getMInutes()) + ":" + String(getSeconds()) + "        " + "MODO");
}

// ISR PCA9555
ISR(INT1_vect){
	// detección de pulsadores de la botonera  y actualización de las variables: p_left, p_off, p_on y p_right

	// leer PORT0
	int val = 0; //getPort0() & 0x0F;
	switch(val) {
		case 7:				// btn RIGHT
			Serial.println("btn RIGHT");
			p_right = 0;
			break;
		case 11:			// btn ON
			Serial.println("btn ON");
			p_on = 0;
			break;
		case 13:			// btn OFF
			Serial.println("btn OFF");
			p_off = 0;
			break;
		case 14:			// btn LEFT
			Serial.println("btn LEFT");
			p_left = 0;
			break;
		default:
			break;
	}
}

byte getPort0() {

}

void test_LCD() {
	// Prueba del la pantalla LCD
	// habilitar canal TX3/RX3, canal de comunicaciones serie con la pantalla LCD (MILFORD 4x20 BKP)
	Serial3.begin(9600); 		//canal 3, 9600 baudios, 8 bits, no parity, 1 stop bit
											
	Serial3.write(0xFE); 
	Serial3.write(0x01); 		//Clear Screen
	delay(100);
  
	Serial3.write(0xFE); 
	Serial3.write(0x00); 		// Cursor Home
	delay(100);                       // posicionarse en línea 1
	Serial3.write("L1: 123 Hola a todos"); 
	delay(1000); // linea 1
  
	Serial3.write(0xFE); 
	Serial3.write(0xC0); 		// posicionarse en linea 2
	Serial3.write("L2: 456 Hola a todas"); 
	delay(1000);  					// linea 2
  
	Serial3.write(0xFE); 
	Serial3.write(0x94); 		// posicionarse en linea 3
	Serial3.write("L3: 01234567890123456"); 
	delay(1000);  					// linea 3
  
	Serial3.write(0xFE); 
	Serial3.write(0xD4); 		// posicionarse en linea 4
	Serial3.write("L4: abcdefghijklmnop"); 
	delay(1000);  					// linea 4	
}

void showMenu() {
	Serial.println("\n--- MENU DE CONFIGURACIÓN DEL PROGRAMADOR DE RIEGO ---");
	Serial.println("1. Ajustar hora");
	Serial.println("2. Ajustar fecha");
	Serial.println("3. Configurar líneas de riego");
	Serial.print("Seleccione una opcion: ");
}

void getDataOption3() {
	//Serial.println("\n");
	day = readInt("Dia [1-7]: ");
	line = readInt("Linea de riego [0-1]: ");
	status = readInt("Estado [0-2]: ");
	startHour = readInt("Hora de inicio [0-23]: ");
	startMinutes = readInt("Minutos de inicio [0-59]: ");
	startSeconds = readInt("Segundos de inicio [0-59]: ");
	durationMinutes = readInt("Tiempo minutos [0-59]: ");
	durationSeconds = readInt("Tiempo segundos [0-59]: ");
}

// Control de válvulas de riego: valv0, valv1
void set_valvula(int valvula, int estado) {

}

// exploración del teclado
void teclado(int col) {
	//leer filas del teclado y detectar si hay pulsación
	int val = PINL >> 4;
	if (val == 15) return;
	while((PINL>>4) != 15) {} //método antirrebote: esperar a que se suelte (dejar de pulsar) la tecla
	switch(val) {
		case 7: 		//fila 0
			setBuffer(0, col);
			break;
		case 11: 	//fila 2
			setBuffer(1, col);
			break;
		case 13: 	//fila 3
			setBuffer(2, col);
			break;
		case 14: 	//fila 4
			readLastKeyboardRow(col);
			break;
	}
}

void setBuffer(int row, int col) {
	buffer = buffer + teclado_map[row][col];
}

void readLastKeyboardRow(int col) {
	if (col == 2) { 	//col = 2  -> #
		count = buffer.toInt();
		buffer="";
		 tone(SPEAKER, 1000, 100);
	} else {
		setBuffer(3, col);
	}
}

void loop() {
	// MILFORD LCD on/off
	Serial3.write(0xFE); 
	Serial3.write(0x08); // Display off
	delay(500);
	Serial3.write(0xFE); 
	Serial3.write(0x0C); // Display on
	delay(500);
}
