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
String dia[] = { "", "MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN" }; 	// day 1-7
String mes[] = { "", "JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};

int day;
int line;
int status;
int startHour;
int startMinutes;
int startSeconds;
int durationMinutes;
int durationSeconds;

// Códigos de 7 segmentos de los caracteres hexadecimales: 0-F
int tabla_7seg [ ] = { 63, 06, 91, 79, 102, 109, 125, 39, 127, 103, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71, 0xFF }; // 0,1,2... E,F, todos encendidos
int negativeValue =  64;

// 	PROGRAMADOR DE RIEGO
String smodo_riego[]={"MAN ", "PROG", "VER ", "AUTO"};
volatile int modo_priego = 0;		// 0:MAN, 1:PROG, 2:VER, 3:AUTO
volatile bool flag_update_lcd = false;
volatile bool flag_botonera = false;
volatile bool flag_mostrar_menu = true;
enum modo_priego { MAN=0, PROG, VER, AUTO };

// pulsadores o botones del programador de riego
volatile boolean p_left = true;
volatile boolean p_off = true;
volatile boolean p_on = true;
volatile boolean p_right = true;

// Estado de válvulas (Bit 0: LR0, Bit 1: LR1)
byte estado_valvulas = 0;

// Estructura de datos para la programación semanal del riego
// LR[8][2] para poder usar índices 1-7 directamente (coincidiendo con RTC dayOfWeek)
// índice 0 no se usa para evitar restar (día - 1).
// LR [Día] [Línea] -> Día 1..7, Línea 0..1
linea_riego LR[8] [2];  		// Array de 7 días (de 1 a 8) y 2 líneas -> se puede ampliar para poner sesiones de riego al día LR[8] [2] [3]

// Variables turnomatic
volatile int count = 0;
volatile int digit = 0;
int increment = 1; 
String buffer = "";
int displayMode = 1;

// modo domótica: variables
int servo = 0;			// hasta tres servomotores: 0,1 y 2
int angulo[ ] = { 0, 0, 0 };		// angulo de cada motor --> angulo[servomotor]
const int anguloIncrement = 5;
const int anguloMax = 90;
const int anguloMin = -90;

// Definición de funciones
void teclado(int col);
void setBuffer(int row, int col);
void readLastKeyboardRow(int col);
int readInt (String prompt);

// Función para leer el mando
void leer_mando_rotatorio() {
    byte portVal = PINB; // Pines 50-53 (PB3-PB0)

    if ((portVal & (1 << 3)) == 0) modo_priego = 0;      // MAN
    else if ((portVal & (1 << 2)) == 0) modo_priego = 1; // PROG
    else if ((portVal & (1 << 1)) == 0) modo_priego = 2; // VER
    else if ((portVal & (1 << 0)) == 0) modo_priego = 3; // AUTO
    
    // Si cambiamos a PROG, forzamos que se vuelva a pintar el menú
    if (modo_priego == 1) flag_mostrar_menu = true;
}

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
	Serial3.write(0xFE); 
	Serial3.write(0x01); //Clear Screen
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
	
	//test_LCD();

	// deshabilitar interrupciones
	cli();
	
    // Timers
    timer1_init(); 				// Timer 1: Refresco LCD (2Hz)
    timer3_init();				// Timer 3: Barrido teclado/display (200Hz)
	timerPCINT0_init(); 		// Timer Mando rotatorio
	timer1OVF_init(); 			// Timer 1 OVF: pantalla LCD

	leer_mando_rotatorio();
	
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
	leer_mando_rotatorio();

    // Si salimos de modo Manual, apagar válvulas inmediatamente
    if (modo_priego != 0) {
        estado_valvulas = 0;
    }

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
	// Leer fecha y hora actual del RTC
    byte* tiempo = getRTC_DateTime(); // [seg, min, hour, dWeek, dMonth, month, year]
    
    // LINEA 1: DÍA - HORA - MODO
    Serial3.write(0xFE); Serial3.write(0x80); // Ir al principio L1
    
    // Día semana
    Serial3.print("D"); Serial3.print(tiempo[3]); Serial3.print(" ");
    
    // Hora HH:MM:SS
    if(tiempo[2]<10) Serial3.print("0"); Serial3.print(tiempo[2]); Serial3.print(":");
    if(tiempo[1]<10) Serial3.print("0"); Serial3.print(tiempo[1]); Serial3.print(":");
    if(tiempo[0]<10) Serial3.print("0"); Serial3.print(tiempo[0]);
    
    // Modo (Derecha)
    Serial3.print("   "); // Espaciado
    switch(modo_priego) {
        case 0: 
			Serial3.print("MAN ");
			break;
        case 1: 
			Serial3.print("PROG");
			break;
        case 2:
			Serial3.print("VER ");
			break;
        case 3:
			Serial3.print("AUTO");
			break;
    }

    // LINEAS 2 y 3: ESTADO DE RIEGO    
	// R0 (Línea 3 del LCD - dirección 0x94)
    Serial3.write(0xFE); Serial3.write(0x94);
    Serial3.print("R0: ");
    
    // R1 (Línea 4 del LCD - dirección 0xD4)
    Serial3.write(0xFE);
	Serial3.write(0xD4); 		// Preparamos pero escribimos el texto luego para mantener orden visual
    String lineaR1 = "R1: "; 	// Buffer temporal para R1

    // Variables para el estado a mostrar
    int est0, est1;
    
    if (modo_priego == 0) { 
        // En MODO MANUAL, mostramos el estado actual real de las válvulas
        // estado_valvulas bit 0 es R0, bit 1 es R1
        est0 = (estado_valvulas & 0x01) ? 1 : 0;
        est1 = (estado_valvulas & 0x02) ? 1 : 0;
    } else {
        // En otros modos, mostrar lo programado para hoy
        est0 = LR[tiempo[3]][0].estado;
        est1 = LR[tiempo[3]][1].estado;
    }

    // MOSTRAR R0
    Serial3.write(0xFE); 
	Serial3.write(0x94); // Mover cursor a L3
    Serial3.print("R0: ");
    if (est0 == 0) Serial3.print("OFF     ");
    else if (est0 == 1) Serial3.print("ON      ");
    else if (modo_priego != 0) { // Solo mostrar horas en Auto/Ver/Prog
        struct hora h = LR[tiempo[3]][0].hi;
        if(h.hh<10) Serial3.print("0"); Serial3.print(h.hh); Serial3.print(":");
        if(h.mm<10) Serial3.print("0"); Serial3.print(h.mm); Serial3.print(":");
        if(h.ss<10) Serial3.print("0"); Serial3.print(h.ss);
    }

    // MOSTRAR R1
	Serial3.write(0xFE); 
	Serial3.write(0xD4); // Mover cursor a L4
    Serial3.print("R1: ");
    if (est1 == 0) Serial3.print("OFF     ");
    else if (est1 == 1) Serial3.print("ON      ");
    else if (modo_priego != 0) {
        struct hora h = LR[tiempo[3]][1].hi;
        if(h.hh<10) Serial3.print("0"); Serial3.print(h.hh); Serial3.print(":");
        if(h.mm<10) Serial3.print("0"); Serial3.print(h.mm); Serial3.print(":");
        if(h.ss<10) Serial3.print("0"); Serial3.print(h.ss);
    }
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

void showMenu() {
	Serial.println("\n--- MENU DE CONFIGURACIÓN DEL PROGRAMADOR DE RIEGO ---");
	Serial.println("1. Ajustar hora");
	Serial.println("2. Ajustar fecha");
	Serial.println("3. Configurar líneas de riego");
	Serial.print("Seleccione una opcion: ");
}

void getDataOption3() {
	// Leer datos en terminal
    Serial.println(F("\nConfiguracion Riego:"));
    day = readInt("Dia [1-7]: ");
    line = readInt("Linea de riego [0-1]: ");
    status = readInt("Estado [0:OFF, 1:ON, 2:AUTO]: ");
    
    // Guardar en la estructura
    // El array LR es LR[8][2]. Usamos day tal cual (1-7).
    if(day >= 1 && day <= 7 && line >= 0 && line <= 1) {
        LR[day][line].estado = status;
        
		// Solo pedir horas si es modo AUTO
        if (status == 2) {
            startHour = readInt("Hora inicio [0-23]: ");
            startMinutes = readInt("Minutos inicio [0-59]: ");
            startSeconds = readInt("Segundos inicio [0-59]: ");
            durationMinutes = readInt("Duracion minutos: ");
            durationSeconds = readInt("Duracion segundos: ");
            
            LR[day][line].hi.hh = startHour;
            LR[day][line].hi.mm = startMinutes;
            LR[day][line].hi.ss = startSeconds;
            LR[day][line].T.mm = durationMinutes;
            LR[day][line].T.ss = durationSeconds;
        }
        Serial.println("Datos guardados.");
    } else {
        Serial.println("Error: Dia o Linea incorrectos.");
    }
}

// Leer entero introducido
int readInt (String prompt) {
	String inputString = ""; 
	bool inputDone = false;
  
	  Serial.print(prompt);		// imprimir mensaje 

	  while (!inputDone) {
		if (Serial.available() > 0) {
		  char c = Serial.read();
		  if (isDigit(c) || (c == '-' && inputString.length() == 0)) {	 // Comprobar si es dígito o signo menos (solo al principio)
			inputString += c;
			Serial.print(c); 		// imprimir carácter introducido 
		  } else if ((c == '\b' || c == 127) && inputString.length() > 0) {  	// Comprobar si es backspace (ASCII 8 o 127)
			inputString.remove(inputString.length() - 1); 	// Borrar último carácter
			Serial.print("\b \b"); 		// mover cursor atrás
		  } else if (c == '\r') {  	// Comprobar enter
			inputDone = true;
			Serial.println();
		  }
		}
		delay(10); 
	}
  
	return inputString.toInt();	 // string a entero
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
	// Máquina de Estados Principal 
    switch(modo_priego) {
        //MODO MANUAL
        case 0: 
            if (flag_botonera) {
                flag_botonera = false;
                byte botones = getPCA_Input(); // Leer estado (Activo bajo)

                // Botón ON (Bit 2) enciende, OFF (Bit 1) apaga
                // Depende de qué línea esté seleccionada.
                // Si pulsa ON -> Abre Válvula 0. Si pulsa OFF -> Cierra V0.

                if (!(botones & 0x04)) { // Botón ON presionado
                    estado_valvulas |= 1; // Encender LR0
                }
                if (!(botones & 0x02)) { // Botón OFF presionado
                    estado_valvulas &= ~1; // Apagar LR0
                }
                setPCA_Output(estado_valvulas);
            }
            break;

        //MODO PROGRAMACIÓN
        case 1:
            if (flag_mostrar_menu) {
                showMenu();
                flag_mostrar_menu = false;
                // Limpiar buffer serie de basura anterior
                while(Serial.available() > 0) Serial.read();
            }

            if (Serial.available() > 0) {
                int op = readInt("");
                if (op == 3) getDataOption3();
                // Volver a mostrar menú al terminar operación
                flag_mostrar_menu = true; 
            }
            break;

        //MODO VER
        case 2:
            // visualización la maneja el Timer1 en el LCD
            break;

        //MODO AUTO
        case 3:
            // Obtener tiempo actual
            byte* t = getRTC_DateTime();
            // Convertir hora actual a segundos totales del día para facilitar comparación
            long segundosActuales = (long)t[2]*3600 + (long)t[1]*60 + (long)t[0];
            int diaHoy = t[3]; // 1-7

            byte nuevoEstadoValvulas = 0;

            // Comprobar Línea 0 y Línea 1
            for (int i=0; i<2; i++) {
                int estadoConf = LR[diaHoy][i].estado;
                
                if (estadoConf == 1) { 
                    // Si está forzado a ON todo el día
                    nuevoEstadoValvulas |= (1 << i); 
                } else if (estadoConf == 2) {
                    // Modo programado por hora
                    long inicioSeg = (long)LR[diaHoy][i].hi.hh*3600 + 
                                    (long)LR[diaHoy][i].hi.mm*60 + 
                                    (long)LR[diaHoy][i].hi.ss;

                    long duracionSeg = (long)LR[diaHoy][i].T.mm*60 + 
                                       (long)LR[diaHoy][i].T.ss;

                    long finSeg = inicioSeg + duracionSeg;

                    // Comprobar si estamos dentro del intervalo
                    if (segundosActuales >= inicioSeg && segundosActuales < finSeg) {
                        nuevoEstadoValvulas |= (1 << i); // Activar bit i
                    }
                }
            }
            
            // Actuar en hardware
            estado_valvulas = nuevoEstadoValvulas;
            setPCA_Output(estado_valvulas);
            break;
    }
}
