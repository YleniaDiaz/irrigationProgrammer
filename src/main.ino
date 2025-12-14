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
String dia[ ] = { "", "MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN" }; 	// day 1-7
String mes[ ]={ "", "JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};

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
enum modo_priego { MAN=0, PROG, VER, AUTO };

// pulsadores o botones del programador de riego
volatile boolean p_left = true;
volatile boolean p_off = true;
volatile boolean p_on = true;
volatile boolean p_right = true;

byte estado_valvulas = 0;				// Estado de válvulas (Bit 0: LR0, Bit 1: LR1)
volatile int linea_seleccionada = 0;	// seleccionar linea en manual (0 o 1)
volatile int view_day = 1; 				// Controla qué día se visualiza en el LCD

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
void printTwoDigits(int number);

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

	// Leer estado inicial del mando
    if (digitalRead(MODO_MANUAL) == LOW) modo_priego = MAN;
    else if (digitalRead(MODO_PROGRAMACION) == LOW) modo_priego = PROG;
    else if (digitalRead(MODO_VER) == LOW) modo_priego = VER;
    else if (digitalRead(MODO_AUTOMATICO) == LOW) modo_priego = AUTO;

	// sincronización del mando rotatorio por interrupciones
	// habilitación de interrupciones PCINT0-3

	// Lectura del switch para definir el modo de funcionamiento del programador: variable modo_priego

	/// EXPANSOR I2C PCA9555
	// declaración del pin para que interrumpa el PCA9555  ->interrupción INT1 (pin 20)
	pinMode(20,INPUT_PULLUP);

	// Configuración interrupción externa INT1

	/// EXPANSOR PCA9555
	initPCA();

    setPCA_Output(0);	// válvulas apagadas al inicio

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
	if (digitalRead(MODO_MANUAL) == LOW) { 
		Serial.println("MODO MANUAL");
		modo_priego = MAN;
	}
	if (digitalRead(MODO_PROGRAMACION) == LOW) { 
		Serial.println("MODO PROG");
		modo_priego = PROG;
	}
	if (digitalRead(MODO_VER) == LOW) { 
		Serial.println("MODO VER");
		modo_priego = VER;
	}
	if (digitalRead(MODO_AUTOMATICO) == LOW) { 
		Serial.println("MODO AUTO");
		modo_priego = AUTO;
	}

	// Limpiar Terminal Virtual al cambiar de modo
    Serial.write(12);
	
	// Si salimos de Manual, apagar válvulas por seguridad
    if (modo_priego != MAN) {
        estado_valvulas = 0;
        setPCA_Output(0);
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
    Serial3.write(0xFE); 
    Serial3.write(0x80); // Ir al inicio de Linea 1
    
    // Imprimir Día que se está visualizando (view_day)
    Serial3.print("D");
    Serial3.print(view_day); 
    
    // Espaciado y Hora Actual
    Serial3.print("  "); 
    printTwoDigits(current_hour); Serial3.print(":");
    printTwoDigits(current_min);  Serial3.print(":");
    printTwoDigits(current_sec);
    
    // Espaciado y Modo
    Serial3.print("   ");
    Serial3.print(smodo_riego[modo_priego]);

    // LINEAS 3 y 4: INFORMACIÓN DE RIEGO
    
    // R0 (i=0) y R1 (i=1)
    for(int i=0; i<2; i++) {
        // Posicionar cursor: Linea 3 (0x94) o Linea 4 (0xD4)
        Serial3.write(0xFE); 
        Serial3.write( (i==0) ? 0x94 : 0xD4 );
        
        Serial3.print("R"); Serial3.print(i); Serial3.print(": ");

        // Lógica de visualización según modo
        if (modo_priego == PROG || modo_priego == VER) {
            // MODO PROGRAMACIÓN / VER: Mostrar datos de la memoria (LR)
			int estado_memoria = LR[view_day][i].estado;
            
            if (estado_memoria == 0) {
                Serial3.print("OFF            "); // Espacios para borrar texto anterior
            } 
            else if (estado_memoria == 1) {
                Serial3.print("ON             ");
            } 
            else if (estado_memoria == 2) {
                printTwoDigits(LR[view_day][i].hi.hh); Serial3.print(":");
                printTwoDigits(LR[view_day][i].hi.mm); Serial3.print(":");
                printTwoDigits(LR[view_day][i].hi.ss); 
                Serial3.print("  "); // Espacio separador
                printTwoDigits(LR[view_day][i].T.mm); Serial3.print(":");
                printTwoDigits(LR[view_day][i].T.ss);
            }
        } 
        else {
            // MODO MANUAL / AUTO: Mostrar estado válvulas
            if (estado_valvulas & (1 << i)) Serial3.print("ON            "); 
            else Serial3.print("OFF           ");
        }
    }

    // CURSOR (Solo parpadea en MANUAL)
    if (modo_priego == MAN) {
        Serial3.write(0xFE);
        if (linea_seleccionada == 0) Serial3.write(0x94 + 4); // Linea 3, pos 4
        else Serial3.write(0xD4 + 4); // Linea 4, pos 4
        
        Serial3.write(0xFE); Serial3.write(0x0D); // Blink ON
    } else {
        Serial3.write(0xFE); Serial3.write(0x0C); // Cursor OFF
    }
}

// ISR PCA9555
ISR(INT1_vect){
    // No I2C aquí para evitar bloqueos
    flag_botonera = true;
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

int readInt(String prompt) {
    String inputString = "";
    bool inputDone = false;
    
    // Imprimir el mensaje si no está vacío
    if (prompt != "") Serial.print(prompt);
    
    // Limpiar buffer previo
    while(Serial.available()) Serial.read();

    while (!inputDone) {
        // Verificar si salimos del modo PROG mientras esperamos input
        if (modo_priego != PROG) return -1; 

        if (Serial.available() > 0) {
            char c = Serial.read();
            if (isDigit(c)) {
                inputString += c;
                Serial.print(c);
            } else if (c == '\r' || c == '\n') {
                inputDone = true;
                Serial.println();
            }
        }
    }
    return inputString.toInt();
}

void opcion_ajustar_hora() {
    Serial.println("\n--- AJUSTAR HORA ---");
    int h = readInt("Hora (0-23): ");
    if(h == -1) return;
    int m = readInt("Minutos (0-59): ");
    int s = readInt("Segundos (0-59): ");
    
    setTimeRTC(h, m, s);
    Serial.println("Hora actualizada.");
}

void opcion_ajustar_fecha() {
    Serial.println("\n--- AJUSTAR FECHA ---");
    int dw = readInt("Dia Semana (1=Lun ... 7=Dom): ");
    if(dw == -1) return;
    int d = readInt("Dia Mes (1-31): ");
    int m = readInt("Mes (1-12): ");
    int y = readInt("Anio (0-99): ");
    
    setDateRTC(d, m, y, dw);
    Serial.println("Fecha actualizada.");
}

void opcion_riego() {
    Serial.println("\n--- CONFIGURAR RIEGO ---");
    
    //Pedir datos básicos
    int d = readInt("Dia (1-7): ");
    if (d < 1 || d > 7) { 
		Serial.println("Dia incorrecto");
		return;
	}
    
    int l = readInt("Linea (0-1): ");
    if (l < 0 || l > 1) {
		Serial.println("Linea incorrecta");
		return;
	}
    
    int st = readInt("Estado (0:OFF, 1:ON, 2:PROG): ");
    if (st < 0 || st > 2) { 
		Serial.println("Estado incorrecto");
		return;
	}
    
    // Variables temporales para la hora (no guardamos en LR todavía)
    int t_hh = 0, t_mm = 0, t_ss = 0;
    int dur_mm = 0, dur_ss = 0;

    // Si es programación (Estado 2), pedir hora y duración
    if (st == 2) {
        Serial.println("Configuracion Horaria:");
        t_hh = readInt("Hora inicio (0-23): ");
        t_mm = readInt("Minuto inicio (0-59): ");
        t_ss = readInt("Segundo inicio (0-59): ");
        
        dur_mm = readInt("Duracion Minutos: ");
        dur_ss = readInt("Duracion Segundos: ");
    }

    // Guardamos estado
    LR[d][l].estado = st;
    
    // Guardamos tiempos (si es estado 2 se guardan los datos, si no, se guardan ceros)
    if (st == 2) {
        LR[d][l].hi.hh = t_hh;
        LR[d][l].hi.mm = t_mm;
        LR[d][l].hi.ss = t_ss;
        LR[d][l].T.mm = dur_mm;
        LR[d][l].T.ss = dur_ss;
    } else {
        // Limpiar datos antiguos si pasamos a OFF u ON manual
        LR[d][l].hi.hh = 0; LR[d][l].hi.mm = 0; LR[d][l].hi.ss = 0;
        LR[d][l].T.mm = 0; LR[d][l].T.ss = 0;
    }
    
    Serial.print("Programacion Guardada para el Dia "); 
	Serial.println(d);
}

void showMenu() {
    Serial.write(12); // Clear terminal
    Serial.println("\n*** MENU PROGRAMADOR DE RIEGO ***");
    Serial.println("1. Ajustar Hora");
    Serial.println("2. Ajustar Fecha");
    Serial.println("3. Configurar Lineas de Riego");
    Serial.println("---------------------------------");
	Serial.print("Seleccione opcion: ");
}

// Control de válvulas de riego: valv0, valv1
void set_valvula(int valvula, int estado) {
	if (estado == 1) estado_valvulas |= (1 << valvula);
    else estado_valvulas &= ~(1 << valvula);
    setPCA_Output(estado_valvulas);
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

void printTwoDigits(int number) {
  if (number < 10) {
    Serial3.print("0");
  }
  Serial3.print(number);
}

void loop() {
	readRTC(); 		// Leer la hora

	// Que día se ve en pantalla
    if (modo_priego != VER) view_day = current_day;

	static bool menu_mostrado = false;

	// Máquina de Estados Principal 
    switch(modo_priego) {
        //MODO MANUAL
        case 0: 
            if (flag_botonera) {
                flag_botonera = false;
                
                byte botones = getPCA_Input(); 		// Leer estado botones por I2C
                // Bit 0: LEFT, Bit 1: OFF, Bit 2: ON, Bit 3: RIGHT
                if (!(botones & 0x01)) { 		// LEFT pulsado
                    linea_seleccionada = 0; 	// Seleccionar R0
                    Serial.println("Select R0");
                }
                if (!(botones & 0x08)) { 		// RIGHT pulsado
                    linea_seleccionada = 1; 	// Seleccionar R1
                    Serial.println("Select R1");
                }
                
                if (!(botones & 0x04)) { 		// ON pulsado
                    Serial.println("Válvula ON");
                    set_valvula(linea_seleccionada, 1);
                }
                if (!(botones & 0x02)) { 		// OFF pulsado
                    Serial.println("Válvula OFF");
                    set_valvula(linea_seleccionada, 0);
                }
            }
			break;

		//MODO PROGRAMACIÓN
        case 1:
            if (!menu_mostrado) {
                showMenu();
                menu_mostrado = true;
            }

            if (Serial.available() > 0) {
                int op = readInt("");
                
                switch(op) {
                    case 1: opcion_ajustar_hora(); break;
                    case 2: opcion_ajustar_fecha(); break;
                    case 3: opcion_riego(); break;
                    default: Serial.println("Opcion no valida"); break;
                }
                delay(1000); 
                menu_mostrado = false; 		// volver a pintar menú
            }
            break;

        //MODO VER
        case 2:
            // visualización la maneja el Timer1 en el LCD
            break;

        //MODO AUTO
        case 3:
            // Comparar hora actual RTC con LR[dia][linea].hi
            // Si coincide -> estado_valvulas = ON
            // Si pasa el tiempo de duración -> estado_valvulas = OFF
            setPCA_Output(estado_valvulas);
            break;
    }
}
