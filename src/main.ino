/*  Práctica Lab4: Programador de riego
 * 
 * Descripción: Reloj RTC + programador de riego
 * 
 * Fichero: 		25-26_plab4_priego_ydt.pdsprj
 * Creado: 			04/12/2025
 * Autor:			Ylenia Díaz Trinidad
*/

// include de ficheros .h y declaración de nombres de funciones
#include <i2c_functions.h>
#include <timers.h>
#include <datos_priego.h>

// definición de macros
#define PRIGHT  30    			// pulsador right
#define PDOWN   31    			// "" down
#define PLEFT   32   				// "" left
#define PENTER 33    			// "" center
#define PUP     34    				// "" up
#define SPEAKER 37    			// speaker

#define D4    0xFE   				// 1111 1110 unidades
#define D3    0xFD    				// 1111 1101 decenas
#define D2    0xFB    				// 1111 1011 centenas
#define D1    0xF7    				// 1111 0111 millares
#define DOFF  0xFF    			// 1111 1111 apagado: todos los cátados comunes a "1"
#define DON   0xF0    			// 1111 0000   todos los cátados comunes a "0"

// PIN MODOS
#define MODO_MANUAL 50
#define MODO_PROGRAMACION 51
#define MODO_VER 52
#define MODO_AUTOMATICO 53
#define MODO_AUTOPLUS 10        // Pin 10 (PB4)

//EEPROM 24LC64)
#define EEPROM_ADDR  0     						// Dirección para saber si ya está configurado
#define EEPROM_VAL   0x55  						// Valor indicar memoria válida
#define EEPROM_START_ADDR  10   		// Dirección donde empiezan los datos de riego

// Definición de las teclas del nuevo teclado
char teclado_map[][3]={ {'1','2','3'},
											{'4','5','6'},
											{'7','8','9'},
											{'*','0','#'}};
String dia[ ] = { "", "MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN" }; 	// day 1-7
String mes[ ]= { "", "JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};

// Códigos de 7 segmentos de los caracteres hexadecimales: 0-F
int tabla_7seg [ ] = { 63, 06, 91, 79, 102, 109, 125, 39, 127, 103, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71, 0xFF }; // 0,1,2... E,F, todos encendidos
int negativeValue =  64;

// 	PROGRAMADOR DE RIEGO
String smodo_riego[]={"MAN ", "PROG", "VER ", "AUTO", "AUTO+" };
volatile int modo_priego = 0;		// 0:MAN, 1:PROG, 2:VER, 3:AUTO
volatile bool flag_update_lcd = false;
volatile bool flag_botonera = false;
volatile bool menu_mostrado = false;
volatile int autoplus = 25;         // Valor inicial 25%
enum modo_priego { MAN=0, PROG, VER, AUTO, AUTOPLUS };

// pulsadores o botones del programador de riego
volatile boolean p_left = true;
volatile boolean p_off = true;
volatile boolean p_on = true;
volatile boolean p_right = true;

byte estado_valvulas = 0;								// Estado de válvulas (Bit 0: LR0, Bit 1: LR1)
volatile int linea_seleccionada = 0;				// seleccionar linea en modo manual (0 o 1)
volatile int view_day = 1; 								// Controla que día se visualiza en el LCD

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
int servo = 0;										// hasta tres servomotores: 0,1 y 2
int angulo[ ] = { 0, 0, 0 };					// angulo de cada motor --> angulo[servomotor]
const int anguloIncrement = 5;
const int anguloMax = 90;
const int anguloMin = -90;

// Definición de funciones
void teclado(int col);
void setBuffer(int row, int col);
void readLastKeyboardRow(int col);
int readInt (String prompt);
void printTwoDigits(int number);
void guardar_programacion();
void cargar_programacion();
void reset_programacion();

void setup() {
	// CANALES SERIE

	// canal tx0/rx0
	Serial.begin(9600);
	while(!Serial);
	Serial.write(12); 	// clear screen
	delay(150);

	// canal tx3/rx3 (pantalla LCD)
	Serial3.begin(9600);			 //canal 3, 9600 baudios,8 bits, no parity, 1 stop bit
	while(!Serial3);
	Serial3.write(0xFE); 
	Serial3.write(0x01); 			//Clear Screen
	delay(150);

	// puertos de E/S del turnomatic
	// PORTA: Segmentos a-f
	DDRA=0xFF;   		 // PORTA de salida
	PORTA=0xFF;    	// activamos segmentos a-g
		
	// PORTL[7:4]: filas del teclado
	DDRL=0x0F;  		// input;
	PORTL=0xFF;     	// pull-up activos, cátodos/columnas teclado desactivadas 
		
	// PORTC: Pulsadores y altavoz
	DDRC=0x01;    	//PC7:1 input: PC0: output-speaker
	PORTC= 0xFE;   	// pull-up activos menos el speaker que es de salida

	// pines del I2C
	pinMode(LEE_SDA, INPUT);
	pinMode(LEE_SCL, INPUT);
	pinMode(ESC_SDA, OUTPUT);
	pinMode(ESC_SCL, OUTPUT);
	digitalWrite(ESC_SCL, HIGH);
	digitalWrite(ESC_SDA, HIGH); 

	// MANDO ROTATORIO

	// Conexiones de las salidas del switch rotatorio: MAN-PROG-VER-AUTO
	pinMode(MODO_MANUAL, INPUT_PULLUP);							// MAN  PCINT3
	pinMode(MODO_PROGRAMACION, INPUT_PULLUP);			// PROG  PCINT2
	pinMode(MODO_VER, INPUT_PULLUP);									// VER  PCINT1
	pinMode(MODO_AUTOMATICO, INPUT_PULLUP);				// AUTO   PCINT0
    pinMode(MODO_AUTOPLUS, INPUT_PULLUP);

	// Leer estado inicial del mando
    if (digitalRead(MODO_MANUAL) == LOW) modo_priego = MAN;
    else if (digitalRead(MODO_PROGRAMACION) == LOW) modo_priego = PROG;
    else if (digitalRead(MODO_VER) == LOW) modo_priego = VER;
    else if (digitalRead(MODO_AUTOMATICO) == LOW) modo_priego = AUTO;
    else if (digitalRead(MODO_AUTOPLUS) == LOW) modo_priego = AUTOPLUS;

	/// EXPANSOR I2C PCA9555
	// declaración del pin para que interrumpa el PCA9555  ->interrupción INT1 (pin 20)
	pinMode(20,INPUT_PULLUP);

	/// EXPANSOR PCA9555
	initPCA();
    setPCA_Output(0);	// válvulas apagadas al inicio

    // GESTIÓN MEMORIA
    // Comprobar si la memoria ya tiene datos válidos
    byte check = randomRead(EEPROM_ADDR);
    if (check == EEPROM_VAL) {
        cargar_programacion(); 		// Recuperar datos
    } else {
        Serial.println("Memoria vacia. Inicializando...");
        reset_programacion();  		// Poner todo a cero y guardar
    }

	cli();		// deshabilitar interrupciones
	
    // Timers
    timer1_init(); 				// Timer 1: Refresco LCD (2Hz)
    timer3_init();					// Timer 3: Barrido teclado/display (200Hz)
	timerPCINT0_init(); 	// Timer Mando rotatorio
	timer1OVF_init(); 		// Timer 1 OVF: pantalla LCD
	
	sei();		// Habilitar interrupciones
}

// Visualización entrelazada display-teclado 4x3 (turnomatic)
ISR(TIMER3_COMPC_vect){
	PORTL = DOFF;		//bloquea el display

    // Si estamos en modo AUTO+, forzamos visualizar variable autoplus 
    // Usamos la variable 'count' temporalmente o una lógica paralela.
    // Si es AUTOPLUS, sobrescribimos lo que se iba a mostrar:
    
    int valor_a_mostrar = count; // Por defecto
    if (modo_priego == AUTOPLUS) valor_a_mostrar = autoplus; // Mostramos porcentaje
	
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
				if (displayMode == 1 ||  displayMode == 2) PORTA = tabla_7seg[valor_a_mostrar%10];  	//Visualización unidades
				else PORTA = 0x00;  // Apagar
				PORTL  = D4;
				break;
			case 1: 
				if (displayMode == 1  ||  displayMode == 2) PORTA = tabla_7seg[int(valor_a_mostrar / 10)%10]; 	//Visualización decenas
				else PORTA = 0x00;	// Apagar
				PORTL  = D3;
				break;
			case 2: 
				if (displayMode == 1 ) PORTA = tabla_7seg[int(valor_a_mostrar / 100)%10];	//Visualización centenas
				else if (displayMode == 3) PORTA = tabla_7seg[valor_a_mostrar%10];	// visualizar unidades
				else PORTA = 0x00;	// Apagar
				PORTL  = D2;
				break;
			case 3:
				if (displayMode == 3) PORTA = tabla_7seg[int(valor_a_mostrar / 10)%10]; 	// visualizar decenas
				else PORTA = 0x00;	// Apagar
				PORTL  = D1;
				break;
		}
		teclado(digit);
	}
	
	if (digit == 0 || digit == 1 || digit == 2) digit++;
	else digit = 0;
}

//ISR del switch rotatorio: PCINT0-1-2-3, pines: 53-52-51-50, Modos: AUTO-VER-PROG-MANUAL
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
    if (digitalRead(MODO_AUTOPLUS) == LOW) {
        Serial.println("MODO AUTO+");
        modo_priego = AUTOPLUS;
    }
	
    Serial.write(12);							// Limpiar Terminal Virtual al cambiar de modo
	menu_mostrado = false; 			// Forzar pintar menú
	view_day = current_day; 			// Al cambiar de modo volvemos al de hoy
	
	// Si salimos de Manual, apagar válvulas por seguridad
    if (modo_priego != MAN) {
        estado_valvulas = 0;
        setPCA_Output(0);
    }
}

// INTERRUPCION PANTALLA
ISR(TIMER1_OVF_vect) {
    int display_d = (modo_priego == VER) ? view_day : current_day;			 // día a mostrar

    // LÍNEA 1: CABECERA
    Serial3.write(0xFE); 
	Serial3.write(0x80);
    Serial3.print("D"); Serial3.print(display_d); Serial3.print("  "); 
    printTwoDigits(current_hour); Serial3.print(":");
    printTwoDigits(current_min); Serial3.print(":");
    printTwoDigits(current_sec); Serial3.print("   ");
    Serial3.print(smodo_riego[modo_priego]); 

    // LÍNEAS 3 y 4: INFORMACIÓN DE RIEGO 
    for(int i=0; i<2; i++) {
        // Posicionar cursor: Linea 3 (R0) o Linea 4 (R1)
        Serial3.write(0xFE); Serial3.write( (i==0) ? 0x94 : 0xD4 );
        Serial3.print("R"); Serial3.print(i); Serial3.print(": ");

        // Si NO modo MANUAL, mostrar programación guardada
        if (modo_priego != MAN) {
            // Leemos del array LR usando el día a visualizar
            int estado_guardado = LR[display_d][i].estado;
            
            if (estado_guardado == 0) {
                Serial3.print("OFF                  "); 
            } else if (estado_guardado == 1) {
                Serial3.print("ON                   ");
            } else if (estado_guardado == 2) {
                if (modo_priego == AUTOPLUS) {
                    //Cálculo: (Duración + Porcentaje) en segundos
                    long duracion_base = (LR[display_d][i].T.mm * 60L) + LR[display_d][i].T.ss;
                    long duracion_plus = duracion_base + (duracion_base * autoplus / 100);
                    
                    // Visualizar solo segundos con sufijo 's'
                    Serial3.print(duracion_plus);
                    Serial3.print("s                 "); // Espacios para limpiar
                } else {
                    // Formato: HH:MM:SS  MM:SS
                    printTwoDigits(LR[display_d][i].hi.hh); Serial3.print(":");
                    printTwoDigits(LR[display_d][i].hi.mm); Serial3.print(":");
                    printTwoDigits(LR[display_d][i].hi.ss); 
                    Serial3.print("  ");
                    printTwoDigits(LR[display_d][i].T.mm); Serial3.print(":");
                    printTwoDigits(LR[display_d][i].T.ss);
                }
            }
        } else {
            // SOLO EN MANUAL: Mostrar estado "forzado" de las válvulas
            if (estado_valvulas & (1 << i)) Serial3.print("ON              "); 
            else Serial3.print("OFF             ");
        } 
    }

    // GESTIÓN DEL CURSOR (Modo Manual)
    if (modo_priego == MAN) {
        Serial3.write(0xFE); 
        if (linea_seleccionada == 0) Serial3.write(0x94 + 4); 
        else Serial3.write(0xD4 + 4);

        Serial3.write(0xFE); 
		Serial3.write(0x0D); 		// Blink ON
    } else {
        Serial3.write(0xFE); 
		Serial3.write(0x0C); 		// Cursor OFF
    }
}

// ISR PCA9555
ISR(INT1_vect){
    // No I2C para evitar bloqueos
    flag_botonera = true;
}

int readInt(String prompt) {
    String inputString = "";
    bool inputDone = false;
    
    // Imprimir el mensaje si no está vacío
    if (prompt != "") Serial.print(prompt);

    while (!inputDone) {
        // Verificar si salimos del modo PROG mientras esperamos input
        if (modo_priego != PROG) return -1; 

        if (Serial.available() > 0) {
            char c = Serial.read();
            if (isDigit(c)) {
                inputString += c;
                Serial.print(c);
            } else if (c == '\b' || c == 127) { 
                if (inputString.length() > 0) {
                    inputString.remove(inputString.length() - 1); 		// Borrar de la variable
                    Serial.print("\b \b"); 														// Retroceder, Espacio, Retroceder
                }
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
    LR[d] [l].estado = st;
    // Guardamos tiempos (si es estado 2 se guardan los datos, si no, se guardan ceros)
    if (st == 2) {
        LR[d] [l].hi.hh = t_hh;
        LR[d] [l].hi.mm = t_mm;
        LR[d] [l].hi.ss = t_ss;
        LR[d] [l].T.mm = dur_mm;
        LR[d] [l].T.ss = dur_ss;
    } else {
        // Limpiar datos antiguos si pasamos a OFF u ON manual
        LR[d] [l].hi.hh = 0;
        LR[d] [l].hi.mm = 0; LR[d] [l].hi.ss = 0;
        LR[d] [l].T.mm = 0; LR[d] [l].T.ss = 0;
    }

    guardar_programacion();		// Guardar cambios en EEPROM

    Serial.print("Programacion Guardada para el Dia "); 
	Serial.println(d);
}

void showMenu() {
    Serial.write(12); 		// Clear terminal
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
	} else if (col == 0 && modo_priego == AUTOPLUS) { 
        // Tecla '*' en modo AUTO+:-> guardar porcentaje
        int nuevo_val = buffer.toInt();
        if (nuevo_val >= 0 && nuevo_val <= 100) {
            autoplus = nuevo_val;
            Serial.print("Autoplus actualizado: ");
            Serial.println(autoplus);
        }
        buffer="";
        tone(SPEAKER, 1000, 100);
    } else {
		setBuffer(3, col);
	}
}

void printTwoDigits(int number) {
	if (number < 10) Serial3.print("0");
	Serial3.print(number);
}

// Convierte una hora (HH:MM:SS) a segundos totales desde las 00:00:00
long toSeconds(int h, int m, int s) {
    return (h * 3600L) + (m * 60L) + s;
}

// escribir entero (2 bytes) en EEPROM I2C
void eeprom_write_int(int address, int value) {
    write_in_memory(address, (value >> 8) & 0xFF); 			// Byte alto
    delay(5); 																					//esperar ciclo de escritura
    write_in_memory(address + 1, value & 0xFF);    				// Byte bajo
    delay(5);
}

// leer entero (2 bytes) de EEPROM I2C
int eeprom_read_int(int address) {
    byte high = randomRead(address);
    byte low = randomRead(address + 1);
    return (high << 8) | low;
}

// Guarda LR en la memoria EEPROM
void guardar_programacion() {
    Serial.println("Guardando en EEPROM...");
    int addr = EEPROM_START_ADDR;
    
    // Recorrer 7 días (índices 1 al 7) y 2 líneas (0 al 1)
    for (int d = 1; d <= 7; d++) {
        for (int l = 0; l < 2; l++) {
			// Guardar el estado
            eeprom_write_int(addr, LR[d] [l].estado);
			addr += 2;
            
            // Guardar hora inicio (hh, mm, ss)
            eeprom_write_int(addr, LR[d] [l].hi.hh);
			addr += 2;
            eeprom_write_int(addr, LR[d] [l].hi.mm);
			addr += 2;
            eeprom_write_int(addr, LR[d] [l].hi.ss);
			addr += 2;
            
            // Guardar duración (mm, ss)
            eeprom_write_int(addr, LR[d] [l].T.mm);
			addr += 2;
            eeprom_write_int(addr, LR[d] [l].T.ss);
			addr += 2;
        }
    }
    
    // Marcar memoria como inicializada
    write_in_memory(EEPROM_ADDR, EEPROM_VAL);
    delay(5);
    Serial.println("Guardado completado.");
}

// Carga toda la matriz LR desde la memoria EEPROM
void cargar_programacion() {
    Serial.println("Cargando de EEPROM...");
    int addr = EEPROM_START_ADDR;
    
    for (int d = 1; d <= 7; d++) {
        for (int l = 0; l < 2; l++) {
            // Leemos el estado
            LR[d][l].estado = eeprom_read_int(addr); addr += 2;
            
            // Leemos hora inicio
            LR[d][l].hi.hh = eeprom_read_int(addr); addr += 2;
            LR[d][l].hi.mm = eeprom_read_int(addr); addr += 2;
            LR[d][l].hi.ss = eeprom_read_int(addr); addr += 2;
            
            // Leemos duración
            LR[d][l].T.mm = eeprom_read_int(addr); addr += 2;
            LR[d][l].T.ss = eeprom_read_int(addr); addr += 2;
        }
    }
    Serial.println("Carga completada.");
}

// Inicializa la estructura a 0 (solo se usa si la EEPROM está vacía/nueva)
void reset_programacion() {
    for (int d = 1; d <= 7; d++) {
        for (int l = 0; l < 2; l++) {
            LR[d][l].estado = 0;
            LR[d][l].hi.hh = 0; LR[d][l].hi.mm = 0; LR[d][l].hi.ss = 0;
            LR[d][l].T.mm = 0; LR[d][l].T.ss = 0;
        }
    }
    guardar_programacion(); // Guardamos los ceros en memoria
}

void loop() {
	readRTC(); 		// Leer hora

    switch(modo_priego) {
        case 0: //MODO MANUAL
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

        case 1: //MODO PROGRAMACIÓN
            if (!menu_mostrado) {
                showMenu();
                menu_mostrado = true;
            }

            if (Serial.available() > 0) {
                int op = readInt("");
                
                switch(op) {
                    case 1:
						opcion_ajustar_hora();
						break;
                    case 2:
						opcion_ajustar_fecha();
						break;
                    case 3: 
						opcion_riego();
						break;
                    default:
						Serial.println("Opcion no valida");
						break;
                }
                delay(1000); 
                menu_mostrado = false; 		// volver a pintar menú
            }
            break;

        case 2:	//MODO VER
            if (flag_botonera) {
                flag_botonera = false;
                byte botones = getPCA_Input(); // Leer estado (0 = pulsado)

                // Botón LEFT (Bit 0): Retroceder día
                if (!(botones & 0x01)) { 
                    view_day--;
                    if (view_day < 1) view_day = 7; 	// Vuelta al Domingo
                    Serial.print("Viendo Dia: ");
					Serial.println(view_day);
                }

                // Botón RIGHT (Bit 3): Avanzar día
                if (!(botones & 0x08)) { 
                    view_day++;
                    if (view_day > 7) view_day = 1; 	// Vuelta al Lunes
                    Serial.print("Viendo Dia: ");
					Serial.println(view_day);
                }
            }
            break;

        case 3:	//MODO AUTO
			// Recorrer líneas de riego (0 y 1)
            for(int i=0; i<2; i++) {
                // Obtener configuración para el día de HOY y la línea i
                // current_day viene del RTC (1..7)
                int estado_prog = LR[current_day][i].estado;
                
                if (estado_prog == 1) {
                    set_valvula(i, 1);	// Modo ON Forzado (Manual ON guardado en programación)
                } else if (estado_prog == 0) {
                    set_valvula(i, 0);	// Modo OFF Forzado
                } else if (estado_prog == 2) {
                    // Modo PROGRAMADO (Horario)
                    // Calcular tiempo actual en segundos
                    long now_sec = toSeconds(current_hour, current_min, current_sec);
                    
                    // Calcular tiempo de INICIO programado
                    long start_sec = toSeconds(LR[current_day][i].hi.hh, LR[current_day][i].hi.mm, LR[current_day][i].hi.ss);
                                               
                    // Calcular DURACIÓN en segundos
                    long duration_sec = (LR[current_day][i].T.mm * 60L) + LR[current_day][i].T.ss;
                    
                    // calcular tiempo de FIN
                    long end_sec = start_sec + duration_sec;
                    
                    // Comparar si está dentro del intervalo
                    // Se añade (end_sec > start_sec) para evitar activaciones erróneas si la duración es 0
                    if (duration_sec > 0 && now_sec >= start_sec && now_sec < end_sec) {
                        set_valvula(i, 1); // ENCENDER (Dentro del horario)
                    } else {
                        set_valvula(i, 0); // APAGAR (Fuera de horario)
                    }
                }
            }
            break;

        case AUTOPLUS:
            for(int i=0; i<2; i++) {
                int estado_prog = LR[current_day][i].estado;
                
                if (estado_prog == 2) {
                    long now_sec = toSeconds(current_hour, current_min, current_sec);
                    long start_sec = toSeconds(LR[current_day][i].hi.hh, LR[current_day][i].hi.mm, LR[current_day][i].hi.ss);
                    
                    // CALCULO INCREMENTADO
                    long duration_base = (LR[current_day][i].T.mm * 60L) + LR[current_day][i].T.ss;
                    long duration_plus = duration_base + (duration_base * autoplus / 100);
                    
                    long end_sec = start_sec + duration_plus;

                    if (duration_plus > 0 && now_sec >= start_sec && now_sec < end_sec) {
                        set_valvula(i, 1);
                    } else {
                        set_valvula(i, 0);
                    }
                } else if (estado_prog == 1) {
                    set_valvula(i, 1);          // ON manual guardado
                } else {
                    set_valvula(i, 0);          // OFF
                }
            }
            break;
		
		default:
			break;
    }
}
