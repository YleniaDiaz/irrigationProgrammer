// Declaración de pines para el i2c
#define ESC_SCL  4 	 					// puerto de salida para escribir el valor de la línea SCL-out
#define ESC_SDA  39  					// puerto de salida para escribir el valor de la línea SDA-out
#define LEE_SCL  40  					// puerto de entrada para leer el estado de la línea SCL
#define LEE_SDA  41  					// puerto de entrada para leer el estado de la línea SDA

// Direcciones físicas de los dispositivos i2c
#define D_EEPROM0 0xA0			// DEVICE EEPROM0 (chip 0) w:0xA0   r:0xA1
#define D_EEPROM1 0xA8
#define D_SHT21 0x80					// sensor de temperatura/humedad
#define D_DS3232 0xD0				// reloj   w:0xD0   r:0xD1
#define D_PCA9555 0x40				// expansor de E/S digitales

// Memoria
#define ADDR_WRITE 0xA0 	// dspositivo (1010 000) + Write 0
#define ADDR_READ 0xA1 		// dspositivo (1010 000) + Read 1
#define PAGE_SIZE 32 				// Tamaño pag 24LC64
#define MEMORY_SIZE 8192 	// Tamaño total memoria

// declaración de funciones en i2c_functions.cpp
void i2c_start();
void i2c_stop();
void i2c_free();
void i2c_w1();
void i2c_w0();
byte i2c_rbit();
void i2c_write_byte(byte dato);
byte i2c_read_byte();

void write_in_memory (int address, byte data);
byte randomRead(int address);

void initPCA();
