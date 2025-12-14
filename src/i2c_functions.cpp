#include <arduino.h>
#include <i2c_functions.h>

void i2c_start(){
	// cli(); 		// Deshabilitar interrupciones semáforo
    
    // comprobar que el bus está libre antes de tomarlo
    digitalWrite(ESC_SDA, HIGH);
    digitalWrite(ESC_SCL, HIGH);

    // Comprobar que el bus está realmente libre
    if (digitalRead(LEE_SDA) == LOW || digitalRead(LEE_SCL) == LOW) {
		delayMicroseconds(10); 		// Esperar si el bus está ocupado
    }

    // START (SDA baja mientras SCL está alta)
    digitalWrite(ESC_SDA, LOW);  
    delayMicroseconds(10);
    
    // Bajar SCL para primer bit
    digitalWrite(ESC_SCL, LOW);  	// SDA en LOW por seguridad 
}

void i2c_stop() {
	// Poner SDA a LOW 
    digitalWrite(ESC_SCL, LOW);
    digitalWrite(ESC_SDA, LOW);
    delayMicroseconds(10);

    // SCL sube primero
    digitalWrite(ESC_SCL, HIGH);
    delayMicroseconds(10); 	// Esperar a que SCL esté estable

    // SDA sube MIENTRAS SCL está alta (STOP)
    digitalWrite(ESC_SDA, HIGH);
    delayMicroseconds(10);
    
    //sei(); // Habilitar interrupciones semáforo
}

void i2c_free(){
	// Liberar el bus. Si sda fuese 0 entonces habria un stop?
  	digitalWrite(ESC_SCL, HIGH); 			// soltamos scl
  	digitalWrite(ESC_SDA, HIGH); 			// soltamos sda
}

void i2c_w1(){
	// suponemos que SDA=0 y SCL=0
	digitalWrite(ESC_SDA,HIGH);
	digitalWrite(ESC_SCL, HIGH);
	digitalWrite(ESC_SCL, LOW);
	digitalWrite(ESC_SDA, LOW); // dejamos sda=0 scl=0	
}

void i2c_w0(){
	// suponemos que SDA=0 y SCL=0 
	digitalWrite(ESC_SDA, LOW);
	digitalWrite(ESC_SCL, HIGH);
	digitalWrite(ESC_SCL, LOW);		// dejamos scl=0
	digitalWrite(ESC_SDA, LOW); 		// dejamos sda=0 scl=0
}  

byte i2c_rbit(){
	// Suponemos que sda=0 y scl=0
	byte  value;
	digitalWrite(ESC_SDA, HIGH);				// liberar sda 
	digitalWrite(ESC_SCL, HIGH);				// arriba el reloj
	value=digitalRead(LEE_SDA);				// leemos sda
	digitalWrite(ESC_SCL, LOW); 
	digitalWrite(ESC_SDA, LOW); // dejamos sda=0, scl=0
	return value;
}

void i2c_write_byte(byte dato){
	for(int i=0; i<8; i++) {
		if ((dato&128) != 0) i2c_w1();
		else i2c_w0();
		dato = dato << 1;
     }
}

byte i2c_read_byte(){
	byte ibyte = 0;
    for (byte i=0;i<8; i++){
		ibyte=  (ibyte<<1) | (i2c_rbit()&1);
	}
    return ibyte;
}

void write_in_memory (int address, byte data) {
	// address = 16 bits -> int = 16 bits
	start_op:
		i2c_start();											// iniciar trama
		i2c_write_byte(0xA0);									// enviar dirección de memoria diciendo que vamos a escribir (B1010 000 0)
		if (i2c_rbit() != 0) goto start_op;  		// si no se recibe ack hay que empezar de nuevo
		i2c_write_byte(address >> 8); 					// desplazamos address a la derecha para coger la parte HIGH
		if (i2c_rbit() != 0) goto start_op;   	// si no se recibe ack hay que empezar de nuevo
		i2c_write_byte(address & 0x00FF); 		// coger la parte LOW del address
		if (i2c_rbit() != 0) goto start_op;  		// si no se recibe ack hay que empezar de nuevo
		i2c_write_byte(data); 									// escribir dato
		if (i2c_rbit() != 0) goto start_op;   	// si no se recibe ack hay que empezar de nuevo
		i2c_stop(); 											// parar trama
}

byte randomRead(int address) {
	byte data;
	
	random_read_retry:
		// posicionar puntero
		i2c_start();
		i2c_write_byte(ADDR_WRITE);					// escribir
		if (i2c_rbit() != 0) goto random_read_retry;
		i2c_write_byte(address >> 8);						// Parte HIGH de la memoria
		if (i2c_rbit() != 0) goto random_read_retry;
		i2c_write_byte(address & 0x00FF); 			// Parte LOW de la memoria
		if (i2c_rbit() != 0) goto random_read_retry;
		
		i2c_start();
		i2c_write_byte(ADDR_READ);			// leer
		if (i2c_rbit() != 0) goto random_read_retry;
		data = i2c_read_byte(); 
		i2c_w1();									// Enviar NO ACK porque solo vamos a leer un byte
		i2c_stop();
		
		return data;
}

void waitWriteComplete() {
	// La escritura puede tardar, durante la espera la memoria responderá NO ACK
	// entonces hacemos polling hasta que responda ACK
	byte_write_retry:
		i2c_start();
		i2c_write_byte(ADDR_WRITE); // dirección de escritura
		if (i2c_rbit() != 0) {
			i2c_stop();
			delay(1); 	// esperar antes de reintentar
			goto byte_write_retry;
		}

	i2c_stop();		// hubo ACK
}

void pageWrite(int startAddress, byte data, int numBytes) {
	int bytesWritten = 0;
	
	while (bytesWritten < numBytes) {
		int bytesInPage = PAGE_SIZE - (startAddress % PAGE_SIZE); 	// ver cuantos bytes caben en la pag actual
	
		// ver cuantos bytes vamos a escribir
		int bytes = numBytes - bytesWritten; 						// bytes que faltan
		if (bytes > bytesInPage) bytes = bytesInPage; 		// no se puede escribir más de lo que cabe en la pag
		
		// trama Page Write
		page_write_retry:
			i2c_start();
			i2c_write_byte(ADDR_WRITE);						// escribir
			if (i2c_rbit() != 0) goto page_write_retry;
			i2c_write_byte(startAddress >> 8);					// Parte HIGH de la memoria
			if (i2c_rbit() != 0) goto page_write_retry;
			i2c_write_byte(startAddress & 0x00FF); 		// Parte LOW de la memoria
			if (i2c_rbit() != 0) goto page_write_retry;
			
			// enviar todos los bytes
			for (int i = 0; i < bytes; i++) {
				i2c_write_byte(data);
				if(i2c_rbit() != 0) goto page_write_retry;
			}
			
			i2c_stop();
			
			waitWriteComplete();
			
			bytesWritten += bytes;
			startAddress += bytes;
	}
}

void initPCA() {
	pca_init_retry:
		i2c_start();
		i2c_write_byte(D_PCA9555); 
		if (i2c_rbit() != 0) goto pca_init_retry;
		i2c_write_byte(0x06);			// Escribir en registro de Configuración 0 (reg 6)
		if (i2c_rbit() != 0) goto pca_init_retry;
		//i2c_write_byte(0x00);			// Escribir 0x00 para P0 (todo salidas)
		i2c_write_byte(0xFF);      		// 0xFF = Todo Entradas (Botones)
		if (i2c_rbit() != 0) goto pca_init_retry;
		i2c_write_byte(0x00);			// Escribir 0x00 para P1 (todo salidas)
		if (i2c_rbit() != 0) goto pca_init_retry;
		i2c_stop();
}

// Leer estado botones (Puerto 0)
byte getPCA_Input() {
	byte data = 0;
    i2c_start();
    i2c_write_byte(D_PCA9555); 			// Escritura
    i2c_rbit();
    i2c_write_byte(0x00);      			// Puntero a Input Port 0
    i2c_rbit();
    
    i2c_start(); 						// Restart
    i2c_write_byte(D_PCA9555 | 1); 		// Lectura (0x41)
    i2c_rbit();
    
    data = i2c_read_byte();
    i2c_w1(); 							// NACK (fin de lectura)
    i2c_stop();
    return data;
}

// Escribe en las válvulas (Puerto 1)
void setPCA_Output(byte valor) {
    i2c_start();
    i2c_write_byte(D_PCA9555);
    i2c_rbit();
    i2c_write_byte(0x03);      // Puntero a Output Port 1
    i2c_rbit();
    i2c_write_byte(valor);     // Escribir estado válvulas (bit 0 y 1)
    i2c_rbit();
    i2c_stop();
}

//RTC DS3232

// Conversión de Decimal a BCD
byte decToBcd(byte val) {
  return ((val / 10 * 16) + (val % 10));
}

// Conversión de BCD a Decimal
byte bcdToDec(byte val) {
  return ((val / 16 * 10) + (val % 16));
}

// Configurar hora del RTC
void setRTC_Time(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year) {
    i2c_start();
    i2c_write_byte(D_DS3232); 		// Dirección Escritura (0xD0)
    i2c_rbit();
    i2c_write_byte(0x00);     		// Puntero al registro 0 (Segundos)
    i2c_rbit();
    
    i2c_write_byte(decToBcd(second)); i2c_rbit();
    i2c_write_byte(decToBcd(minute)); i2c_rbit();
    i2c_write_byte(decToBcd(hour));   i2c_rbit();
    i2c_write_byte(decToBcd(dayOfWeek)); i2c_rbit();
    i2c_write_byte(decToBcd(dayOfMonth)); i2c_rbit();
    i2c_write_byte(decToBcd(month)); i2c_rbit();
    i2c_write_byte(decToBcd(year));  i2c_rbit();
    
    i2c_stop();
}

// Leer datos del RTC (Devuelve un array con los datos)
// Índices: 0:seg, 1:min, 2:hour, 3:dayWeek, 4:dayMonth, 5:month, 6:year
byte* getRTC_DateTime() {
    static byte rtc_data[7];
    
    i2c_start();
    i2c_write_byte(D_DS3232); 			// Escritura para poner puntero
    i2c_rbit();
    i2c_write_byte(0x00);     			// Registro 0
    i2c_rbit();
    
    i2c_start();              			// Restart
    i2c_write_byte(D_DS3232 | 1); 		// Lectura (0xD1)
    i2c_rbit();
    
    // Leer 7 bytes
    for(int i=0; i<6; i++) {
        rtc_data[i] = bcdToDec(i2c_read_byte());
        i2c_w0(); 						// ACK (queremos más datos)
    }
    rtc_data[6] = bcdToDec(i2c_read_byte());
    i2c_w1(); 							// NACK (último dato)
    
    i2c_stop();
    return rtc_data;
}
