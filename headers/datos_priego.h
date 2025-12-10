
// estructura de datos del programador
// PROGRAMACIÓN PARA 7 DÍAS
// Cada día, dos sesiones de riego (0 y 1) configuradas como: off-comienzo del riego-duración del riego
	
struct hora{
    int hh,mm,ss;
};

struct tiempo_riego{
    int mm, ss;
};

struct linea_riego{
    int estado; 	// 0:OFF  1:ON  2: Tiempo comienzo riego
    hora hi;		// hora inicial: hh:mm:ss (AUTO)
    tiempo_riego T;	// tiempo que está regando (AUTO)
};
