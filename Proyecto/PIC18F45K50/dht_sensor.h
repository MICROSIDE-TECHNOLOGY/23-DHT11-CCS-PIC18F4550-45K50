#include <stdint.h> //Permite el uso de variables tipo uint8_t

/*******************************************************************************
Tipos de variables para la lectura del sensor
********************************************************************************/

typedef uint8_t dht11_pulse_counter_t;

static dht11_pulse_counter_t __dht11_high_time = 30;

/*******************************************************************************
Instrucciones que controlan el sensor
********************************************************************************/

/*******************************************************************************
 Funcion: dht11_send_start
 -----------------------
 Se�al de inicio al sensor

 retorna: no parametros
********************************************************************************/
static inline void dht11_send_start()
{
    output_drive( DHT11_PIN );
    output_low(DHT11_PIN);

    delay_ms( 20 );                  // Se�al de inicio

    output_high(DHT11_PIN);
    output_float( DHT11_PIN );

    delay_us( 1 );
}

int dht11_init();
int dht11_read( float *temp, float *hum );

/*******************************************************************************
 Funcion: dht11_init
 -----------------------
 Configura I/O para el Bus de comunicaci�n OW, verifica que el sensor se encuentre
 conectado y funcionando correctamente

 retorna: no parametros
********************************************************************************/
int dht11_init()
{
    static uint8_t interrupt_status = 0;
    double tmp;

    interrupt_status = interrupt_enabled(global);
    disable_interrupts(global);

    dht11_send_start();

    dht11_pulse_counter_t pulse_lenght = 1;

    while ( input( DHT11_PIN ) ) {
        if ( !pulse_lenght++ ) {
            if ( interrupt_status ) {
               enable_interrupts(global);
            }
            return -1;
        }
            
    }
    pulse_lenght = 1;
    while ( !input( DHT11_PIN ) ) {
        if ( !pulse_lenght++ ) {
            if ( interrupt_status ) {
               enable_interrupts(global);
            }
            return -1;
        }
    }

    if ( interrupt_status ) {
      enable_interrupts(global);
    }

    // La respuesta del sensor es de 80uS, dividimos la cantidad
    // de incrementos del contador entre este valor para estimar
    // 1uS
    tmp = pulse_lenght/80.0;

    // 70uS = 1, 30 uS = 0, entonces pulsos > 35uS se consideran 1 
    tmp = tmp * 35;

    __dht11_high_time = (dht11_pulse_counter_t)tmp;

    return 0;
}

/*******************************************************************************
 Funcion: dht11_read
 -----------------------
 Envia una petici�n de lectura al DHT11, interpreta la respuesta y convierte el valor
 en su representaci�n con punto flotante.

 @param temp: Variable de temperatura
 @param hum: Variable de humedad

 retorna:
 @return 0: Operaci�n exitosa
 @return -1: Tiempo de espera agotado
 @return -2: Error de suma de verificaci�n
********************************************************************************/
int dht11_read( float *temp, float *hum )
{
    static uint8_t interrupt_status = 0;

    uint8_t data[5] = { 0 };

    dht11_pulse_counter_t pulse_train[40] = { 0 };

    dht11_pulse_counter_t pulse_lenght = 1;

    // Deshabilita las interrupciones para calcular los intervalos de tiempo
    interrupt_status = interrupt_enabled(global);
    disable_interrupts(global);

    dht11_send_start();

    // Respuesta del sensor
    while ( input( DHT11_PIN ) ) {
        if ( !pulse_lenght++ )
            goto timeout_error;
    }
    pulse_lenght = 1;
    while ( !input( DHT11_PIN ) ) {
        if ( !pulse_lenght++ )
            goto timeout_error;
    }
    pulse_lenght = 1;
    while ( input( DHT11_PIN ) ) {
        if ( !pulse_lenght++ )
            goto timeout_error;
    }
    pulse_lenght = 1;
    while ( !input( DHT11_PIN ) ) {
        if ( !pulse_lenght++ )
            goto timeout_error;
    }

    // Capturar el tren de pulsos
    for ( int i = 0; i < 40; i++ ) {
        pulse_train[i] = 1;
        pulse_lenght = 1;
        while ( input( DHT11_PIN ) ) {
            if ( !pulse_train[i]++ )
                goto timeout_error;
        }
        while ( !input( DHT11_PIN ) ) {
            if ( !pulse_lenght++ )
                goto timeout_error;
        }
    }

    // Decodificar el tren de pulsos
    for ( int i = 0; i < 40; i += 8 ) {
        for ( int j = 0; j < 8; j++ ) {
            if ( pulse_train[i + j] > __dht11_high_time ) {
                data[i / 8] |= 1 << ( 7 - j );
            }
        }
    }

    // Restablecer las interrupciones
    if ( interrupt_status ) {
      enable_interrupts(global);
    }

    *hum = (float)(data[0] + ( data[1] * 0.1 ));
    *temp = (data[2] + ( data[3] * 0.1 ));

    if ( data[4] != ( data[0] + data[1] + data[2] + data[3] ) )
        return data[4];

    return 0;

timeout_error:
    if ( interrupt_status ) {
      enable_interrupts(global);
    }
    return -1;
}
