/************************************************************************************************
Company:
Microside Technology Inc.
Product Revision  :  1
Device            :  X-TRAINER
Driver Version    :  1.0
************************************************************************************************/
/*
---------------------------------------------------------------------------
Esta práctica consiste en imprimir un mensaje, a través de una
pantalla LCD
---------------------------------------------------------------------------
*/

#include <18F45K50.h>                          //Incluye el microcontrolador con el que se va a trabajar
#use delay( internal = 48MHz )                 // Tipo de oscilador y frecuencia dependiendo del microcontrolador
#use i2c( master, sda=PIN_B0, scl=PIN_B1 )
#build( reset = 0x02000, interrupt = 0x02008 ) // Asigna los vectores de reset e interrupción para la versión con bootloader
#org 0x0000, 0x1FFF {}                         // Reserva espacio en memoria para el bootloader

#define DHT11_PIN PIN_B2

#include "flex_lcd.h"     //LIBRERIA LCD
#include "dht_sensor.h"

float temperature = 0;
float humidity = 0;
int dht11_stat = 0;

void main()
{
   
    delay_ms( 1000 );
    lcd_i2c_init( 0x27 );            // Inicializa la pantalla LCD

    lcd_i2c_clear();
    lcd_i2c_setCursor( 3, 0 );
    lcd_i2c_write_byte( "DHT11" );
    lcd_i2c_setCursor( 3, 1 );

    if ( dht11_init() != 0 ) {
        lcd_i2c_write_byte( "ERROR" );
        while ( 1 ) {
        }
    }

    lcd_i2c_write_byte( "OK" );
    delay_ms( 1000 );

    while ( 1 ) {
        delay_ms( 2000 );

        dht11_stat = dht11_read( &temperature, &humidity );

        if ( dht11_stat != 0 ) {
            lcd_i2c_clear();
            lcd_i2c_setCursor( 0, 0 );
            lcd_i2c_write_byte( "DHT11 Error" );
            continue;
        }

        lcd_i2c_clear();
        lcd_i2c_setCursor( 0, 0 );
        lcd_i2c_write_byte( "T: " );
        lcd_i2c_write_double( temperature, 2 );
        lcd_i2c_write_byte( " C" );
        lcd_i2c_setCursor( 0, 1 );
        lcd_i2c_write_byte( "H: " );
        lcd_i2c_write_double( humidity, 2 );
        lcd_i2c_write_byte( " %" );
    }
}
