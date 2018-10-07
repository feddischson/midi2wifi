/*
 * This small code just echos the UART data.
 * Each time, the received byte is incremented by one before sending it back.
 * 
 * This small program is used for testing purposes with the following setup:
 *    +--------+
 *    |        |                                                  +---------------+
 *    |        |          +----------+          +-----------+      |               |
 *    |        | USB/UART |          |   WIFI   |           | UART |               |
 *    |  PC    +----------+  ESP01   +----------+ ESP01     +------+  ATTINY2313   |
 *    |        |          | (Device) |          | (Host)    |      |               |
 *    |        |          +----------+          +-----------+      |               |
 *    |        |                                                  +---------------+
 *    +--------+
 *
 *
 *
 *
 */


#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>

void initStatus   ( void );
void initPorts    ( void );
uint8_t status;

void initUSART ( void )
{
   // no parity
   // 1 stop bit
   UCSRC =     (0 << UMSEL)   // async. 
            |  (0 << UPM0)
            |  (0 << UPM1)    // no parity
            |  (0 << USBS)    // 1 stop bit
            |  (1 << UCSZ0)
            |  (1 << UCSZ1)   // 8 bit data transfer
            |  (0 << UCPOL);

   UCSRB =     (1 << RXEN)    // enable receive
            |  (1 << TXEN)    // enable transmit
            |  (1 << RXCIE);  // enable rx irq

   UCSRA   = (1 << U2X);

   // init the baudrate
   UBRRH = 0x00;
   UBRRL = 3;
}

ISR(USART_RX_vect)
{
   while ( !(UCSRA & (1<<RXC)) )
      ;
   uint8_t tmp = UDR;
   writeUSART(tmp);
}

void writeUSART(uint8_t data)
{
   while ( !(UCSRA & (1<<UDRE)) )
        ;
   UDR = data; 
}

void initPorts ( void )
{
   DDRA = 0xff;
   DDRB = 0xff;
   DDRD = 0xff;
   PORTB = 0;
}

void initStatus ( void )
{
   status = 0x55;
   PORTB = status;
}

void changeStatus( void )
{
   if(status % 2)
      status = status << 1;
   else
      status = (status << 1)|1;
   PORTB = status;
}

int main(void)
{

   initPorts      ( );
   initStatus     ( );
   initUSART      ( );
   sei();
   while(1)
   {  
      _delay_ms(2000);
      changeStatus( );
   }  
}
