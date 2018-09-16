volatile uint32_t timer_value = 0;
volatile uint32_t ovf_n = 0;
volatile uint8_t led_state = 0;
double dt = 0;
int counter = 0;
uint32_t sample_id = 0;
volatile uint32_t overflows_n = 0;

#define CONV_SEC_TO_CM  1e5f
#define CONV_SEC_TO_M   1e3f

#define CONV_SEC_TO_FT  3280.83989501f


void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  // Capture pin
  DDRD &= ~(0x02);  // PD2 as input
  PORTD |= 0x02;  // pullups on PD2
  EICRA |= 0x01;  // any logic change INT0 / PD2
  EIFR &= ~(0x01);  // clear the interrupt flag
  EIMSK |= 0x01;  // enable interrupt on INT0 / PD2

  // Timer
  TCCR1A &= ~(0x03); // normal mode
}

inline void start_timer() {
  TCCR1B = 0;       // stop the timer
  TCNT1L = 0;       //reset the value L 
  TCNT1H = 0;       // reset the value H
  ovf_n = 0;        // set the number of overflows to 0
  EIFR &= ~(0x01);  // clear the interrupt flag on the PD2
  TIMSK1 |= 0x01;   // enable overflow interrupt
  TCCR1B = 0x03;    // start the timner
}

inline void stop_tmimer() {
  TCCR1B = 0;         // literally stop it
  TIMSK1 &= ~(0x01);  // disable timer 1 interrupt
  TCNT1L = 0;         // reset the LOW byte of the value
  TCNT1H = 0;         // reset the high byte of the value

}

ISR(TIMER1_OVF_vect) {      // interrupt overflow routine
  ovf_n++;
  overflows_n++;
}

ISR (INT0_vect)
{
    /* interrupt code here */
    counter++;
    if( PIND & 0x04 ) {
      start_timer();
    } else {
      timer_value = 0;
      timer_value |= TCNT1L;
      timer_value |= (TCNT1H<<8);
//      timer_value |= (((uint32_t)ovf_n)<<16);
      stop_tmimer();
    }
//     EIFR &= ~(0x01);  // clear the interrupt flag
}

void loop() {
  sample_id++;
  dt = timer_value * 0.000004f;
  
  
  // put your main code here, to run repeatedly:
  Serial.print(sample_id);
  Serial.print(", timer value: ");
  Serial.print(timer_value);
  Serial.print(", dt: ");
  Serial.print(dt, 4);
  Serial.print(" sec ");
  Serial.print(", dist [cm]: ");
  Serial.print( dt*CONV_SEC_TO_CM, 2 );
  Serial.print(", dist [m]: ");
  Serial.print( dt*CONV_SEC_TO_M, 2 );
  Serial.print(", dist [ft]: ");
  Serial.print( dt*CONV_SEC_TO_FT, 2 );

  Serial.print(", overflows: ");
  Serial.print( overflows_n );
  Serial.println("");
  
overflows_n = 0;


  
  delay(500);
}
