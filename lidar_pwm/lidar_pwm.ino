volatile uint32_t timer_value = 0;
volatile uint16_t ovf_n = 0;
volatile uint8_t led_state = 0;
float dt = 0;
int counter = 0;
uint32_t sample_id = 0;

#define CONV_SEC_TO_CM  1e5f
#define CONV_SEC_TO_M   1e3f

#define CONV_SEC_TO_FT  3280.83989501f

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);

  // Capture pin
  DDRD &= ~(0x02);  // PD2 as input
  PORTD |= 0x02;  // pullups on PD2
  EICRA |= 0x01;  // any logic change INT0 / PD2
  EIFR &= ~(0x01);  // clear the interrupt flag
  EIMSK |= 0x01;  // enable interrupt on INT0 / PD2

  // Timer
  TCCR1A &= ~(0x03); // normal mode
  TCCR1B = 0x05;  // 1024 prescaler
}

inline void start_timer() {
  TCCR1B = 0;
    TCNT1L = 0;
  TCNT1H = 0;
  ovf_n = 0;
  TIFR1 &= ~(0x01); // clear overflow flag
  TIMSK1 |= 0x01; // enable overflow interrupt
  TCCR1B = 0x02;  // clock /8
}

inline void stop_tmimer() {
  TIMSK1 &= ~(0x01);  // disable timer 1 interrupt
  TCCR1B = 0;
    TCNT1L = 0;
  TCNT1H = 0;

}

ISR(TIM1_OVF_vect) {      // interrupt overflow routine
  ovf_n++;
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
      timer_value |= (((uint32_t)ovf_n)<<16);
      stop_tmimer();
    }
     EIFR &= ~(0x01);  // clear the interrupt flag
}

void loop() {
  sample_id++;
  dt = timer_value*5e-7;
  
  
  // put your main code here, to run repeatedly:
  Serial.print(sample_id);
  Serial.print(", ");
  Serial.print(", dt: ");
  Serial.print(dt, 4);
  Serial.print(" sec ");
  Serial.print(", dist [cm]: ");
  Serial.print( dt*CONV_SEC_TO_CM, 2 );
  Serial.print(", dist [m]: ");
  Serial.print( dt*CONV_SEC_TO_M, 2 );
  Serial.print(", dist [ft]: ");
  Serial.print( dt*CONV_SEC_TO_FT, 2 );

  Serial.println("");
  



  
  delay(500);
}
