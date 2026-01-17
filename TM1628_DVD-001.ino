//Driver of display of 7 segments used on DVD players cheaps
/****************************************************/
/* This is only one example of code structure       */
/* OFFCOURSE this code can be optimized, but        */
/* the idea is let it so simple to be easy catch    */
/* where can do changes and look to the results     */
/****************************************************/

#define DSP_in 7// If 0 write LCD, if 1 read of LCD
#define DSP_clk 8 // if 0 is a command, if 1 is a data0
#define DSP_stb 9 // Must be pulsed to LCD fetch data of bus

#define AdjustPins    PIND // before is C, but I'm use port C to VFC Controle signals, if you change the selection of pins, adjuste this port!

/*Global Variables Declarations*/
unsigned char hours = 0;
unsigned char minutes = 0;
unsigned char minute = 0;
unsigned char secs=0;
unsigned char seconds=0;
unsigned char milisec = 0;

unsigned char memory_secs=0;
unsigned char memory_minutes=0;

unsigned char number;
unsigned char numberA0;
unsigned char numberA1;
unsigned char numberB0;
unsigned char numberB1;
unsigned char numberC0;
unsigned char numberC1;
unsigned char numberD0;
unsigned char numberD1;
unsigned char numberE0;
unsigned char numberE1;
unsigned char numberF0;
unsigned char numberF1;

unsigned char digit=0;
unsigned char grid=0;
unsigned char gridSegments = 0b00000011; // Here I define the number of GRIDs and Segments I'm using

boolean flagSecs=false;

 //The next lines is the creation of 0 to F chars bit by bit!
unsigned int segHexNumber[16][2] ={
  //        -defabcg     --------
         (0b01111110),(0b00000000), // 0
         (0b00000110),(0b00000000), // 1
         (0b01101101),(0b00000000), // 2 
         (0b01001111),(0b00000000), // 3
         (0b00010111),(0b00000000), // 4
         (0b01011011),(0b00000000), // 5
         (0b01111011),(0b00000000), // 6
         (0b00001110),(0b00000000), // 7
         (0b01111111),(0b00000000), // 8 
         (0b00011111),(0b00000000), // 9 
         (0b00111111),(0b00000000), // A 
         (0b01110011),(0b00000000), // B
         (0b01111000),(0b00000000), // C
         (0b01100111),(0b00000000), // D
         (0b01111001),(0b00000000), // E
         (0b00111001),(0b00000000)  // F
};
void SM1628C_init(void){
  delayMicroseconds(200); //power_up delay
  // Note: Allways the first byte in the input data after the STB go to LOW is interpret as command!!!

  // Configure VFD display (grids)
  cmd_with_stb(gridSegments); // cmd 1 // SM1628 is driver of 6 up to 7 grids
  delayMicroseconds(1);
  // Write to memory display, increment address, normal operation
  cmd_with_stb(0b01000000);//(BIN(01000000));
  delayMicroseconds(1);
  // Address 00H - 15H ( total of 11*2Bytes=176 Bits)
  cmd_with_stb(0b11000000);//(BIN(01100110)); 
  delayMicroseconds(1);
  // set DIMM/PWM to value
  cmd_with_stb((0b10001000) | 7);//0 min - 7 max  )(0b01010000)
  delayMicroseconds(1);
}
void cmd_without_stb(unsigned char a){
  // send without stb
  unsigned char data = 170; //value to transmit, binary 10101010
  unsigned char mask = 1; //our bitmask
  
  data=a;
  //This don't send the strobe signal, to be used in burst data send
         for (mask = 00000001; mask>0; mask <<= 1) { //iterate through bit mask
           digitalWrite(DSP_clk, LOW);
                 if (data & mask){ // if bitwise AND resolves to true
                    digitalWrite(DSP_in, HIGH);
                 }
                 else{ //if bitwise and resolves to false
                   digitalWrite(DSP_in, LOW);
                 }
          delayMicroseconds(5);
          digitalWrite(DSP_clk, HIGH);
          delayMicroseconds(5);
         }
   //digitalWrite(DSP_clk, LOW);
}
void cmd_with_stb(unsigned char a){
  // send with stb
  unsigned char data = 170; //value to transmit, binary 10101010
  unsigned char mask = 1; //our bitmask
  
  data=a;
  
  //This send the strobe signal
  //Note: The first byte input at in after the STB go LOW is interpreted as a command!!!
  digitalWrite(DSP_stb, LOW);
  delayMicroseconds(1);
         for (mask = 00000001; mask>0; mask <<= 1) { //iterate through bit mask
           digitalWrite(DSP_clk, LOW);
           delayMicroseconds(1);
                 if (data & mask){ // if bitwise AND resolves to true
                    digitalWrite(DSP_in, HIGH);
                 }
                 else{ //if bitwise and resolves to false
                   digitalWrite(DSP_in, LOW);
                 }
          digitalWrite(DSP_clk, HIGH);
          delayMicroseconds(1);
         }
   digitalWrite(DSP_stb, HIGH);
   delayMicroseconds(1);
}
void test_hexadecimal_DSP(){
  for(uint8_t num = 0; num < 16; num++){
    delayMicroseconds(1);
              //cmd_with_stb(gridSegments); // cmd 1 // SM1628C is a driver of 7 grids
              cmd_with_stb(0b01000000);//To set auto-increment
              cmd_with_stb(0b10000000); // cmd 2 //Normal operation; Set pulse as 1/16
              digitalWrite(DSP_stb, LOW);
              delayMicroseconds(1);
              cmd_without_stb(0b11000000); // Grids of display... they have done the swap of this pins with segments
              //                    
              digitalWrite(DSP_stb, LOW);
              delayMicroseconds(1);
              //
              cmd_without_stb(0b00000000);  //DIG1 --------
              cmd_without_stb(0b00000000);  //DIG1 --------
              //
              cmd_without_stb(0b00000000);  //DIG2 --------
              cmd_without_stb(0b00000000);  //DIG2 --------
              //
              cmd_without_stb(0b00000000);  //DIG3 -, Pause, Play, CD, DVD, ":", , ,
              cmd_without_stb(0b00000000);  //DIG3 --------
              //
              cmd_without_stb(0b00000000);  //DIG4 -defabcg
              cmd_without_stb(0b00000000);  //DIG4 --------
              //
              cmd_without_stb(0b00000000);  //DIG5 -defabcg
              cmd_without_stb(0b00000000);  //DIG5 --------
              //
              cmd_without_stb(0b00000000);  //DIG6 -defabcg
              cmd_without_stb(0b00000000);  //DIG6 --------
              //
              cmd_without_stb(segHexNumber[num][0]); //DIG7 -defabcg //This will represent the "number" created at position from 0~16 Byte HL & HU
              cmd_without_stb(segHexNumber[num][1]); //DIG7 -------- //This will represent the "number" created at position from 0~16 Byte HL & HU
              //
              digitalWrite(DSP_stb, HIGH);
              cmd_with_stb((0b10001000) | 7); //cmd 4
              delay(750);
  }
}
void test_symbols_DSP(void){
  digitalWrite(DSP_stb, LOW);
      delayMicroseconds(1);
      //cmd_with_stb(gridSegments); // cmd 1 // SM1628C is a drive of 7 grids
      cmd_with_stb(0b01000000);//To set auto-increment
      cmd_with_stb(0b10000000); // cmd 2 //Normal operation; Set pulse as 1/16
             
      digitalWrite(DSP_stb, LOW);
      delayMicroseconds(1);
      cmd_without_stb(0b11000000); // First register in memory position, auto-increment after write is set!
      //               
      cmd_without_stb(0b00000000); //DIG1 -defabcg
      cmd_without_stb(0b00000000); //DIG1 --------
      //                              
      cmd_without_stb(0b00000000); //DIG2 -defabcg
      cmd_without_stb(0b00000000); //DIG2 --------
      //
      cmd_without_stb(0b00000000); //DIG3 -, Pause, Play, CD, DVD, ":", , ,
      cmd_without_stb(0b00000000); //DIG3 --------
      //
      cmd_without_stb(0b00000000); //DIG4 -defabcg
      cmd_without_stb(0b00000000); //DIG4 --------
      //
      cmd_without_stb(0b00000000); //DIG5 -defabcg
      cmd_without_stb(0b00000000); //DIG5 --------
      //
      cmd_without_stb(0b00000000); //DIG6 -defabcg
      cmd_without_stb(0b00000000); //DIG6 --------
      //
      cmd_without_stb(0b00000000); //DIG7 -defabcg
      cmd_without_stb(0b00000000); //DIG7 --------
      //
      digitalWrite(DSP_stb, HIGH);
      cmd_with_stb((0b10001000) | 7); //cmd 4
      delay(200);
}
void clear_VFD(void){
  /*
  Here I clean all registers 
  Could be done only on the number of grid
  to be more fast. The 7 * 2 bytes = 14 registers
  */
      for (int n=0; n < 14; n++){  // important be 14, if not, bright the half of wells./this on the VFD of 6 grids)
        //cmd_with_stb(gridSegments); // cmd 1 // SM1628C is fixed to 7 grids
      cmd_with_stb(0b01000000);//To set auto-increment
      cmd_with_stb(0b10000000); //cmd 2 //Normal operation; Set pulse as 1/16
      digitalWrite(DSP_stb, LOW);
      delayMicroseconds(1);
            cmd_without_stb((0b11000000) | n); // cmd 3 //wich define the start address (00H to 15H)
            cmd_without_stb(0b00000000); // Data to fill table of 7 grids * 15 segm = 80 bits on the table
            //
            //cmd_with_stb((0b10001000) | 7); //cmd 4
            digitalWrite(DSP_stb, HIGH);
            delayMicroseconds(1);
     }
}
void setup() {
  // put your setup code here, to run once:

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  seconds = 0x00;
  minutes =0x00;
  hours = 0x00;

  /*CS12  CS11 CS10 DESCRIPTION
  0        0     0  Timer/Counter1 Disabled 
  0        0     1  No Prescaling
  0        1     0  Clock / 8
  0        1     1  Clock / 64
  1        0     0  Clock / 256
  1        0     1  Clock / 1024
  1        1     0  External clock source on T1 pin, Clock on Falling edge
  1        1     1  External clock source on T1 pin, Clock on rising edge
  */
  // initialize timer1 
  cli();           // disable all interrupts
  //initialize timer1 
  //noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;// This initialisations is very important, to have sure the trigger take place!!!
  
  TCNT1  = 0;
  
  // Use 62499 to generate a cycle of 1 sex 2 X 0.5 Secs (16MHz / (2*256*(1+62449) = 0.5
  OCR1A = 62498;            // compare match register 16MHz/256/2Hz
  //OCR1A = 1500; // only to use in test, increment seconds to fast!
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= ((1 << CS12) | (0 << CS11) | (0 << CS10));    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

  // Note: this counts is done to a Arduino 1 with Atmega 328... Is possible you need adjust
  // a little the value 62499 upper or lower if the clock have a delay or advance on hours.
    
  //  a=0x33;
  //  b=0x01;

  CLKPR=(0x80);
  //Set PORT
  DDRD = 0xFF;  // IMPORTANT: from pin 0 to 7 is port D, from pin 8 to 13 is port B
  PORTD=0x00;
  DDRB =0xFF;
  PORTB =0x00;

  SM1628C_init();

  clear_VFD();

  //only here I active the enable of interrupts to allow run the test of VFD
  //interrupts();             // enable all interrupts
  sei();
}
/******************************************************************/
/************************** Update Clock **************************/
/******************************************************************/
void send_update_clock(void){
    if (secs >=60){
      secs =0;
      minutes++;
      minute++;
    }
    if (minutes >=60){
      minutes =0;
      minute =0;
      hours++;
    }
    if (hours >=24){
      hours =0;
    }
    //*************************************************************
    numberA0=(secs%10);
    numberB0=(secs/10);
    //*************************************************************
    numberC0=(minute%10);
    numberD0=(minute/10);
    //**************************************************************
    numberE0=(hours%10);
    numberF0=(hours/10);
    //**************************************************************
}
void updateDisplay(void){
    delayMicroseconds(1);
    //cmd_with_stb(gridSegments); // cmd 1 // SM1628C is a driver of 7 grids
    cmd_with_stb(0b01000000);//To set auto-increment
    cmd_with_stb(0b10000000); // cmd 2 //Normal operation; Set pulse as 1/16
    digitalWrite(DSP_stb, LOW);
    delayMicroseconds(1);
    cmd_without_stb(0b11000000); // Grids of display... they have done the swap of this pins with segments
    //                    
    digitalWrite(DSP_stb, LOW);
    delayMicroseconds(1);
    // Is importante fill the next three lines, to write the minutes at firt digit at left
    // also is possible write direct to the position of the register of digit.
    cmd_without_stb(0b00000000);  //DIG1 --------
    cmd_without_stb(0b00000000);  //DIG1 --------
    //
    cmd_without_stb(0b00000000);  //DIG2 --------
    cmd_without_stb(0b00000000);  //DIG2 --------
    //
    cmd_without_stb(0b00000100);  //DIG3 -, Pause, Play, CD, DVD, ":", , ,
    cmd_without_stb(0b00000000);  //DIG3 --------
    // Please select the par of lines you want use, Hors & Minutes or Seconds & Minutes because we only have four digits.
    // Hours dozens
    // cmd_without_stb(segHexNumber[numberF0][0]);  //This will represent the "number" created at position from 0~16 Byte HL & HU
    // cmd_without_stb(segHexNumber[numberF0][1]);  //This will represent the "number" created at position from 0~16 Byte HL & HU
    // Hours units
    // cmd_without_stb(segHexNumber[numberE0][0]);  //This will represent the "number" created at position from 0~16 Byte HL & HU
    // cmd_without_stb(segHexNumber[numberE0][1]);  //This will represent the "number" created at position from 0~16 Byte HL & HU
    // Minutes dozens
    cmd_without_stb(segHexNumber[numberD0][0]);  //This will represent the "number" created at position from 0~16 Byte HL & HU
    cmd_without_stb(segHexNumber[numberD0][1]);  //This will represent the "number" created at position from 0~16 Byte HL & HU
    // Minutes units
    cmd_without_stb(segHexNumber[numberC0][0]);  //This will represent the "number" created at position from 0~16 Byte HL & HU
    cmd_without_stb(segHexNumber[numberC0][1]);  //This will represent the "number" created at position from 0~16 Byte HL & HU
    // Seconds dozens
    cmd_without_stb(segHexNumber[numberB0][0]);  //This will represent the "number" created at position from 0~16 Byte HL & HU
    cmd_without_stb(segHexNumber[numberB0][1]);  //This will represent the "number" created at position from 0~16 Byte HL & HU
    // Seconds units
    cmd_without_stb(segHexNumber[numberA0][0]);  //This will represent the "number" created at position from 0~16 Byte HL & HU
    cmd_without_stb(segHexNumber[numberA0][1]);  //This will represent the "number" created at position from 0~16 Byte HL & HU
    //
    //Because the panel only have 4 digits, I need make the option by Seconds & Minutes, or Minutes & Hours!
    digitalWrite(DSP_stb, HIGH);
    cmd_with_stb((0b10001000) | 7); //cmd 4
    delay(5);
}
void adjustHMS(){
 // Important is necessary put a pull-up resistor to the VCC(+5VDC) to this pins (3, 4, 5)
 // if dont want adjust of the time comment the call of function on the loop
  /* Reset Seconds to 00 Pin number 3 Switch to GND*/
    if((AdjustPins & 0x08) == 0 )
    {
      _delay_ms(200);
      secs=00;
    }
    
    /* Set Minutes when SegCntrl Pin 4 Switch is Pressed*/
    if((AdjustPins & 0x10) == 0 )
    {
      _delay_ms(200);
      if(minutes < 59)
      minutes++;
      else
      minutes = 0;
    }
    /* Set Hours when SegCntrl Pin 5 Switch is Pressed*/
    if((AdjustPins & 0x20) == 0 )
    {
      _delay_ms(200);
      if(hours < 23)
      hours++;
      else
      hours = 0;
    }
}
void readButtons(){
  int inPin = 7;     // pushbutton connected to digital pin 7
  int val = 0;       // variable to store the read value
  
  byte array[8] = {0,0,0,0,0,0,0,0};

  unsigned char mask = 1; //our bitmask

  array[0] = 1;

  digitalWrite(DSP_stb, LOW);
  delayMicroseconds(2);
  cmd_without_stb(0b01000010); // cmd 2 //10=Read Keys; 00=Wr DSP;
  delayMicroseconds(2);
      
  pinMode(7, INPUT);  // Important this point! Here I'm changing the direction of the pin to INPUT data.
  delayMicroseconds(2);
  //PORTD != B01010100; // this will set only the pins you want and leave the rest alone at
  //their current value (0 or 1), be careful setting an input pin though as you may turn 
  //on or off the pull up resistor  
  //This don't send the strobe signal, to be used in burst data send
         for (int z = 0; z < 4; z++){
             //for (mask=0b00000001; mask > 0; mask <<= 1) { //iterate through bit mask
                   for (int h =8; h > 0; h--) {
                      digitalWrite(DSP_clk, HIGH);  // Remember wich the read data happen when the clk go from LOW to HIGH! Reverse from write data to out.
                      delayMicroseconds(2);
                     val = digitalRead(inPin);
                           if (val & mask){ // if bitwise AND resolves to true
                             //Serial.print(val);
                            //data =data | (1 << mask);
                            array[h] = 1;
                           }
                           else{ //if bitwise and resolves to false
                            //Serial.print(val);
                           // data = data | (1 << mask);
                           array[h] = 0;
                           }
                    digitalWrite(DSP_clk, LOW);
                    delayMicroseconds(2);
                   } 
             
              Serial.print(z);  // All the lines of print is only used to debug, comment it, please!
              Serial.print(" - " );
                        
                                  for (int bits = 7 ; bits > -1; bits--) {
                                      Serial.print(array[bits]);
                                   }                   
                        if (z==0){
                            if(array[5] == 1){
                             flagSecs = !flagSecs;  // This change the app to hours or seconds
                            }
                        }
                        
                        if (z==0){
                            if(array[7] == 1){
                              digitalWrite(10, !digitalRead(10));
                          }
                        }
                        
                        if (z==3){
                           if(array[7] == 1){
                             //digitalWrite(VFD_onRed, !digitalRead(VFD_onRed));
                             //digitalWrite(VFD_onGreen, !digitalRead(VFD_onGreen));
                            }
                          }                        
                  Serial.println();
          }  // End of "for" of "z"
      Serial.println();  // This line is only used to debug, please comment it!

  digitalWrite(DSP_stb, HIGH);
  delayMicroseconds(2);
  cmd_with_stb((0b10001000) | 7); //cmd 4
  delayMicroseconds(2);
  pinMode(7, OUTPUT);  // Important this point! Here I'm changing the direction of the pin to OUTPUT data.
  delay(1); 
}
void loop() {
  // You can comment untill while cycle to avoid the test running.
  test_symbols_DSP();
  delay(1500);
  test_hexadecimal_DSP();
  delay(100);
  clear_VFD();
  delay(200);
  // Here we are making the update of display four times by second in the static mode!
       while(1){
                send_update_clock();
                delay(100);
                readButtons(); //Uncomment if want use buttons on panel, proceed with Id position!
                delay(100);
                updateDisplay();
       }
}
ISR(TIMER1_COMPA_vect)   {  //This is the interrupt request
      secs++;
} 
