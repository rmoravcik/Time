
#include <TimeLib.h>

bool newSecondFireFlag = false;              // rtc new second fire flag
unsigned long pitCounter = 0;                // pit counter

void setup() {

  InternalRTC.attachClockInterrupt(rtc_irq); // set user IRQ function fired each new second by internal RTC
                                             // run only on megaAVR-0 series!
                                             // note : use InternalRTC.detachClockInterrupt() for detach IRQ function

  InternalRTC.attachInterrupt(pit_irq, 16);  // set user IRQ function fired by internal periodic interrupt (PIT)
                                             // (1Hz by defaut, max 8192Hz, only power of 2 number)
                                             // run only on megaAVR-0 series!
                                             // note : use InternalRTC.detachInterrupt() for detach IRQ function
                                           
  setTime(8,15,00,24,04,1981);               // set the current system timestamp from arbitrary date/time
                                             // the next second was fire exactly 1 second after the call at this function

  Serial.begin(9600);                        // init serial
  pinMode(LED_BUILTIN, OUTPUT);              // init built-in led
  
}

void pit_irq() { // executed all pediodic interrupt
     
  pitCounter++; // increment pit counter
 
}

void rtc_irq() { // executed at each RTC second interrupt
     
  newSecondFireFlag = true; // fire flag
 
}

void loop() {

  if( newSecondFireFlag ) { // make actions if flag is fired:

    newSecondFireFlag = false; // unfire flag

    unsigned long actualTime = now(); // get actual system timestamp

    // display current system date and time:
    Serial.print(year(actualTime));
    Serial.print("-");
    Serial.print(month(actualTime));
    Serial.print("-");
    Serial.print(day(actualTime));
    Serial.print(" ");
    Serial.print(hour(actualTime));
    Serial.print(":");
    Serial.print(minute(actualTime));
    Serial.print(":");
    Serial.print(second(actualTime));

    // display PIT counter at the end of line:
    Serial.print(" -> ");
    Serial.print(pitCounter);
    Serial.println(" PIT cycles.");

    // blink led:
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);

  }

}
