#include <unistd.h>
#include "rp1-gpio.h"
 
#define PIN 26
 
RP1_GPIO rp1;
 
int main(void)
{
  rp1.begin();

  //おまじない
  rp1.pinMode(22, rp1.OUTPUT);
  rp1.digitalWrite(22, rp1.LOW);//LOW HIGH TOGGLE
  rp1.pinMode(22, rp1.INPUT);
  
  rp1.pinMode(PIN, rp1.OUTPUT);
  rp1.pinMode(4, rp1.OUTPUT);
  rp1.pinMode(23, rp1.INPUT);
  
  while (1)
  {
    rp1.digitalWrite(4, rp1.LOW);//LOW HIGH TOGGLE
    rp1.digitalWrite(PIN, rp1.TOGGLE);//LOW HIGH TOGGLE
    usleep(500000); // wait 0.5 Sec
    rp1.digitalWrite(4, rp1.HIGH);//LOW HIGH TOGGLE
    usleep(500000); // wait 0.5 Sec
    //usleep(1);
  }
  rp1.end();
  return 0;
}
