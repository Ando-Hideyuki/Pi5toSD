#include <unistd.h>
#include "rp1-gpio.h"
 
#define PIN 26
 
RP1_GPIO rp1;
 
int main(void)
{
  rp1.begin();
  rp1.pinMode(PIN, rp1.OUTPUT);
  while (1)
  {
    rp1.digitalWrite(PIN, rp1.TOGGLE);//LOW HIGH TOGGLE
    //usleep(500000); // wait 0.5 Sec
    //usleep(1);
  }
  rp1.end();
  return 0;
}
