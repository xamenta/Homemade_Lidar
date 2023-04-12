#include <Arduino.h>
#include "ComponentsFile.h"

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  StepperInitialize(1000.0, 900.0, 1000.0);
  TfInitialize();
  ServoInitialization();
  // Serial.println("Initialization Complete");
  delay(5000);
}

void loop()
{
  if (!both_run)
  {
    RunLoopOne();
  }
  else
  {
    RunLoopBoth();
  }
}
