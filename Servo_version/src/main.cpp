#include <Arduino.h>
#include "ComponentsFile.h"

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  TfInitialize();
  ServoInitialization();
  // Serial.println("Initialization Complete");
  delay(1000);
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
