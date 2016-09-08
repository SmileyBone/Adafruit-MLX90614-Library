/*************************************************** 
  This is a library example for changing the address of the MLX90614 Temp Sensor
  
  Written by Zach Lovett
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_MLX90614.h>

//Put the new address for the sensor here (example is 0x11)
#define NEW_SENSOR_ADDRESS 0x11

Adafruit_MLX90614 mlx = Adafruit_MLX90614(0);

bool success = false;

void setup() {
  Serial.begin(115200);

  Serial.println("Adafruit MLX90614 Address change");  

  mlx.begin();
  success = mlx.setAddress(NEW_SENSOR_ADDRESS);

  if(success){
    Serial.print("Address changed to ");
    Serial.println(NEW_SENSOR_ADDRESS);
  }
  else{
    Serial.print("Address change failed.");
  }
}

void loop() {
  delay(500);
}
