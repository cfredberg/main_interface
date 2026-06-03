#include "Adafruit_MLX90393.h"

Adafruit_MLX90393 sensor = Adafruit_MLX90393();
#define MLX90393_CS 10

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if (! sensor.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
  //if (! sensor.begin_SPI(MLX90393_CS)) {  // hardware SPI mode
    // Serial.println("No sensor found ... check your wiring?");
    while (1) { delay(10); }
  }
  // Serial.println("Found a MLX90393 sensor");

  sensor.setGain(MLX90393_GAIN_1X);
  // You can check the gain too
  // Serial.print("Gain set to: ");
  // switch (sensor.getGain()) {
  //   case MLX90393_GAIN_1X: Serial.println("1 x"); break;
  //   case MLX90393_GAIN_1_33X: Serial.println("1.33 x"); break;
  //   case MLX90393_GAIN_1_67X: Serial.println("1.67 x"); break;
  //   case MLX90393_GAIN_2X: Serial.println("2 x"); break;
  //   case MLX90393_GAIN_2_5X: Serial.println("2.5 x"); break;
  //   case MLX90393_GAIN_3X: Serial.println("3 x"); break;
  //   case MLX90393_GAIN_4X: Serial.println("4 x"); break;
  //   case MLX90393_GAIN_5X: Serial.println("5 x"); break;
  // }

  sensor.setResolution(MLX90393_X, MLX90393_RES_17);
  sensor.setResolution(MLX90393_Y, MLX90393_RES_17);
  sensor.setResolution(MLX90393_Z, MLX90393_RES_16);

  // Set oversampling
  sensor.setOversampling(MLX90393_OSR_3);

  // Set digital filtering
  sensor.setFilter(MLX90393_FILTER_5);
}

void loop() {
  // put your main/home/charlie/Downloads/arduino-ide_2.3.2_Linux_64bit/arduino-ide code here, to run repeatedly:
  float speed_num = (float) analogRead(A0);
  String reverse = (String)(bool) digitalRead(7);
  String speed_percent = (String)(int)((speed_num/1013)*100);

  float x, y, z;
  // if (sensor.readData(&x, &y, &z)) {
  //     Serial.print("X: "); Serial.print(x, 4); Serial.println(" uT");
  //     Serial.print("Y: "); Serial.print(y, 4); Serial.println(" uT");
  //     Serial.print("Z: "); Serial.print(z, 4); Serial.println(" uT");
  // } else {
  //     Serial.println("Unable to read XYZ data from the sensor.");
  // }

// --- I can't print to serial, but maybe implement led status??? ---
  sensor.readData(&x, &y, &z)



  Serial.println(speed_percent + " " + reverse + " [" + x + "," + y + "," + z + "]");
  // Serial.println(98);
  delay(10);
}
