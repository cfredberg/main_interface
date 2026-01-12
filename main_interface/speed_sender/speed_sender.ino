void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main/home/charlie/Downloads/arduino-ide_2.3.2_Linux_64bit/arduino-ide code here, to run repeatedly:
  float speed_num = (float) analogRead(A0);
  String reverse = (String)(bool) digitalRead(7);
  String speed_percent = (String)(int)((speed_num/1013)*100);

  Serial.println(speed_percent + " " + reverse);
  // Serial.println(98);
  delay(10);
}
