void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main/home/charlie/Downloads/arduino-ide_2.3.2_Linux_64bit/arduino-ide code here, to run repeatedly:
  float speed_num = (float) analogRead(A0);
  float hazmat_num = (float) analogRead(A1);
  String reverse = (String)(bool) digitalRead(7);
  String reset_arm = (String)(bool) digitalRead(9);
  String toggle = (String)(bool) digitalRead(8);

  String speed_percent = "";
  if ((int)(((speed_num/1013)*10000)/70) > 100){
    speed_percent = "100";
  } else {
    speed_percent = (String)(int)(((speed_num/1013)*10000)/70);
  }

  String hazmat_percent = "";
  if ((int)(((hazmat_num/1013)*10000)/70) > 100){
    hazmat_percent = "100";
  } else {
    hazmat_percent = (String)(int)(((hazmat_num/1013)*10000)/70);
  }


  Serial.println(speed_percent + " " + reverse + " " + hazmat_percent + " " + reset_arm + " " + toggle);
  // Serial.println(98);
  delay(10);
}
