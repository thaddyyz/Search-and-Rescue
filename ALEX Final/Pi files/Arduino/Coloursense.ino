
void colsetup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  analogWrite(A5,LOW);
  digitalWrite(A4,0);
  //Serial.begin(9600);
}
//if red>green and blue>red = green if green>blue and blue>red=red 
void colread() {
  // Setting red filtered photodiodes to be read
  red_total = 0;
  green_total = 0;
  blue_total = 0;
  for (x = 0; x < 3; x++) {
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    frequency = pulseIn(sensorOut, LOW,45000);
    red_total = red_total + frequency;
    delay(8);

    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    frequency = pulseIn(sensorOut, LOW,45000);
    green_total = green_total + frequency;   
    delay(8);

    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    frequency = pulseIn(sensorOut, LOW,45000);
    blue_total = blue_total + frequency;
    delay(8);
  }
  //red = red_total/5;
  //green = green_total/5;
  //blue = blue_total/5;
  
  
  if (blue_total > green_total && green_total > red_total)
  {
    //Serial.println("green");
    col_reading = 2;
  }
  else if (green_total > blue_total && blue_total > red_total)
  {
    //Serial.println("red");
    col_reading = 1;
  }
  else
  {
    //Serial.println("none");
    col_reading = 0; 
  }
  if(ultrasonic_distance>=7)
  {
    col_reading = 3;
  }

}
