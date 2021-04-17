void setupUltrasonic() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //Serial.begin(9600);
}

//in loop:
void ultrasonicCheck() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH,80000);
  ultrasonic_distance= (duration*0.0343)/2;//ultrasonic_distance
  if(ultrasonic_distance>126)
  {
    ultrasonic_distance=126;
  }
}
