void watchObjects(int pin){
  boolean pinState = digitalRead(pin);
  Serial.print("Pin: "+String(pin)+" is: ");
  if (pinState){
    Serial.println("TRUE");
  }else{
    Serial.println("FALSE");
  }
}
