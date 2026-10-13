void setup()
{
  pinMode(12,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(10,OUTPUT);
 Serial.begin(9600);
}

void loop()
{
 Serial.println("RED");
 digitalWrite(12,HIGH);
 delay(10000);
 digitalWrite(12,LOW);
 Serial.println("YELLOW");
 digitalWrite(11,HIGH);
 delay(3000);
 digitalWrite(11,LOW);
 Serial.println("GREEN");
 digitalWrite(10,HIGH);
 delay(7000);
 digitalWrite(10,LOW);
}