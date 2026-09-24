void setup()
{
  pinMode(A0,INPUT);
  Serial.begin(9600);
}

void loop()
{
   float  k=analogRead(A0);
  float v=(k*5.0)/1023.0;
  float t=(v-0.5)*100;
  if (t>40){
  Serial.println(t);
    Serial.println("->Fan ON");
    delay(1000);}
  else if( t<25){
    Serial.println(t);
    Serial.println("->Fan OFF");
    delay(1000);}
   else{
     Serial.println(t);
     Serial.println("->Fan STAND BY");
     delay(1000);}
}