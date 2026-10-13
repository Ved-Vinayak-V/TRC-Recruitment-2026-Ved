// C++ code
//
void setup()
{
  pinMode(A0, INPUT);
  pinMode(2,OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  int k=analogRead(A0);
  if( k<400){
    digitalWrite(2,HIGH);
  }
  else{
    digitalWrite(2,LOW);
  }
}