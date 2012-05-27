const uint8_t SENSOR_PIN = A2;
const int RESOLUTION = 50;
const int DELAY = 10;

int pressure = 0;
int zerodata = 0;
char input = ' ';

void setup()
{
  Serial.begin(9600);
  pinMode(SENSOR_PIN, INPUT);
}

void loop()
{
  pressure = 0;
  for (int x=0; x<RESOLUTION; x++)
  {
    pressure = pressure + analogRead(SENSOR_PIN);
    delay(DELAY);
  }
  
  pressure = pressure / RESOLUTION;
  
  if (Serial.available())
  {
    input = Serial.read();
  }
  
  if (input == 'z')
  {
    zerodata = pressure;
  }
  
  input = ' ';
  
  //Serial.println(String(pressure) + " - " + String(zerodata) + " = " + String(pressure - zerodata));
  //Serial.println(dataToPressure());
  //Serial.println(dataToDepth());
  Serial.print("Raw: ");
  Serial.print(pressure);
  Serial.print(" \tDepth: ");
  Serial.print(dataToDepth());
  Serial.print("\tPressure: ");
  Serial.println(dataToPressure());
}

float dataToDepth()
{
  return 0.0157 * (pressure - zerodata) + 0.4;
}

float dataToPressure()
{
  return 0.148 * (pressure - zerodata);
}
