// Program to simulate a spindel for testing lathe G33 (threading)
// It generatates Low pulses as if NO NPN sensors where connected.
// When no sensors are connected, all should work
// 

#define MinRPM 30UL        //   30 RPM Speed slowest
#define MaxRPM 3000UL      // 3000 RPM Speed fastest

#define PotMeter A0       //Potmeter pin, must be analog, not A4 or A5 on arduino (SDA SCL)
#define MinAdc 50UL           //add dead band on start
#define MaxAdc 975UL          //add dead band on end

#define PulsesPerRevolution 4UL
#define PulseLowMin 1UL

#define IndexPin 13
#define SyncPin 12

//#define DebugInfo
//define one of the fixed speed simulation setting or define none and use a potmeter connected to A0
//#define FixSpeed30RPM
//#define FixSpeed120RPM
//#define FixSpeed600RPM

int ReadPotmeter()
{
  return analogRead(PotMeter);
}

unsigned long CalculateDelayTime(unsigned long RPM)
{
  return  60000UL / (RPM * PulsesPerRevolution );
}

unsigned long MapPotToRPM(unsigned long Pot)
{
  unsigned long RPM = map(Pot, MinAdc, MaxAdc, MinRPM, MaxRPM);
  if (RPM < MinRPM) return MinRPM;
  if (RPM > MaxRPM) return MaxRPM;
  return RPM;
}

unsigned long MapRPMtoDelay(unsigned long RPM)
{
  if (RPM < MinRPM) RPM = MinRPM;
  if (RPM > MaxRPM) RPM = MaxRPM;
  return  CalculateDelayTime(RPM);
}

unsigned long MapPotToDelay(unsigned long Pot)
{
  if (Pot < MinAdc) Pot = MinAdc;
  if (Pot > MaxAdc) Pot = MaxAdc;
  return MapRPMtoDelay(MapPotToRPM(Pot));
}

void ShowInfo()
{
#ifdef DebugInfo
  unsigned long ADC;
  ADC = (unsigned long) ReadPotmeter();
  Serial.print("ADC: "); Serial.print(ADC);
  Serial.print(" RPM: "); Serial.print(MapPotToRPM(ADC));
  Serial.print(" Sync Pulse Delay: "); Serial.print(MapPotToDelay(ADC));
  Serial.println();
#endif
}

void setup()
{
#ifdef DebugInfo
  Serial.begin(115200);
#endif
  pinMode(IndexPin, OUTPUT);     // Initialize the pin as an output
  pinMode(SyncPin, OUTPUT);     // Initialize the  pin as aput
}

int GetPulseTime()
{
#if defined FixSpeed30RPM
  return MapRPMtoDelay(30);
#elif defined FixSpeed120RPM
  return MapRPMtoDelay(120);
#elif defined FixSpeed600RPM
  return MapRPMToDelay(600);
#else
  return MapPotToDelay(ReadPotmeter());
#endif 
}

void loop()
{
  int PulseTime;
  int PulseLow;
  int PulseHigh;
  for (;;)
  {
    for (int i = 0; i < PulsesPerRevolution; i++)
    {
      PulseTime = GetPulseTime();
      PulseTime -= PulseLowMin;
      if (i == 0) digitalWrite(IndexPin, LOW); // Turn the index pin high on the first sync pulse
      else digitalWrite(IndexPin, HIGH);         // Turn the index pin low on all other sync pulses
      digitalWrite(SyncPin, LOW);              // Turn the sync pin high
      delay(PulseLowMin);                      // Wait for the pulse high time
      digitalWrite(SyncPin, HIGH);               // Turn the sync pin Low
      digitalWrite(IndexPin, HIGH);              // Turn the index pin Low
      delay(PulseTime);                         // Wait for the pulse low time
    }
#ifdef DebugInfo
    ShowInfo();
#endif
  }
}
