// Program to simulate a spindel for testing lathe G33 (threading)
// It generatates Low pulses as if NO NPN sensors where connected.
// When no sensors are connected, all should work
//

#define MinRPM 30L        //   30 RPM Speed slowest
#define MaxRPM 3000L      // 3000 RPM Speed fastest

#define PotMeter A0       //Potmeter pin, must be analog, not A4 or A5 on arduino (SDA SCL)
#define MinAdc 50L        //add dead band on start
#define MaxAdc 975L       //add dead band on end

#define PulsesPerRevolution 4L
#define PulseLowMin 1L

#define PulseLevel LOW       // Use LOW for Low level pulse (NO), HIGH for High level pulse (NC)
                        
#if PulseLevel == LOW
  #define NoPulseLevel HIGH
#else
  #define NoPulseLevel LOW
#endif

#define IndexPin 0
#define SyncPin 2
#define LedPin 1

//#define DebugInfo
//define one of the fixed speed simulation setting or define none and use a potmeter connected to A0
//#define FixSpeed30RPM
//#define FixSpeed120RPM
//#define FixSpeed600RPM

long ReadPotmeter()
{
  return (long) analogRead(PotMeter);
}

long MapPotToDelay( long Pot)
{
  unsigned long Delay = map(Pot, 0, 1024, 499.0, 11.5); // 10 bit adc, 30 RPM to 1200 RPM, 4 index pulses
  if (Delay < 11) Delay = 11;   // 30 RPM
  if (Delay>499) Delay=249;     // 1200 RPM
  return Delay;
}

void setup()
{
  pinMode(IndexPin, OUTPUT);   // Initialize the pin as an output
  pinMode(SyncPin, OUTPUT);    // Initialize the  pin as aput
  pinMode(LedPin, OUTPUT);     // Initialize the  pin as aput
}

int GetPulseTime()
{
  return MapPotToDelay(ReadPotmeter());
}

void loop()
{
  long PulseTime;
  for (;;)
  {
    PulseTime = GetPulseTime();
    for (int i = 0; i < PulsesPerRevolution; i++)
    {
      if (i == 0)
      {
        digitalWrite(IndexPin, PulseLevel);   // Set pulse level on the first sync pulse
        digitalWrite(LedPin, HIGH);           // Set the led pin high
      }
      digitalWrite(SyncPin, PulseLevel);      // Turn the sync pin high
      delay(PulseLowMin);                     // Wait for the pulse high time
      digitalWrite(SyncPin, NoPulseLevel);    // Turn the sync pin Low
      digitalWrite(IndexPin, NoPulseLevel);   // Turn the index pin Low
      digitalWrite(LedPin, LOW);              // Turn the index pin Low
      delay(PulseTime);                       // Wait for the pulse low time
    }
#ifdef DebugInfo
    ShowInfo();
#endif
  }
}
