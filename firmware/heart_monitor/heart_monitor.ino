#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4; // base averaging
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute;
int beatAvg;

// IR filter (median + moving average)
const int MEDIAN_SIZE = 7;
const int AVG_SIZE = 4;

long medianBuffer[MEDIAN_SIZE];
int medianIndex = 0;

long avgBuffer[AVG_SIZE];
int avgIndex = 0;

long filterIR(long newValue)
{
  // Median
  medianBuffer[medianIndex] = newValue;
  medianIndex = (medianIndex + 1) % MEDIAN_SIZE;

  long sorted[MEDIAN_SIZE];
  for (int i = 0; i < MEDIAN_SIZE; i++) sorted[i] = medianBuffer[i];

  for (int i = 0; i < MEDIAN_SIZE - 1; i++)
  {
    for (int j = i + 1; j < MEDIAN_SIZE; j++)
    {
      if (sorted[j] < sorted[i])
      {
        long tmp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = tmp;
      }
    }
  }

  long medianVal = sorted[MEDIAN_SIZE / 2];

  // Moving Average
  avgBuffer[avgIndex] = medianVal;
  avgIndex = (avgIndex + 1) % AVG_SIZE;

  long sum = 0;
  for (int i = 0; i < AVG_SIZE; i++) sum += avgBuffer[i];

  return sum / AVG_SIZE;
}

// BPM stabilizer
const int BPM_AVG_SIZE = 5;
float bpmBuffer[BPM_AVG_SIZE];
int bpmIndex = 0;
bool stabilized = false;

float filterBPM(float bpm)
{
  static float avg = 0;

  // Reject impossible values
  if (bpm < 30 || bpm > 200) return avg;

  // Reject sudden jumps
  if (stabilized && abs(bpm - avg) > 30) return avg;

  // Add to buffer
  bpmBuffer[bpmIndex] = bpm;
  bpmIndex = (bpmIndex + 1) % BPM_AVG_SIZE;

  float sum = 0;
  for (int i = 0; i < BPM_AVG_SIZE; i++) sum += bpmBuffer[i];
  avg = sum / BPM_AVG_SIZE;

  if (!stabilized && bpmIndex == 0) stabilized = true;

  return avg;
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Initializing...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }

  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup();                     
  particleSensor.setPulseAmplitudeRed(0x1F);  // higher LED power for better signal
  particleSensor.setPulseAmplitudeGreen(0);   
}

void loop()
{
  long irValue = filterIR(particleSensor.getIR());

  if (checkForBeat(irValue) == true)
  {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    float bpm = 60 / (delta / 1000.0);
    beatsPerMinute = filterBPM(bpm);

    if (beatsPerMinute > 0)
    {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000) Serial.print(" No finger?");

  Serial.println();
}
