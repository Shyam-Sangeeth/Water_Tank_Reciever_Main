class ZMPT101B
{
private:
  int analogPin = A0;
  double sensorValue;
  int val[100];
  int max_v = 0;
  double Veff = 0;
  double VmaxD = 0;
  double VeffD = 0;
  float measureVoltage()
  {
    for (int i = 0; i < 100; i++)
    {
      sensorValue = analogRead(analogPin);
      if (sensorValue > 520)
        val[i] = sensorValue;
      else
        val[i] = 0;
      delay(1);
    }
    max_v = 0;
    for (int i = 0; i < 100; i++)
    {
      if (val[i] > max_v)
        max_v = val[i];
      val[i] = 0;
    }
    if (max_v != 0)
    {
      VmaxD = max_v;
      VeffD = VmaxD / sqrt(2);
      Veff = (((VeffD - 420.76) / -90.24) * -210.2) + 210.2;
    }
    else
    {
      Veff = 0;
    }
    VmaxD = 0;
    return Veff;
  }

public:
  ZMPT101B(int sensorPin)
  {
    analogPin = sensorPin;
    pinMode(analogPin, INPUT);
  }
  float acVoltage()
  {
    return measureVoltage();
  }
};