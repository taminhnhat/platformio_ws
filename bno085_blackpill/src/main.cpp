#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.
// Note sensorValue.status gives calibration accuracy (which improves over time)
#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS PC13
#define BNO08X_INT PC14

// #define FAST_MODE

// For SPI mode, we also need a RESET
// #define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

struct euler_t
{
  float yaw;
  float pitch;
  float roll;
} ypr;

struct vector4Double
{
  double x;
  double y;
  double z;
  double w;
};
struct vector3Double
{
  double x;
  double y;
  double z;
};
struct sensors
{
  vector4Double orientation;
  vector3Double angular_velocity;
  vector3Double linear_acceleration;
  vector3Double magnetic;
  double temperature;
} ros2_sensor;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
// SH2_GYROSCOPE_CALIBRATED SH2_LINEAR_ACCELERATION SH2_MAGNETIC_FIELD_CALIBRATED
long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval)
{
  Serial1.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, report_interval))
  {
    Serial1.println("Could not enable stabilized remote vector");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, report_interval))
  {
    Serial1.println("Could not enable gyroscope calibrated");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, report_interval))
  {
    Serial1.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, report_interval))
  {
    Serial1.println("Could not enable magnetic field calibrated");
  }
}

void setup(void)
{

  Serial1.begin(115200);
  Serial1.println("serial connected");
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial1.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C())
  {
    // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
    // if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial1.println("Failed to find BNO08x chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial1.println("BNO08x Found!");

  setReports(reportType, reportIntervalUs);

  Serial1.println("Reading events");
  delay(100);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees = false)
{

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees)
  {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool degrees = false)
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t *rotational_vector, euler_t *ypr, bool degrees = false)
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void loop()
{

  if (bno08x.wasReset())
  {
    Serial1.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue))
  {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId)
    {
    case SH2_ARVR_STABILIZED_RV:
      ros2_sensor.orientation.x = sensorValue.un.arvrStabilizedRV.i;
      ros2_sensor.orientation.y = sensorValue.un.arvrStabilizedRV.j;
      ros2_sensor.orientation.z = sensorValue.un.arvrStabilizedRV.k;
      ros2_sensor.orientation.w = sensorValue.un.arvrStabilizedRV.real;
      quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      ros2_sensor.angular_velocity.x = sensorValue.un.gyroscope.x;
      ros2_sensor.angular_velocity.y = sensorValue.un.gyroscope.y;
      ros2_sensor.angular_velocity.z = sensorValue.un.gyroscope.z;
      break;
    case SH2_LINEAR_ACCELERATION:
      ros2_sensor.linear_acceleration.x = sensorValue.un.linearAcceleration.x;
      ros2_sensor.linear_acceleration.y = sensorValue.un.linearAcceleration.y;
      ros2_sensor.linear_acceleration.z = sensorValue.un.linearAcceleration.z;
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      ros2_sensor.magnetic.x = sensorValue.un.magneticField.x;
      ros2_sensor.magnetic.y = sensorValue.un.magneticField.y;
      ros2_sensor.magnetic.z = sensorValue.un.magneticField.z;
      break;
    default:
      break;
    }

    Serial1.print("yaw="); // This is accuracy in the range of 0 to 3
    Serial1.print(ypr.yaw);
    Serial1.print(",pitch=");
    Serial1.print(ypr.pitch);
    Serial1.print(",roll=");
    Serial1.println(ypr.roll);

    Serial1.print("qx=");
    Serial1.print(ros2_sensor.orientation.x);
    Serial1.print(",qy=");
    Serial1.print(ros2_sensor.orientation.y);
    Serial1.print(",qz=");
    Serial1.print(ros2_sensor.orientation.z);
    Serial1.print(",qw");
    Serial1.print(ros2_sensor.orientation.w);

    Serial1.print(",gx=");
    Serial1.print(ros2_sensor.angular_velocity.x);
    Serial1.print(",gy=");
    Serial1.print(ros2_sensor.angular_velocity.y);
    Serial1.print(",gz=");
    Serial1.print(ros2_sensor.angular_velocity.z);

    Serial1.print(",ax=");
    Serial1.print(ros2_sensor.linear_acceleration.x);
    Serial1.print(",ay=");
    Serial1.print(ros2_sensor.linear_acceleration.y);
    Serial1.print(",az=");
    Serial1.print(ros2_sensor.linear_acceleration.z);

    Serial1.print(",mx=");
    Serial1.print(ros2_sensor.magnetic.x);
    Serial1.print(",my=");
    Serial1.print(ros2_sensor.magnetic.y);
    Serial1.print(",mz=");
    Serial1.print(ros2_sensor.magnetic.z);
    Serial1.println("");
  }
}