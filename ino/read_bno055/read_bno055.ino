#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>


#define BNO055_SAMPLERATE_DELAY_MS (25)

Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    myIMU.begin();
    delay(1000);
    int8_t temp = myIMU.getTemp();
    myIMU.setExtCrystalUse(true);
}

void loop() {
    // put your main code here, to run repeatedly:
    float theta;
    float phi;

    uint8_t systemCal, gyroCal, accelCal, magCal = 0;

    const float G = 9.81;

    myIMU.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);

    // read accelerometer values in m/s^2
    imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    acc[0]= acc[0] / G; acc[1]= acc[1] / G; acc[2]= acc[2] / G;

    // read gyroscope values in rps, radians per second
    imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    // read magnetometer values in uT, micro Teslas
    imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    SerializeData(&acc.x(),  &acc.y(),  &acc.z(),
                  &gyro.x(), &gyro.y(), &gyro.z(),
                  &mag.x(),  &mag.y(),  &mag.z(),
                  &systemCal, &gyroCal, &accelCal, &magCal);

    delay(BNO055_SAMPLERATE_DELAY_MS);
}

/* This function serializes the data
 *  - gyro x3 floats
 *  - acc x3 floats (normalized to g)
 *  - magnetometer x3 floats
 *  - calibration bytes
 */
void SerializeData(double* accX,  double* accY,  double* accZ,
                   double* gyroX, double* gyroY, double* gyroZ,
                   double* magX,  double* magY,  double* magZ,
                   uint8_t * systemCal, uint8_t * gyroCal,
                   uint8_t * accelCal, uint8_t * magCal
    ){

    const size_t buf_sz = 3*4*3 + 4;

    byte* bAccX = (byte*)(accX);
    byte* bAccY = (byte*)(accY);
    byte* bAccZ = (byte*)(accZ);

    byte* bGyroX = (byte*)(gyroX);
    byte* bGyroY = (byte*)(gyroY);
    byte* bGyroZ = (byte*)(gyroZ);

    byte* bMagX = (byte*)(magX);
    byte* bMagY = (byte*)(magY);
    byte* bMagZ = (byte*)(magZ);

    byte* bSystemCal = (byte*)(systemCal);
    byte* bGyroCal   = (byte*)(gyroCal);
    byte* bAccelCal  = (byte*)(accelCal);
    byte* bMagCal    = (byte*)(magCal);

    byte buf[buf_sz] =  {
        bGyroX[0], bGyroX[1], bGyroX[2], bGyroX[3],
        bGyroY[0], bGyroY[1], bGyroY[2], bGyroY[3],
        bGyroZ[0], bGyroZ[1], bGyroZ[2], bGyroZ[3],

        bAccX[0], bAccX[1], bAccX[2], bAccX[3],
        bAccY[0], bAccY[1], bAccY[2], bAccY[3],
        bAccZ[0], bAccZ[1], bAccZ[2], bAccZ[3],

        bMagX[0], bMagX[1], bMagX[2], bMagX[3],
        bMagY[0], bMagY[1], bMagY[2], bMagY[3],
        bMagZ[0], bMagZ[1], bMagZ[2], bMagZ[3],

        bSystemCal[0], bGyroCal[0], bAccelCal[0], bMagCal[0]
    };

    Serial.write(buf, buf_sz);
}
