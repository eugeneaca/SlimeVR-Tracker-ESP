/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain, S.J. Remington
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "mpu9250Ksensor.h"
#include "network/network.h"
#include "globals.h"
#include "helper_3dmath.h"
#include <i2cscan.h>
#include "calibration.h"
//#include "magneto1.4.h"
#include "mahony.h"
// #include "madgwick.h"
#if not (defined(_MAHONY_H_) || defined(_MADGWICK_H_))
//#include "dmpmag.h"
#endif
#include "ledmgr.h"

#include "quaternion.h"

#include "udplogger.h"
#define Serial logger
#define println print

//constexpr float gscale = (250. / 32768.0) * (PI / 180.0); //gyro default 250 LSB per d/s -> rad/s

#define SKIP_CALC_MAG_INTERVAL 10
#define MAG_CORR_RATIO 0.2

    void
    MPU9250KSensor::motionSetup() {

    uint8_t id = imu.getMPU9250ID(addr);
    if (id != 0x71 && id != 0x73)  // mpu9250 or mpu9255
    {
        Serial.printf("MPU9250KSensor::motionSetup() -- wrong IMU 0x%02x",id);
        return ;
    }

    imu.resetMPU9250(addr);

    aRes = imu.getAres(Ascale);
    gRes = imu.getGres(Gscale);
    mRes = imu.getMres(Mscale);

    imu.initMPU9250(addr, Ascale, Gscale, sampleRate);

    //uint8_t magCalRaw[3];
    imu.initAK8963Slave(addr, Mscale, Mmode, magCalibration);

    { // calibration
        updateAG();
        vector3<float> a = getAccel();
        if (a.z < 0 && 10.0 * (a.x * a.x + a.y * a.y) < a.z * a.z)
        {
            LEDManager::on(CALIBRATING_LED);
            Serial.println("Calling Calibration... Flip front to confirm start calibration.");
            delay(5000);
            LEDManager::off(CALIBRATING_LED);

            //imu.getAcceleration(&ax, &ay, &az);
            updateAG();
            a = getAccel();

            if (a.z > 0 && 10.0 * (a.x * a.x + a.y * a.y) < a.z * a.z)
                startCalibration(0);
        }
    }

    DeviceConfig *config = getConfigPtr();
    gyroBias = config->calibration[sensorId].gyroBias;
    magBias = config->calibration[sensorId].magBias;

    Serial.printf("{%x} [NOTICE] Gyro calibration saved values: %f %f %f\n", addr,gyroBias.x, gyroBias.y, gyroBias.z);
    Serial.printf("{%x} [NOTICE] Magnetometer calibration saved values: %f %f %f\n", addr, magBias.x, magBias.y, magBias.z);

    working = true;
    configured = true;

    /*
    DeviceConfig * const config = getConfigPtr();
    calibration = &config->calibration[sensorId];
    // initialize device
    imu.initialize(addr);
    if(!imu.testConnection()) {
        Serial.print("[ERR] MPU9250: Can't communicate with MPU, response 0x");
        Serial.println(imu.getDeviceID(), HEX);
        return;
    }

    Serial.print("[OK] MPU9250: Connected to MPU, ID 0x");
        Serial.println(imu.getDeviceID(), HEX);


    // turn on while flip back to calibrate. then, flip again after 5 seconds.    
    // TODO: Move calibration invoke after calibrate button on slimeVR server available 

    
    //Serial.printf("Probing %s sensor in 3 ...", (sensorId == 0)?"first":"second");
    //delay(1000);
    //Serial.println("2 ...");
    //delay(1000);
    //Serial.println("1 ...");
    //delay(1000);
    

    int16_t ax, ay, az;
    imu.getAcceleration(&ax, &ay, &az);

    if(az<0 && 10.0*(ax*ax+ay*ay)<az*az) {
        LEDManager::on(CALIBRATING_LED);
        Serial.println("Calling Calibration... Flip front to confirm start calibration.");
        delay(5000);
        LEDManager::off(CALIBRATING_LED);
        imu.getAcceleration(&ax, &ay, &az);
        if(az>0 && 10.0*(ax*ax+ay*ay)<az*az) 
            startCalibration(0);
    }
#if not (defined(_MAHONY_H_) || defined(_MADGWICK_H_))
    devStatus = imu.dmpInitialize();
    if(devStatus == 0){
        for(int i = 0; i < 5; ++i) {
            delay(50);
            digitalWrite(LOADING_LED, LOW);
            delay(50);
            digitalWrite(LOADING_LED, HIGH);
        }

        // turn on the DMP, now that it's ready
        Serial.println(F("[NOTICE] Enabling DMP..."));
        imu.setDMPEnabled(true);

        // TODO: Add interrupt support
        // mpuIntStatus = imu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("[NOTICE] DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = imu.dmpGetFIFOPacketSize();
        working = true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("[ERR] DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
#else
    working = true;
    configured = true;
#endif
    */
}


void MPU9250KSensor::motionLoop() {

    updateAG();
    vector3<float> a = getAccel();
    a = a.norm();
    // Serial.printf("accel {%x} : %-3.2f , %-3.2f , %-3.2f\n", (int)addr, a.x, a.y, a.z);

    vector3<float> g = getGyro();
    g = g * (3.1415f / 180);
    //Serial.printf("gyro {%x} : %-3.2f , %-3.2f , %-3.2f\n", (int)addr, g.x, g.y, g.z);

    updateM();
    vector3<float> m = getMag();
    m = m.norm();
    //Serial.printf("mag {%x} : %-3.2f , %-3.2f , %-3.2f\n", (int)addr, m.x, m.y, m.z);

    //  https://en.wikipedia.org/wiki/Triple_product#Vector_triple_product
    //  a⨯(b⨯c) = (a⋅c)b - (a⋅b)c

    // hn = a⨯(a⨯m) = (a⋅m)a - (a⋅a)m


    //vector3<float> hn = (a*(a*m)) - (m*(a*a));

    //hn = hn.norm();

    //Serial.printf("HN {%x} : %-3.2f , %-3.2f , %-3.2f\n", (int)addr, hn.x, hn.y, hn.z);

    unsigned long now = micros();
    unsigned long deltat = now - last; // seconds since last update
    last = now;
    //getMPUScaled();
    //mahonyQuaternionUpdate(q, a.x, a.y, a.z, g.x, g.y, g.z,m.x, m.y, -m.z, deltat * 1.0e-6);

    // this also flips axii to the same order axii are in BNO055 config P0
    MahonyAHRSupdate(q,
                     a.x, a.y, a.z,
                     g.x, g.y, g.z,
                     m.y, m.x, -m.z,  // yes X and Y is swapped and Z is inverted. AK8963 is upsidedown in MPU9250. See yourself in datasheet!
                     deltat * 1.0e-6);

    // madgwickQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
    //quaternion.set(q[0], q[1], q[2], q[3]);
    //quaternion.set(-q[2], q[1], q[3], q[0]);
    //quaternion.set(-q[1], -q[2], -q[0], q[3]);
    
    /*
    quaternion::Quaternion<float> qq(q[0], q[1], q[2], q[3]);
    qq = quaternion::normalize(qq);
    std::array<float, 3> eul = quaternion::to_euler<float>(qq, 1e-6f);
    const float DEG = 180.00f / 3.1415f;
    Serial.printf("{%x} euler angles : %-03d %-03d %-03d\n", addr, int(eul[0] * DEG), int(eul[1] * DEG), int(eul[2] * DEG));
    */

    //quaternion.set(qq.a(), qq.b(), qq.c(), qq.d());

    // some read on quaternions : https://www.mdpi.com/2226-4310/5/3/72/pdf
    //  and https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation

    quaternion.set(q[1], q[2], q[3] , q[0]);

    Quat rot = {Quat(Vector3(0, 0, 1), DEG_270)};
    quaternion = rot * quaternion;

    //Serial.printf("{%x} quat : w %-1.2f  x %-1.2f  y %-1.2f  z %-1.2f\n", addr, quaternion.w, quaternion.x, quaternion.y, quaternion.z);

    //quaternion *= sensorOffset;
    quaternion = sensorOffset * quaternion;

    /*{
        quaternion::Quaternion<float> qq(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        qq = quaternion::normalize(qq);
        std::array<float, 3> eul = quaternion::to_euler<float>(qq, 1e-6f);
        const float DEG = 180.00f / 3.1415f;
        Serial.printf("{%x} euler: %-03d %-03d %-03d\n", addr, int(eul[0] * DEG), int(eul[1] * DEG), int(eul[2] * DEG));
    }*/

    if (!lastQuatSent.equalsWithEpsilon(quaternion))
    {
        newData = true;
        lastQuatSent = quaternion;
    }

    /*
#if not (defined(_MAHONY_H_) || defined(_MADGWICK_H_))
    // Update quaternion
    if(!dmpReady)
        return;
    Quaternion rawQuat{};
    if(!imu.GetCurrentFIFOPacket(fifoBuffer,imu.dmpGetFIFOPacketSize())) return;
    imu.dmpGetQuaternion(&rawQuat, fifoBuffer);
    logger.printf("Quat: %f %f %f %f\n", rawQuat.y, rawQuat.x, rawQuat.z, rawQuat.w );
    Quat quat(-rawQuat.y, rawQuat.x, rawQuat.z, rawQuat.w);
    if(!skipCalcMag){
        getMPUScaled();
        if(Mxyz[0]==0.0f && Mxyz[1]==0.0f && Mxyz[2]==0.0f) return;
        VectorFloat grav;
        imu.dmpGetGravity(&grav,&rawQuat);
        float Grav[3]={grav.x,grav.y,grav.z};
        skipCalcMag=SKIP_CALC_MAG_INTERVAL;
        if(correction.length_squared()==0.0f) {
            correction=getCorrection(Grav,Mxyz,quat);
            if(sensorId) skipCalcMag=SKIP_CALC_MAG_INTERVAL/2;
        }
        else {
            Quat newCorr = getCorrection(Grav,Mxyz,quat);
            if(!__isnanf(newCorr.w)) correction = correction.slerp(newCorr,MAG_CORR_RATIO);
        }
    }else skipCalcMag--;
    quaternion=correction*quat;
    logger.printf("Quat11: %f %f %f %f\n", quaternion.x, quaternion.y, quaternion.z, quaternion.w );
#else
    unsigned long now = micros();
    unsigned long deltat = now - last; //seconds since last update
    last = now;
    getMPUScaled();
    mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
    // madgwickQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
    quaternion.set(-q[2], q[1], q[3], q[0]);

#endif
    quaternion *= sensorOffset;
    if(!lastQuatSent.equalsWithEpsilon(quaternion)) {
        newData = true;
        lastQuatSent = quaternion;
    }
    */
}

/*
void MPU9250KSensor::getMPUScaled()
{
    
    float temp[3];
    int i;
    int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // Gxyz[0] = ((float)gx - calibration->G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
    // Gxyz[1] = ((float)gy - calibration->G_off[1]) * gscale;
    // Gxyz[2] = ((float)gz - calibration->G_off[2]) * gscale;
    Gxyz[0] = (float)gx * gscale; //250 LSB(d/s) default to radians/s
    Gxyz[1] = (float)gy * gscale;
    Gxyz[2] = (float)gz * gscale;

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;

    //apply offsets (bias) and scale factors from Magneto
    #if useFullCalibrationMatrix == true
        for (i = 0; i < 3; i++)
            temp[i] = (Axyz[i] - calibration->A_B[i]);
        Axyz[0] = calibration->A_Ainv[0][0] * temp[0] + calibration->A_Ainv[0][1] * temp[1] + calibration->A_Ainv[0][2] * temp[2];
        Axyz[1] = calibration->A_Ainv[1][0] * temp[0] + calibration->A_Ainv[1][1] * temp[1] + calibration->A_Ainv[1][2] * temp[2];
        Axyz[2] = calibration->A_Ainv[2][0] * temp[0] + calibration->A_Ainv[2][1] * temp[1] + calibration->A_Ainv[2][2] * temp[2];
    #else
        for (i = 0; i < 3; i++)
            Axyz[i] = (Axyz[i] - calibration->A_B[i]);
    #endif

    // Orientations of axes are set in accordance with the datasheet
    // See Section 9.1 Orientation of Axes
    // https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
    Mxyz[0] = (float)my;
    Mxyz[1] = (float)mx;
    Mxyz[2] = -(float)mz;
    //apply offsets and scale factors from Magneto
    #if useFullCalibrationMatrix == true
        for (i = 0; i < 3; i++)
            temp[i] = (Mxyz[i] - calibration->M_B[i]);
        Mxyz[0] = calibration->M_Ainv[0][0] * temp[0] + calibration->M_Ainv[0][1] * temp[1] + calibration->M_Ainv[0][2] * temp[2];
        Mxyz[1] = calibration->M_Ainv[1][0] * temp[0] + calibration->M_Ainv[1][1] * temp[1] + calibration->M_Ainv[1][2] * temp[2];
        Mxyz[2] = calibration->M_Ainv[2][0] * temp[0] + calibration->M_Ainv[2][1] * temp[1] + calibration->M_Ainv[2][2] * temp[2];
    #else
        for (i = 0; i < 3; i++)
            Mxyz[i] = (Mxyz[i] - calibration->M_B[i]);
    #endif    
}
*/

constexpr int calibrationSamples = 300;
//float dataAcc[calibrationSamples * 3];
//float dataMag[calibrationSamples * 3];

void MPU9250KSensor::startCalibration(int calibrationType) {

    LEDManager::on(CALIBRATING_LED);
    Serial.println("[NOTICE] Gathering raw data for device calibration...");
    DeviceConfig *config = getConfigPtr();

    // Wait for sensor to calm down before calibration
    Serial.println("[NOTICE] Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    vector3<float> g_sum;
    for (int i = 0; i < calibrationSamples; i++)
    {
        updateAG();
        g_sum = g_sum + getGyroRaw();
    }
    g_sum = g_sum / calibrationSamples;
    Serial.printf("[NOTICE] Gyro calibration results: %f %f %f\n", g_sum.x, g_sum.y, g_sum.z);
    //Network::sendRawCalibrationData(g_sum.x, g_sum.y, g_sum.z, CALIBRATION_TYPE_EXTERNAL_GYRO, 0);
    config->calibration[sensorId].gyroBias[0] = g_sum.x;
    config->calibration[sensorId].gyroBias[1] = g_sum.y;
    config->calibration[sensorId].gyroBias[2] = g_sum.z;

    /*
    LEDManager::on(CALIBRATING_LED);
    Serial.println("[NOTICE] Gathering raw data for device calibration...");
    constexpr int calibrationSamples = 300;
    DeviceConfig *config = getConfigPtr();
    // Reset values
    Gxyz[0] = 0;
    Gxyz[1] = 0;
    Gxyz[2] = 0;

    // Wait for sensor to calm down before calibration
    Serial.println("[NOTICE] Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    for (int i = 0; i < calibrationSamples; i++)
    {
        int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        Gxyz[0] += float(gx);
        Gxyz[1] += float(gy);
        Gxyz[2] += float(gz);
    }
    Gxyz[0] /= calibrationSamples;
    Gxyz[1] /= calibrationSamples;
    Gxyz[2] /= calibrationSamples;
    Serial.printf("[NOTICE] Gyro calibration results: %f %f %f\n", Gxyz[0], Gxyz[1], Gxyz[2]);
    Network::sendRawCalibrationData(Gxyz, CALIBRATION_TYPE_EXTERNAL_GYRO, 0);
    config->calibration[sensorId].G_off[0] = Gxyz[0];
    config->calibration[sensorId].G_off[1] = Gxyz[1];
    config->calibration[sensorId].G_off[2] = Gxyz[2];
    */

    Serial.println("[NOTICE] Gently rotate the device while it's gathering accelerometer and magnetometer data");
    LEDManager::pattern(CALIBRATING_LED, 15, 300, 3000 / 310);
    {
        float xmin=0,ymin=0,zmin=0;
        float xmax=0,ymax=0,zmax=0;
        bool flag = true;
        for (int i = 0; i < calibrationSamples; i++)
        {
            LEDManager::on(CALIBRATING_LED);

            /*updateAG();
            vector3<float> a = getAccel();
            dataAcc[ 3*i + 0] = a.x;
            dataAcc[3 * i + 1] = a.y;
            dataAcc[3 * i + 2] = a.z;
            */

            updateM();
            vector3<float> m = getMagRaw();

            if(fabsf(m.x) > 5000 || fabsf(m.y) > 5000 || fabsf(m.z) > 5000 ) continue;

            if( flag )
            {
                flag= false;
                xmin = xmax = m.x;
                ymin = ymax = m.y;
                zmin = zmax = m.z;
            }

            if( m.x < xmin ) xmin = m.x;
            if( m.y < ymin )  ymin = m.y;
            if( m.z < zmin )  zmin = m.z;
            if( m.x > xmax ) xmax = m.x;
            if( m.y > ymax )  ymax = m.y;
            if( m.z > zmax )  zmax = m.z;

            //dataMag[3 * i + 0] = m.x;
            //dataMag[3 * i + 1] = m.y;
            //dataMag[3 * i + 2] = m.z;

            delay(50);
            LEDManager::off(CALIBRATING_LED);
            delay(150);
        }
        Serial.println("[NOTICE] Now Calculate Calibration data");

        vector3<float> magBias((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2 );
        /*
        float A_BAinv[4][3];
        float M_BAinv[4][3];
        CalculateCalibration(dataAcc, calibrationSamples, A_BAinv);
        CalculateCalibration(dataMag, calibrationSamples, M_BAinv);
        */

        Serial.printf("{%x} magnetometer bias values : %f %f %f\n", addr, magBias.x, magBias.y, magBias.z);

        Serial.println("[NOTICE] Finished Calculate Calibration data");
        Serial.println("[NOTICE] Now Saving EEPROM");
        /*
        for (int i = 0; i < 3; i++)
        {
            config->calibration[sensorId].A_B[i] = A_BAinv[0][i];
            config->calibration[sensorId].A_Ainv[0][i] = A_BAinv[1][i];
            config->calibration[sensorId].A_Ainv[1][i] = A_BAinv[2][i];
            config->calibration[sensorId].A_Ainv[2][i] = A_BAinv[3][i];

            config->calibration[sensorId].M_B[i] = M_BAinv[0][i];
            config->calibration[sensorId].M_Ainv[0][i] = M_BAinv[1][i];
            config->calibration[sensorId].M_Ainv[1][i] = M_BAinv[2][i];
            config->calibration[sensorId].M_Ainv[2][i] = M_BAinv[3][i];
        }*/
        config->calibration[sensorId].magBias[0] = magBias.x;
        config->calibration[sensorId].magBias[1] = magBias.y;
        config->calibration[sensorId].magBias[2] = magBias.z;
        setConfig(*config);
        LEDManager::off(CALIBRATING_LED);
        //Network::sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0);
        Serial.println("[NOTICE] Finished Saving EEPROM");
        Serial.println("[NOTICE] Calibration data gathered and sent");
    }

    /*
    // Blink calibrating led before user should rotate the sensor
    Serial.println("[NOTICE] Gently rotate the device while it's gathering accelerometer and magnetometer data");
    LEDManager::pattern(CALIBRATING_LED, 15, 300, 3000/310);
    float *calibrationDataAcc = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    float *calibrationDataMag = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    for (int i = 0; i < calibrationSamples; i++)
    {
        LEDManager::on(CALIBRATING_LED);
        int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        calibrationDataAcc[i * 3 + 0] = ax;
        calibrationDataAcc[i * 3 + 1] = ay;
        calibrationDataAcc[i * 3 + 2] = az;
        calibrationDataMag[i * 3 + 0] = my;
        calibrationDataMag[i * 3 + 1] = mx;
        calibrationDataMag[i * 3 + 2] = -mz;
        Network::sendRawCalibrationData(calibrationDataAcc, CALIBRATION_TYPE_EXTERNAL_ACCEL, 0);
        Network::sendRawCalibrationData(calibrationDataMag, CALIBRATION_TYPE_EXTERNAL_MAG, 0);
        delay(50);
        LEDManager::off(CALIBRATING_LED);
        delay(200);
    }
    Serial.println("[NOTICE] Now Calculate Calibration data");

    float A_BAinv[4][3];
    float M_BAinv[4][3];
    CalculateCalibration(calibrationDataAcc, calibrationSamples, A_BAinv);
    CalculateCalibration(calibrationDataMag, calibrationSamples, M_BAinv);
    free(calibrationDataAcc);
    free(calibrationDataMag);
    Serial.println("[NOTICE] Finished Calculate Calibration data");
    Serial.println("[NOTICE] Now Saving EEPROM");
    for (int i = 0; i < 3; i++)
    {
        config->calibration[sensorId].A_B[i] = A_BAinv[0][i];
        config->calibration[sensorId].A_Ainv[0][i] = A_BAinv[1][i];
        config->calibration[sensorId].A_Ainv[1][i] = A_BAinv[2][i];
        config->calibration[sensorId].A_Ainv[2][i] = A_BAinv[3][i];

        config->calibration[sensorId].M_B[i] = M_BAinv[0][i];
        config->calibration[sensorId].M_Ainv[0][i] = M_BAinv[1][i];
        config->calibration[sensorId].M_Ainv[1][i] = M_BAinv[2][i];
        config->calibration[sensorId].M_Ainv[2][i] = M_BAinv[3][i];
    }
    setConfig(*config);
    LEDManager::off(CALIBRATING_LED);
    Network::sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0);
    Serial.println("[NOTICE] Finished Saving EEPROM");
    Serial.println("[NOTICE] Calibration data gathered and sent");
    */
}