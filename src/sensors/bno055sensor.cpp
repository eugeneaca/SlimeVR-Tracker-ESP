/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

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
#include "bno055sensor.h"
#include "network/network.h"
#include "globals.h"
#include "ledmgr.h"

#include "quaternion.h"
#include "udplogger.h"
#define Serial logger
#define println print

void BNO055Sensor::motionSetup() {
    imu = Adafruit_BNO055(sensorId, addr);
    delay(3000);
    if (!imu.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
    {
        Serial.print("[ERR] IMU BNO055: Can't connect to ");
        Serial.println(getIMUNameByType(sensorType));
        LEDManager::signalAssert();
        return;
    }

    delay(1000);
    imu.setExtCrystalUse(false);
    imu.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P0);
    imu.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P0);
    Serial.print("[NOTICE] Connected to");
    Serial.println(getIMUNameByType(sensorType));
    working = true;
    configured = true;
}

void BNO055Sensor::motionLoop() {
    // TODO Optimize a bit with setting rawQuat directly
    Quat quat = imu.getQuat();
    quat.normalize();
    quaternion.set(quat.x, quat.y, quat.z, quat.w);
    

    //Serial.printf("\t\t\t\t\t\t{%x} quat : w %-1.2f  x %-1.2f  y %-1.2f  z %-1.2f\n", addr, quaternion.w, quaternion.x, quaternion.y, quaternion.z );

    quaternion *= sensorOffset;  // ??
    // Quat rot = {Quat(Vector3(1, 0, 0), DEG_90 )};
    // quaternion *= rot;

    /*{
        quaternion::Quaternion<float> qq(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
        qq = quaternion::normalize(qq);
        std::array<float, 3> eul = quaternion::to_euler<float>(qq, 1e-6f);
        const float DEG = 180.00f / 3.1415f;
        Serial.printf("\t\t\t\t\t{%x} euler : %-03d %-03d %-03d\n", addr, int(eul[0] * DEG), int(eul[1] * DEG), int(eul[2] * DEG));
    }*/

    if(!OPTIMIZE_UPDATES || !lastQuatSent.equalsWithEpsilon(quaternion)) {
        newData = true;
        lastQuatSent = quaternion;
    }
}

void BNO055Sensor::startCalibration(int calibrationType) {

}