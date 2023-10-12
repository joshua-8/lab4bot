//   This program controls an x drive robot
//   https://github.com/rcmgames/RCMv2
//   for information about the electronics, see the link at the top of this page: https://github.com/RCMgames
#include "rcm.h" //defines pins
#include <Adafruit_BNO08x.h> //https://github.com/adafruit/Adafruit_BNO08x v1.2.3
#include <ESP32_easy_wifi_data.h> //https://github.com/joshua-8/ESP32_easy_wifi_data >=v1.0.0
#include <JMotor.h> //https://github.com/joshua-8/JMotor

const int dacUnitsPerVolt = 413; // increasing this number decreases the calculated voltage
const int encoderTicksPerRev = 625;
float wheelCir = (0.1 / 2 * PI);
JTwoDTransform robotToWheelScalar = { 1 / wheelCir, 1 / wheelCir, 1.35 / ((float)TWO_PI) / wheelCir }; // adjust until it converts robot speed in your chosen units to wheel units (increasing numbers makes robot faster)
// TODO: calibrate robot to wheel scalar
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, dacUnitsPerVolt);
JMotorDriverEsp32L293 flMotorDriver = JMotorDriverEsp32L293(portA, true, false, false, 8000, 12);
JMotorDriverEsp32L293 frMotorDriver = JMotorDriverEsp32L293(portB, true, false, false, 8000, 12);
JMotorDriverEsp32L293 blMotorDriver = JMotorDriverEsp32L293(portC, true, false, false, 8000, 12);
JMotorDriverEsp32L293 brMotorDriver = JMotorDriverEsp32L293(portD, true, false, false, 8000, 12);
JEncoderSingleAttachInterrupt flEncoder = JEncoderSingleAttachInterrupt(inport1, 1.0 / encoderTicksPerRev, false, 20000, 200, FALLING);
JEncoderSingleAttachInterrupt frEncoder = JEncoderSingleAttachInterrupt(inport2, 1.0 / encoderTicksPerRev, false, 20000, 200, FALLING);
JEncoderSingleAttachInterrupt blEncoder = JEncoderSingleAttachInterrupt(inport3, 1.0 / encoderTicksPerRev, false, 20000, 200, FALLING);
JEncoderSingleAttachInterrupt brEncoder = JEncoderSingleAttachInterrupt(port1Pin, 1.0 / encoderTicksPerRev, false, 20000, 200, FALLING);
JMotorCompBasic motorCompensator = JMotorCompBasic(voltageComp, 1.7, 0.5); // volts per rps, min rps
JControlLoopBasic flCtrlLoop = JControlLoopBasic(10, 400);
JControlLoopBasic frCtrlLoop = JControlLoopBasic(10, 400);
JControlLoopBasic blCtrlLoop = JControlLoopBasic(10, 400);
JControlLoopBasic brCtrlLoop = JControlLoopBasic(10, 400);
JMotorControllerClosed flMotor = JMotorControllerClosed(flMotorDriver, motorCompensator, flEncoder, flCtrlLoop, INFINITY, INFINITY, 0.1);
JMotorControllerClosed frMotor = JMotorControllerClosed(frMotorDriver, motorCompensator, frEncoder, frCtrlLoop, INFINITY, INFINITY, 0.1);
JMotorControllerClosed blMotor = JMotorControllerClosed(blMotorDriver, motorCompensator, blEncoder, blCtrlLoop, INFINITY, INFINITY, 0.1);
JMotorControllerClosed brMotor = JMotorControllerClosed(brMotorDriver, motorCompensator, brEncoder, brCtrlLoop, INFINITY, INFINITY, 0.1);
JDrivetrainMecanum drivetrain = JDrivetrainMecanum(frMotor, flMotor, blMotor, brMotor, robotToWheelScalar); // drivetrain converts from robot speed to motor speeds
JDrivetrainFieldOriented drivetrainFO = JDrivetrainFieldOriented(drivetrain);
JDrivetrainControllerBasic drivetrainController = JDrivetrainControllerBasic(drivetrainFO, { INFINITY, INFINITY, INFINITY }, { 5, 5, 5 }, { 0.05, 0.05, 0.5 }, false);

JTwoDTransform driverInput = JTwoDTransform();
float heading = 0;
float headingOffset = 0;

struct euler_t {
    float yaw;
    float pitch;
    float roll;
} ypr;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr)
{
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);
    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
}
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr)
{
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr);
}
void setReports()
{
    if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {
        Serial.println("Could not enable stabilized remote vector, STOPPING PROGRAM");
        while (1) {
            delay(10);
        }
    }
}

void Enabled()
{
    // code to run while enabled, put your main code here
    drivetrainController.moveVel(driverInput);
}

void Enable()
{
    // turn on outputs
    drivetrainFO.resetDist();
    headingOffset = heading;
    drivetrainController.enable();
    driverInput = { 0, 0, 0 };
}

void Disable()
{
    // shut off all outputs
    drivetrainController.disable();
    driverInput = { 0, 0, 0 };
}

jENCODER_MAKE_ISR_MACRO(frEncoder);
jENCODER_MAKE_ISR_MACRO(flEncoder);
jENCODER_MAKE_ISR_MACRO(brEncoder);
jENCODER_MAKE_ISR_MACRO(blEncoder);
void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here
    frEncoder.setUpInterrupts(frEncoder_jENCODER_ISR);
    flEncoder.setUpInterrupts(flEncoder_jENCODER_ISR);
    brEncoder.setUpInterrupts(brEncoder_jENCODER_ISR);
    blEncoder.setUpInterrupts(blEncoder_jENCODER_ISR);

    Wire.begin(port4Pin, port5Pin); // sda,scl

    if (!bno08x.begin_I2C()) {
        Serial.println("Failed to find BNO08x chip, STOPPING PROGRAM");
        while (1) {
            delay(10);
        }
    }
    if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {
        Serial.println("Could not enable stabilized remote vector, STOPPING PROGRAM");
        while (1) {
            delay(10);
        }
    }
}

void Always()
{
    // always runs if void loop is running, top level JMotor run() functions should be put here
    if (bno08x.getSensorEvent(&sensorValue)) {
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr);
    }
    heading = ypr.yaw;
    drivetrainFO.giveHeading(heading - headingOffset);
    drivetrainController.run();
    delay(1);
}

void configWifi()
{
    EWD::mode = EWD::Mode::connectToNetwork;
    EWD::routerName = "router";
    EWD::routerPassword = "password";
    EWD::routerPort = 25210;
    EWD::signalLossTimeout = 350;
}

void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
    driverInput.x = EWD::recvFl();
    driverInput.y = EWD::recvFl();
    driverInput.theta = EWD::recvFl();
}
void WifiDataToSend()
{
    EWD::sendFl(voltageComp.getSupplyVoltage());
    // add data to send here: (EWD::sendBl(), EWD::sendBy(), EWD::sendIn(), EWD::sendFl())(boolean, byte, int, float)
    EWD::sendFl(drivetrainController.getVel().x);
    EWD::sendFl(drivetrainController.getVel().y);
    EWD::sendFl(drivetrainController.getVel().theta);
    EWD::sendFl(drivetrainFO.getHeading());
}

////////////////////////////// you don't need to edit below this line ////////////////////

void setup()
{
    Serial.begin(115200);
    pinMode(ONBOARD_LED, OUTPUT);
    PowerOn();
    Disable();
    configWifi();
    EWD::setupWifi(WifiDataToParse, WifiDataToSend);
}

void loop()
{
    EWD::runWifiCommunication();
    if (!EWD::wifiConnected || EWD::timedOut()) {
        enabled = false;
    }
    Always();
    if (enabled && !wasEnabled) {
        Enable();
    }
    if (!enabled && wasEnabled) {
        Disable();
    }
    if (enabled) {
        Enabled();
        digitalWrite(ONBOARD_LED, millis() % 500 < 250); // flash, enabled
    } else {
        if (!EWD::wifiConnected)
            digitalWrite(ONBOARD_LED, millis() % 1000 <= 100); // short flash, wifi connection fail
        else if (EWD::timedOut())
            digitalWrite(ONBOARD_LED, millis() % 1000 >= 100); // long flash, no driver station connected
        else
            digitalWrite(ONBOARD_LED, HIGH); // on, disabled
    }
    wasEnabled = enabled;
}
