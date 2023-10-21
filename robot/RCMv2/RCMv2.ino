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
JTwoDTransform robotToWheelScalar = { 1 / wheelCir, 1 / wheelCir, ((float)1.35) / ((float)TWO_PI) / wheelCir }; // adjust until it converts robot speed in your chosen units to wheel units (increasing numbers makes robot faster)
// TODO: calibrate robot to wheel scalar
JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, dacUnitsPerVolt);
JMotorDriverEsp32L293 flMotorDriver = JMotorDriverEsp32L293(portA, true, false, false, 8000, 12);
JMotorDriverEsp32L293 frMotorDriver = JMotorDriverEsp32L293(portB, true, false, false, 8000, 12);
JMotorDriverEsp32L293 blMotorDriver = JMotorDriverEsp32L293(portC, true, false, false, 8000, 12);
JMotorDriverEsp32L293 brMotorDriver = JMotorDriverEsp32L293(portD, true, false, false, 8000, 12);
JEncoderSingleAttachInterrupt flEncoder = JEncoderSingleAttachInterrupt(inport1, 1.0 / encoderTicksPerRev, false, 20000, 200, FALLING);
JEncoderSingleAttachInterrupt frEncoder = JEncoderSingleAttachInterrupt(inport2, 1.0 / encoderTicksPerRev, false, 20000, 200, FALLING);
JEncoderSingleAttachInterrupt blEncoder = JEncoderSingleAttachInterrupt(inport3, 1.0 / encoderTicksPerRev, false, 20000, 200, FALLING);
JEncoderSingleAttachInterrupt brEncoder = JEncoderSingleAttachInterrupt(port3Pin, 1.0 / encoderTicksPerRev, false, 20000, 200, FALLING);
JMotorCompBasic motorCompensator = JMotorCompBasic(voltageComp, 1.7, 0.5); // volts per rps, min rps
JControlLoopBasic flCtrlLoop = JControlLoopBasic(10, 400);
JControlLoopBasic frCtrlLoop = JControlLoopBasic(10, 400);
JControlLoopBasic blCtrlLoop = JControlLoopBasic(10, 400);
JControlLoopBasic brCtrlLoop = JControlLoopBasic(10, 400);
JMotorControllerClosed flMotor = JMotorControllerClosed(flMotorDriver, motorCompensator, flEncoder, flCtrlLoop, INFINITY, INFINITY, 0.01);
JMotorControllerClosed frMotor = JMotorControllerClosed(frMotorDriver, motorCompensator, frEncoder, frCtrlLoop, INFINITY, INFINITY, 0.01);
JMotorControllerClosed blMotor = JMotorControllerClosed(blMotorDriver, motorCompensator, blEncoder, blCtrlLoop, INFINITY, INFINITY, 0.01);
JMotorControllerClosed brMotor = JMotorControllerClosed(brMotorDriver, motorCompensator, brEncoder, brCtrlLoop, INFINITY, INFINITY, 0.01);
JDrivetrainMecanum drivetrain = JDrivetrainMecanum(frMotor, flMotor, blMotor, brMotor, robotToWheelScalar); // drivetrain converts from robot speed to motor speeds
JDrivetrainFieldOriented drivetrainFO = JDrivetrainFieldOriented(drivetrain);
JDrivetrainControllerBasic drivetrainController = JDrivetrainControllerBasic(drivetrainFO, { INFINITY, INFINITY, INFINITY }, { 5, 5, 5 }, { 0.15, 0.15, 0.25 }, false);

byte mode = 0;
float heading = 0;
float headingOffset = 0;

JTwoDTransform driverInput = JTwoDTransform();

float brightDiff = 0;

boolean dance = false;

float driveHeading = 0;
boolean turnL = false;
boolean turnR = false;

int danceState = 0;

// bno08x code modified from example from Adafruit's library
struct euler_t {
    float yaw;
    float pitch;
    float roll;
} ypr;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

void Enabled()
{
    if (mode != 4) {
        danceState = 0;
    }
    // code to run while enabled, put your main code here
    if (mode == 1) { // drive
        drivetrainController.moveVel(driverInput);
    } else if (mode == 2) { // go towards light
        float brightnessL = analogRead(port2Pin) / 4096.0;
        float brightnessR = analogRead(port1Pin) / 4096.0;
        brightDiff = brightnessR - brightnessL;

        JTwoDTransform drive = { 0, 0, constrain(brightDiff * 9, -.7, .7) };
        drive.x = -0.25 * cos(heading - headingOffset);
        drive.y = -0.25 * sin(heading - headingOffset); // back to non field oriented
        drivetrainController.moveVel(drive);
    } else if (mode == 3) { // go towards heading
        if (turnL)
            driveHeading += 0.02;
        if (turnR)
            driveHeading -= 0.02;

        JTwoDTransform drive = { 0, 0, 0 };
        drive.x = 0.25 * cos(driveHeading);
        drive.y = 0.25 * sin(driveHeading);
        drivetrainController.moveVel(drive);
    } else if (mode == 4) {
        switch (danceState) {
        case 0:
            if (dance) {
                headingOffset = heading;
                drivetrainController.resetDist();
                drivetrainController.moveDistInc({ 0.4, 0, 0 });
                danceState = 1;
            }
            break;
        case 1:
            if (drivetrainController.isDrivetrainAtTarget()) {
                drivetrainController.moveDistInc({ 0, -.4, 0 });
                danceState = 2;
            }
            break;
        case 2:
            if (drivetrainController.isDrivetrainAtTarget()) {
                drivetrainController.moveDistInc({ 0, 0, PI / 2 });
                danceState = 3;
            }
            break;
        case 3:
            if (drivetrainController.isDrivetrainAtTarget()) {
                drivetrainController.moveDistInc({ -.4, 0, 0 });
                danceState = 4;
            }
            break;
        case 4:
            if (drivetrainController.isDrivetrainAtTarget()) {
                drivetrainController.moveDistInc({ 0, 0, -PI / 2 });
                danceState = 5;
            }
            break;
        case 5:
            if (drivetrainController.isDrivetrainAtTarget()) {
                drivetrainController.moveDistInc({ 0, 0.35, 0 });
                danceState = 6;
            }
            break;
        case 6:
            if (drivetrainController.isDrivetrainAtTarget()) {
                danceState = 0;
            }
            break;
            brightDiff = danceState;
        }
    } else {
        drivetrainController.moveVel({ 0, 0, 0 });
    }
}

void Enable()
{
    // turn on outputs
    drivetrainController.resetDist();
    drivetrainController.moveDist({ 0, 0, 0 });
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
    if (!enabled) {
        float brightnessL = analogRead(port2Pin) / 4096.0;
        float brightnessR = analogRead(port1Pin) / 4096.0;
        brightDiff = brightnessL - brightnessR;
    }

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
    mode = EWD::recvBy();
    if (mode == 1) {
        driverInput.x = EWD::recvFl();
        driverInput.y = EWD::recvFl();
        driverInput.theta = EWD::recvFl();
    }
    if (mode == 3) {
        turnL = EWD::recvBl();
        turnR = EWD::recvBl();
    }
    if (mode == 4) {
        dance = EWD::recvBl();
    }
}
void WifiDataToSend()
{
    EWD::sendFl(voltageComp.getSupplyVoltage());
    // add data to send here: (EWD::sendBl(), EWD::sendBy(), EWD::sendIn(), EWD::sendFl())(boolean, byte, int, float)
    EWD::sendFl(drivetrainController.getDist().x);
    EWD::sendFl(drivetrainController.getDist().y);
    EWD::sendFl(drivetrainController.getDist().theta);
    EWD::sendFl(drivetrainFO.getHeading());
    EWD::sendFl(brightDiff);
}

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
