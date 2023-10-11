//   This program controls Benjamin's RCM power up mini robot "Swervy"
//   https://github.com/rcmgames/RCMv2
//   for information about the electronics, see the link at the top of this page: https://github.com/RCMgames
#include "rcm.h" //defines pins
#include <ESP32_easy_wifi_data.h> //https://github.com/joshua-8/ESP32_easy_wifi_data >=v1.0.0
#include <JMotor.h> //https://github.com/joshua-8/JMotor

const int dacUnitsPerVolt = 380; // increasing this number decreases the calculated voltage
const float wheelCir = PI * 0.1; // wheel diam
const int encoderTicksPerRev = 2340;

JTwoDTransform robotToWheelScalar = { 1, 1, 1 }; // adjust until it converts robot speed in your chosen units to wheel units (increasing numbers makes robot faster)

JVoltageCompMeasure<10> voltageComp = JVoltageCompMeasure<10>(batMonitorPin, dacUnitsPerVolt);
JMotorDriverEsp32L293 flMotorDriver = JMotorDriverEsp32L293(portA, true, false, false, 8000, 12);
JMotorDriverEsp32L293 frMotorDriver = JMotorDriverEsp32L293(portB, true, false, false, 8000, 12);
JMotorDriverEsp32L293 blMotorDriver = JMotorDriverEsp32L293(portC, true, false, false, 8000, 12);
JMotorDriverEsp32L293 brMotorDriver = JMotorDriverEsp32L293(portD, true, false, false, 8000, 12);
JEncoderSingleAttachInterrupt flEncoder = JEncoderSingleAttachInterrupt(inport1, wheelCir / encoderTicksPerRev, false, 200000, 25, FALLING);
JEncoderSingleAttachInterrupt frEncoder = JEncoderSingleAttachInterrupt(inport2, wheelCir / encoderTicksPerRev, false, 200000, 25, FALLING);
JEncoderSingleAttachInterrupt blEncoder = JEncoderSingleAttachInterrupt(inport3, wheelCir / encoderTicksPerRev, false, 200000, 25, FALLING);
JEncoderSingleAttachInterrupt brEncoder = JEncoderSingleAttachInterrupt(port1Pin, wheelCir / encoderTicksPerRev, false, 200000, 25, FALLING);
JMotorCompStandardConfig flMotorConfig = JMotorCompStandardConfig(3, .55, 5, 1.60, 6.75, 2.75, 100);
JMotorCompStandardConfig frMotorConfig = JMotorCompStandardConfig(3, .55, 5, 1.60, 6.75, 2.75, 100);
JMotorCompStandardConfig blMotorConfig = JMotorCompStandardConfig(3, .55, 5, 1.60, 6.75, 2.75, 100);
JMotorCompStandardConfig brMotorConfig = JMotorCompStandardConfig(3, .55, 5, 1.60, 6.75, 2.75, 100);
JMotorCompStandard flMotorCompensator = JMotorCompStandard(voltageComp, flMotorConfig, 1.0 / (wheelCir)); // factor converts from ground speed to rotations per second
JMotorCompStandard frMotorCompensator = JMotorCompStandard(voltageComp, frMotorConfig, 1.0 / (wheelCir));
JMotorCompStandard blMotorCompensator = JMotorCompStandard(voltageComp, blMotorConfig, 1.0 / (wheelCir));
JMotorCompStandard brMotorCompensator = JMotorCompStandard(voltageComp, brMotorConfig, 1.0 / (wheelCir));
JControlLoopBasic flCtrlLoop = JControlLoopBasic(7);
JControlLoopBasic frCtrlLoop = JControlLoopBasic(7);
JControlLoopBasic blCtrlLoop = JControlLoopBasic(7);
JControlLoopBasic brCtrlLoop = JControlLoopBasic(7);
JMotorControllerClosed flMotor = JMotorControllerClosed(flMotorDriver, flMotorCompensator, flEncoder, flCtrlLoop);
JMotorControllerClosed frMotor = JMotorControllerClosed(frMotorDriver, frMotorCompensator, frEncoder, frCtrlLoop);
JMotorControllerClosed blMotor = JMotorControllerClosed(blMotorDriver, blMotorCompensator, blEncoder, blCtrlLoop);
JMotorControllerClosed brMotor = JMotorControllerClosed(brMotorDriver, brMotorCompensator, brEncoder, brCtrlLoop);
JDrivetrainMecanum drivetrain = JDrivetrainMecanum(frMotor, flMotor, blMotor, brMotor, robotToWheelScalar); // drivetrain converts from robot speed to motor speeds
JDrivetrainControllerBasic drivetrainController = JDrivetrainControllerBasic(drivetrain, { 1, 1, 1 }, { 1, 1, 1 }, { 1, 1, 1 });

void Enabled()
{
    // code to run while enabled, put your main code here
}

void Enable()
{
    // turn on outputs
    drivetrain.enable();
}

void Disable()
{
    // shut off all outputs
    drivetrain.disable();
}

void PowerOn()
{
    // runs once on robot startup, set pin modes and use begin() if applicable here
    Wire.begin();
}

void Always()
{
    // always runs if void loop is running, top level JMotor run() functions should be put here
    drivetrain.run();

    delay(1);
}

void configWifi()
{
    EWD::mode = EWD::Mode::connectToNetwork;
    EWD::routerName = "router";
    EWD::routerPassword = "password";
    EWD::routerPort = 25210;

    // EWD::mode = EWD::Mode::createAP;
    // EWD::APName = "rcm0";
    // EWD::APPassword = "rcmPassword";
    // EWD::APPort = 25210;
}

void WifiDataToParse()
{
    enabled = EWD::recvBl();
    // add data to read here: (EWD::recvBl, EWD::recvBy, EWD::recvIn, EWD::recvFl)(boolean, byte, int, float)
}
void WifiDataToSend()
{
    EWD::sendFl(voltageComp.getSupplyVoltage());
    // add data to send here: (EWD::sendBl(), EWD::sendBy(), EWD::sendIn(), EWD::sendFl())(boolean, byte, int, float)
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
