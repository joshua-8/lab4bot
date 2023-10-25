//   This program controls an x drive robot
//   https://github.com/rcmgames/RCMv2
//   for information about the electronics, see the link at the top of this page: https://github.com/RCMgames
#include <ESP32_easy_wifi_data.h> //https://github.com/joshua-8/ESP32_easy_wifi_data >=v1.0.0

#define ONBOARD_LED 2

void WifiDataToParse()
{
}
void WifiDataToSend()
{
}

void setup()
{
    Serial.begin(115200);
    pinMode(ONBOARD_LED, OUTPUT);
    Serial.println();

    EWD::mode = EWD::Mode::createAP;
    EWD::APName = "yourAP";
    EWD::APPassword = "yourPassword";
    EWD::APPort = 25210;
    EWD::signalLossTimeout = 250;

    // EWD::setupWifi(WifiDataToParse, WifiDataToSend);
    EWD::sendCallback=WifiDataToSend;
    EWD::receiveCallback=WifiDataToParse;
    WiFi.disconnect(true, true);
    delay(100);

    WiFi.onEvent(EWD::WiFiEvent);

    if (!WiFi.softAP(EWD::APName, EWD::APPassword)) {
        log_e("Soft AP creation failed.");
        while (1)
            ;
    }
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
}

void loop()
{
    EWD::runWifiCommunication();
}
