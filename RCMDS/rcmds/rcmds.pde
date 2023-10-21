/////////////////////////add interface elements here
//   Joystick(float _xPos, float _yPos, float _size, float _xRange, float _yRange, color _background, color _stick, String _xa, String _ya, int _upKey, int _leftKey, int _downKey, int _rightKey, int _xTilt, int _yTilt) {
Joystick movement;
//  Slider(float _xPos, float _yPos, float _size, float _width, float _low, float _high, color _background, color _stick, String _ga, int _pKey, int _mKey, float _inc, int _tilt, boolean _horizontal, boolean _reverse) {
Slider turning;
//////////////////////
float batVolt=0.0;
boolean enabled=false;
////////////////////////add variables here
PVector moveVal=new PVector(0, 0);
float turnVal=0.0;

float velx=0;
float vely=0;
float velt=0;

float heading=0;

byte mode=0;

float DTH=0;

void setup() {
  size(1400, 800);
  rcmdsSetup();
  //setup UI here
  setupGamepad("Controller (XBOX 360 For Windows)");
  movement=new Joystick(200, 300, 200, -1, 1, color(100), color(200), "X Axis", "Y Axis", 'i', 'j', 'k', 'l', 0, 0);
  turning=new Slider(200, 600, 200, 50, -1, 1, color(100), color(200), "X Rotation", 'd', 'a', 1, 0, true, true);
}
void draw() {
  background(0);
  runWifiSettingsChanger();
  enabled=enableSwitch.run(enabled);
  /////////////////////////////////////add UI here
  moveVal=movement.run(new PVector(0, 0));
  turnVal=turning.run(0);

  if (keyPressed&&key=='1') {
    mode=1;
  }
  if (keyPressed&&key=='2') {
    mode=2;
  }
  if (keyPressed&&key=='3') {
    mode=3;
  }
  if (keyPressed&&key=='4') {
    mode=4;
  }

  String[] msg={"battery voltage", "ping", "forwards", "left", "clockwise", "v f", "v l", "v t", "heading", "mode", "DiffHeading"};
  String[] data={str(batVolt), str(wifiPing), str(moveVal.y), str(moveVal.x), str(turnVal), str(velx), str(vely), str(velt), str(heading), str(mode), str(DTH)};
  dispTelem(msg, data, width/2, height*2/3, width/4, height*2/3, 12);

  sendWifiData(true);
  endOfDraw();
}
void WifiDataToRecv() {
  batVolt=recvFl();
  ////////////////////////////////////add data to read here
  velx=recvFl();
  vely=recvFl();
  velt=recvFl();
  heading=recvFl();
  DTH=recvFl();
}
void WifiDataToSend() {
  sendBl(enabled);
  ///////////////////////////////////add data to send here
  sendBy(mode);
  if (mode==1) {
    sendFl(moveVal.y*.5);
    sendFl(moveVal.x*.5);
    sendFl(turnVal);
  }
  if (mode==3) {
    sendBl(moveVal.x>0.2);
    sendBl(moveVal.x<-.2);
  }
  if (mode==4) {
    sendBl(abs(moveVal.x)>0.2||abs(moveVal.y)>0.2);
  }
}
