
//All Includes
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>
#include <vector>

//Define IMU
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET 5
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t updatedPosition;

//Define GPS
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

//Define Bluetooth
BLEUart bleuart;

union floatUnion {
  float f;
  char floatBuff[sizeof(float)];
} fUnion;
union longUnion {
  long l;
  char longBuff[sizeof(long)];
} lUnion;

class PositionChange {
  public:
    float data[4];
    byte data_type;
    long timeStamp;
    PositionChange(long newTimeStamp, byte newDataType, float dataOne, float dataTwo, float dataThree, float dataFour) {
      timeStamp = newTimeStamp;
      data_type = newDataType;
      data[0] = dataOne;
      data[1] = dataTwo;
      data[2] = dataThree;
      data[3] = dataFour;
    }

};

void connect_callback(uint16_t conn_handle)
{
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));
  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}



bool loopBack = false;
bool cyclicStart = false;
PositionChange *tracked[2000];
int currentIdx = 0;
int currentStartIdx = 0;

long gpsTimer = 0;
bool recording = false;
int recordingTime = 0;
long originalTimestamp = 0;

bool addData(class PositionChange *newPos) {
  loopBack = false;
  delete tracked[currentIdx];
  tracked[currentIdx] = newPos;
  currentIdx += 1;
  if (currentIdx >= 2000) {
    currentIdx = 0;
    cyclicStart = true;
    loopBack = true;
  }
  if (cyclicStart) {
    currentStartIdx = currentIdx;
  }
  return loopBack;
}

void saveSensorData(byte dataType, float writeData[]) {
  addData(new PositionChange(millis(), dataType, writeData[0], writeData[1], writeData[2], writeData[3]));

}

float updatePacket[4];
void updateIMUValues() {
  if (bno08x.getSensorEvent(&updatedPosition)) {
    switch (updatedPosition.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        updatePacket[0] = updatedPosition.un.gameRotationVector.real;
        updatePacket[1] = updatedPosition.un.gameRotationVector.i;
        updatePacket[2] = updatedPosition.un.gameRotationVector.j;
        updatePacket[3] = updatedPosition.un.gameRotationVector.k;

        saveSensorData(SH2_GAME_ROTATION_VECTOR, updatePacket);
        break;
      case SH2_LINEAR_ACCELERATION:
        updatePacket[0] = updatedPosition.un.linearAcceleration.x;
        updatePacket[1] = updatedPosition.un.linearAcceleration.y;
        updatePacket[2] = updatedPosition.un.linearAcceleration.z;
        saveSensorData(SH2_LINEAR_ACCELERATION, updatePacket);
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        updatePacket[0] = updatedPosition.un.gyroscope.x;
        updatePacket[1] = updatedPosition.un.gyroscope.y;
        updatePacket[2] = updatedPosition.un.gyroscope.z;
        saveSensorData(SH2_GYROSCOPE_CALIBRATED, updatePacket);
        break;
    }
  } else {
  }
}

long timeStamp = 0;
void updateGPSValues() {
  while (GPS.available()) { //Loop until you get NMEA
    GPS.read();
  }
  //  Serial.println(c);
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {
      if (GPS.fix) {
        if (timeStamp != GPS.milliseconds) {
          updatePacket[0] = GPS.latitude;
          updatePacket[1] = GPS.longitude;
          updatePacket[2] = GPS.altitude;
          saveSensorData(17, updatePacket);
          timeStamp = GPS.milliseconds;
        }

      }
    }
  }
}

void setup() {
  //Start debugging serial port at 115200 baudrate
  Serial.begin(115200);
  while ( !Serial ) yield();

  //Start GPS at 9600 baudrate
  GPS.begin(38400);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  //  GPS.sendCommand("$PGCMD, 33, 0*6D");
  Serial.println("GPS Started");

  //Start IMU sensor
  Serial.println(bno08x.begin_I2C());
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION);
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
  bno08x.enableReport(SH2_ROTATION_VECTOR);
  Serial.println("IMU started");

  //Setup bluetooth
  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Bluefruit52");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  bleuart.begin();
  Serial.println("Blueart begun");
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
  Serial.println("Bluetooth Advertising Started");

}

long startTime = 0;
void loop() {
  if (recording) {
    updateIMUValues();
    updateGPSValues();
    if (startTime == 0) {
      startTime = millis();
    }
    if (millis() - startTime > recordingTime) {
      Serial.println("recording ended");
      recording = false;
      startTime=0;
      Serial.println(currentIdx);
      Serial.println(currentStartIdx);
      Serial.println(currentIdx - currentStartIdx);
      for (int i = 0; i < currentIdx - currentStartIdx; i++) {
        Serial.println(i);
        long elapsedTime = tracked[i]->timeStamp - originalTimestamp;
        uint8_t dataToSend[21];

        lUnion.l = tracked[i]->timeStamp;
        dataToSend[0] = lUnion.longBuff[0];
        dataToSend[1] = lUnion.longBuff[1];
        dataToSend[2] = lUnion.longBuff[2];
        dataToSend[3] = lUnion.longBuff[3];
        dataToSend[4] = tracked[i]->data_type;

        fUnion.f = tracked[i]->data[0];
        dataToSend[5] = fUnion.floatBuff[0];
        dataToSend[6] = fUnion.floatBuff[1];
        dataToSend[7] = fUnion.floatBuff[2];
        dataToSend[8] = fUnion.floatBuff[3];

        fUnion.f = tracked[i]->data[1];
        dataToSend[9] = fUnion.floatBuff[0];
        dataToSend[10] = fUnion.floatBuff[1];
        dataToSend[11] = fUnion.floatBuff[2];
        dataToSend[12] = fUnion.floatBuff[3];

        fUnion.f = tracked[i]->data[2];
        dataToSend[13] = fUnion.floatBuff[0];
        dataToSend[14] = fUnion.floatBuff[1];
        dataToSend[15] = fUnion.floatBuff[2];
        dataToSend[16] = fUnion.floatBuff[3];

        fUnion.f = tracked[i]->data[3];
        dataToSend[17] = fUnion.floatBuff[0];
        dataToSend[18] = fUnion.floatBuff[1];
        dataToSend[19] = fUnion.floatBuff[2];
        dataToSend[20] = fUnion.floatBuff[3];


        bleuart.write(dataToSend, 21);
      }

    }

  } else {
    while ( bleuart.available() )
    {
      if ((int)bleuart.read() == 170) {
        recordingTime = bleuart.read() * 1000;
        recording = true;
        currentIdx = 0;
        currentStartIdx = 0;
      }
    }
  }
}
