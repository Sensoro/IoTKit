#include "IOTModule.h"
#include <Wire.h>
#include "OPT3001.h"
#include "SHT3XD.h"
#include "Moto.h"
#include "LIS3MDL.h"
#include "LP3944.h"
#include "Adafruit_LIS3DH.h"
#include "Adafruit_Sensor.h"
#include <SoftwareSerial.h>


#define OPT3001_I2CADDR     0X45
#define SHT3XD_I2CADDR      0X44
#define LP3944_I2CADDR      0x60
#define LIS3DH_I2CADDR      0X19

#define MOTO_ANODE_PIN            7
#define MOTO_CATHODE_PIN          2

#define DEFAULT_BLE_TXP     0
#define DEFAULT_BLE_INTV    100

#define led1_red            LP3944_LED0
#define led1_green          LP3944_LED1
#define led1_blue           LP3944_LED2


#define CMD_LED             0X00
#define CMD_MOTO            0X01


IOTModule   module;
OPT3001     lightSensor(OPT3001_I2CADDR);
SHT3XD      tempHumiSensor(SHT3XD_I2CADDR);
Moto        moto(MOTO_ANODE_PIN, MOTO_CATHODE_PIN);
LIS3MDL     magSensor;
LP3944      ledDriver(LP3944_I2CADDR);

Adafruit_LIS3DH lis = Adafruit_LIS3DH();

/*Iot收数据结构体*/
IotRcvData_t IotRcvDataArray[2];
uint8_t IotRcvDataArrayLen = 0;

/*用于存放蓝牙广播数据*/
uint8_t BleAdvData[19] = {0};
uint8_t BleAdvDataLen = 0;

void lightSensorCfg();

/*用于存放Iot数据*/
static uint8_t TxBuff[32];
static uint8_t TxBuffLen = 0;

void setup() {

  int8_t bleTxp = 0;
  uint16_t bleIntv = 0;

  Serial.begin(115200);
  while (!Serial);
  Serial.println("start");

  Wire.begin();

  /*初始化电机*/
  moto.begin();

  /*连接通信模块*/
  if ( 0 != module.begin(9600)) {
    Serial.println("module start error");
    while (1);
  }
  Serial.println("module start success!!");

  /*光感初始化*/
  if ( NO_ERROR != lightSensor.begin() ) {
    Serial.println("Failed to begin light sensoro!");
    while (1);
  }
  Serial.println("lightSensor init success!");

  /*温湿度传感器初始化*/
  if ( SHT3XD_NO_ERROR != tempHumiSensor.begin() ) {
    Serial.println("Failed to begin temp&humi sensoro!");
    while (1);
  }
  Serial.println("tempHumiSensor init success!");

  /*地磁传感器初始化*/
  if (!magSensor.init()) {
    Serial.println("Failed to initialize magnetometer!");
    while (1);
  }
  Serial.println("magSensor init success!");

  /*加速度传感器初始化*/
  if (! lis.begin(LIS3DH_I2CADDR)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Failed to begin accelerator!");
    while (1);
  }
  Serial.println("accSensor init success!");

  /*led驱动初始化*/
  if (!ledDriver.begin()) {
    Serial.println("Failed to initialize ledDriver!");
  }
  /*光感配置*/
  lightSensorCfg();
  /*地磁传感器配置*/
  magSensor.enableDefault();
  /*加速度传感器配置*/
  lis.setRange(LIS3DH_RANGE_2_G);

  module.wakeUp();

  /*获取蓝牙广播功率，广播间隔*/
  if ( 0 != module.BleTxpIntvGet(&bleTxp, &bleIntv, 2)) {
    Serial.println(" BleTxpIntvGet error");
    while (1);
  }

  Serial.print("bleTxp = ");
  Serial.println(bleTxp);
  Serial.print("bleIntv = ");
  Serial.println(bleIntv);

  bleTxp = DEFAULT_BLE_TXP;
  bleIntv = DEFAULT_BLE_INTV;

  /*设置蓝牙广播功率，广播间隔*/
  if ( 0 != module.BleTxpIntvSet(bleTxp, bleIntv)) {
    Serial.println("BleTxpIntvSet error");
    while (1);
  }

  Serial.println("Setup done!");
}

void loop() {
  int len;
  uint8_t rx_buff[50];
  uint32_t ret_id = 0;

  module.wakeUp();

  prepareIotPkt0AndBleData();
  Serial.println("Set BLE data");

  /*设置蓝牙广播，将传感器数据广播出去*/
  if ( 0 != module.BleDataSet(BleAdvData, BleAdvDataLen)) {
    Serial.println("BleDataSet error");
  }
  Serial.println("Send IotPkt0");

  // TODO: get id from fifo, now just get the first id
  if (IotRcvDataArrayLen != 0 )
  {
    ret_id = IotRcvDataArray[0].id;
    Serial.print("ret_id ="); Serial.println(ret_id);
    //    IotRcvDataArrayLen = 0;
  }

  /*发送Iot数据到基站*/
  if ( 0 != module.IotSend(TxBuff, TxBuffLen, false, ret_id) ) {
    Serial.println("send error");
  }

  /*查询是否有数据待接收*/
  if ( 0 == module.IotDataQury(&len) ) {
    if (len != 0) {
      Serial.println("there are some data to receive");

      /*接收数据*/
      if ( 0 == module.IotDataRcv(rx_buff, len) ) {

        /*解析数据*/
        module.IotDataParse(rx_buff, len, IotRcvDataArray, &IotRcvDataArrayLen );
        for (uint8_t i = 0; i < IotRcvDataArrayLen; i++) {
          Serial.print("subpkt "); Serial.print(i); Serial.println(":");
          Serial.print("id = "); Serial.println(IotRcvDataArray[i].id);
          Serial.print("data = "); dump_data(IotRcvDataArray[i].data, IotRcvDataArray[i].data_len);
          //TODO: store id into a fifo

          /*处理数据*/
          processRcvData(IotRcvDataArray[i].data, IotRcvDataArray[i].data_len);
        }
      }
    } else {
      Serial.println("NO data to receive");
    }
  }
  module.sleep();
  delay(1000);
}

/* @brief 处理接收到的数据
 */
void processRcvData(uint8_t *data, uint8_t len)
{
  uint8_t type = 0;
  if (len != 2 && len != 4) {
    Serial.println("Invalid Rcv Data");
  }
  type = data[0];
  switch (type) {
    case CMD_LED:
      {
        uint8_t red = data[1];
        uint8_t green = data[2];
        uint8_t blue = data[3];
        if (red > 0) {
          ledDriver.setLedStatus(led1_red, LP3944_LED_STATUS_ON);
        } else {
          ledDriver.setLedStatus(led1_red, LP3944_LED_STATUS_OFF);
        }
        if (green > 0) {
          ledDriver.setLedStatus(led1_green, LP3944_LED_STATUS_ON);
        } else {
          ledDriver.setLedStatus(led1_green, LP3944_LED_STATUS_OFF);
        }
        if (blue > 0) {
          ledDriver.setLedStatus(led1_blue, LP3944_LED_STATUS_ON);
        } else {
          ledDriver.setLedStatus(led1_blue, LP3944_LED_STATUS_OFF);
        }
      }
      break;
    case CMD_MOTO:
      {
        uint8_t moto_mode = data[1];
        if (moto_mode == 0x00) {
          moto.stop();
        } else if (moto_mode == 0x01) {
          moto.rotate();
        } else if (moto_mode == 0x02) {
          moto.reversal();
        } else {
          moto.stop();
        }
      }
      break;
  }

}

/* @brief 采集温度湿度光线数据，放到TxBuff里（第0byte为前缀0x00）及BleAdvData里，
 */
void prepareIotPkt0AndBleData(void)
{
  BleAdvDataLen = 0;
  TxBuffLen = 0;
  TxBuff[TxBuffLen++] = 0x00;
  SHT3XD_Result shtResult = tempHumiSensor.readTempAndHumidity(REPEATABILITY_LOW, MODE_CLOCK_STRETCH, 50);
  if ( NO_ERROR == shtResult.error) {
    Serial.print("temp ="); Serial.println(shtResult.t);
    Serial.print("humi ="); Serial.println(shtResult.rh);
    memcpy(&TxBuff[TxBuffLen], &shtResult.t, 4);
    TxBuffLen += 4;
    memcpy(&TxBuff[TxBuffLen], &shtResult.rh, 4);
    TxBuffLen += 4;

    memcpy(&BleAdvData[BleAdvDataLen], &shtResult.t, 4);
    BleAdvDataLen += 4;
    memcpy(&BleAdvData[BleAdvDataLen], &shtResult.rh, 4);
    BleAdvDataLen += 4;
  } else {
    Serial.println("Read temp and humidity error");
    memset(&TxBuff[TxBuffLen], 0, 4);
    TxBuffLen += 4;
    memset(&TxBuff[TxBuffLen], 0, 4);
    TxBuffLen += 4;
  }

  OPT3001_Result optResult = lightSensor.readResult();
  if ( NO_ERROR == optResult.error) {
    Serial.print("lux ="); Serial.println(optResult.lux);
    memcpy(&TxBuff[TxBuffLen], &optResult.lux, 4);
    TxBuffLen += 4;

    memcpy(&BleAdvData[BleAdvDataLen], &optResult.lux, 4);
    BleAdvDataLen += 4;
  } else {
    Serial.println("Read light error");
    memset(&TxBuff[TxBuffLen], 0, 4);
    TxBuffLen += 4;
  }
  //  Serial.println("magSensor.read()");
  //  magSensor.read();
  //  //  magSensor.vector_normalize(&magData);
  //  float x_gs = (float)magSensor.m.x / 6842;
  //  float y_gs = (float)magSensor.m.y / 6842;
  //  float z_gs = (float)magSensor.m.z / 6842;
  //
  //  //  Serial.print("mag X ="); Serial.print(magSensor.m.x);
  //  //  Serial.print(", mag Y ="); Serial.print(magSensor.m.y);
  //  //  Serial.print(", mag Z ="); Serial.println(magSensor.m.z);
  //  Serial.print("mag X ="); Serial.print(x_gs);
  //  Serial.print(", mag Y ="); Serial.print(y_gs);
  //  Serial.print(", mag Z ="); Serial.println(z_gs);
  //  memcpy(&TxBuff[TxBuffLen], &magSensor.m.x, 4);
  //  TxBuffLen += 4;
  //  memcpy(&TxBuff[TxBuffLen], &magSensor.m.y, 4);
  //  TxBuffLen += 4;
  //  memcpy(&TxBuff[TxBuffLen], &magSensor.m.z, 4);
  //  TxBuffLen += 4;

}

/* @brief 采集加速度计数据，放到TxBuff里（第0byte为前缀0x01）
 */
void prepareIotPkt1(void)
{
  TxBuffLen = 0;
  TxBuff[TxBuffLen++] = 0x01;

  lis.read();      // get X Y and Z data at once
  Serial.print("X:  "); Serial.print(lis.x_g);
  Serial.print("  \tY:  "); Serial.print(lis.y_g);
  Serial.print("  \tZ:  "); Serial.println(lis.z_g);

  memcpy(&TxBuff[TxBuffLen], &lis.x_g, 4);
  TxBuffLen += 4;
  memcpy(&TxBuff[TxBuffLen], &lis.y_g, 4);
  TxBuffLen += 4;
  memcpy(&TxBuff[TxBuffLen], &lis.z_g, 4);
  TxBuffLen += 4;
}

void dump_data(uint8_t *p_buff, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(p_buff[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

/* @brief 光感配置
 */
void lightSensorCfg() {

  OPT3001_Config newConfig;

  newConfig.RangeNumber = B1100;
  newConfig.ConvertionTime = B0;
  newConfig.Latch = B1;
  newConfig.ModeOfConversionOperation = B11;

  OPT3001_ErrorCode errorConfig = lightSensor.writeConfig(newConfig);
  if (errorConfig != NO_ERROR)
    Serial.println("OPT3001 configuration error");
}
