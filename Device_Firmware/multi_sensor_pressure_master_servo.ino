
/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This sketch show how to use BLEClientService and BLEClientCharacteristic
 * to implement a custom client that is used to talk with Gatt server on
 * peripheral.
 *
 * Note: you will need another feather52 running peripheral/custom_HRM sketch
 * to test with.
 */

#include <bluefruit.h>
#include <Servo.h>

uint8_t custom_service[16] =
{
  0x57, 0x80, 0xD2, 0x94, 0xA3, 0xB2, 0xFE, 0x39,
  0x5F, 0x87, 0xFD, 0x35, 0x90, 0xAE, 0x8B, 0x22
};
uint8_t custom_char1[16] =
{
  0x57, 0x80, 0xD2, 0x94, 0xA3, 0xB2, 0xFE, 0x39,
  0x5F, 0x87, 0xFD, 0x35, 0x91, 0xAE, 0x8B, 0x22
};
uint8_t custom_char2[16] =
{
  0x57, 0x80, 0xD2, 0x94, 0xA3, 0xB2, 0xFE, 0x39,
  0x5F, 0x87, 0xFD, 0x35, 0x92, 0xAE, 0x8B, 0x22
};
uint8_t custom_service2[16] =
{
  0x57, 0x80, 0xD2, 0x94, 0xA3, 0xB2, 0xFE, 0x39,
  0x5F, 0x87, 0xFD, 0x35, 0x93, 0xAE, 0x8B, 0x22
};

// Name : SensingGlove
uint8_t SENSORTAG_ADV_COMPLETE_LOCAL_NAME[] =            {0x53,0x65,0x6E,0x73,0x69,0x6E,0x67,0x47,0x6C,0x6F,0x76,0x65};  

Servo myservo[3];  // create servo objects to control a servo
Servo myservo_pressure[3];  // create servo objects to control a servo
const int vibration_pin[3] = {A5, A7, 11};
int val = 0;

BLEClientService        strain_temperature(custom_service);
BLEClientCharacteristic strain(0xAE91);
BLEClientCharacteristic temperature(0xAE92);
BLEClientService        pressure_service(custom_service2);
BLEClientCharacteristic pressure(0xAE94);

void setup()
{
  myservo_pressure[0].attach(A3); 
  myservo_pressure[1].attach(A1);
  myservo_pressure[2].attach(A2);

  myservo[0].attach(A0); 
  myservo[1].attach(A4);
  //myservo[2].attach(A4);
  
  for(int i = 0; i < 3; i++)
  {
    pinMode(vibration_pin[i], OUTPUT);
  }
  
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Sensing Glove Application Example");
  Serial.println("--------------------------------------\n");

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);

  Bluefruit.setName("Bluefruit52 Central");

  // Initialize services
  strain_temperature.begin();  

  // Initialize  characteristics
  // Note: Client Char will be added to the last service that is begin()ed.
  strain.setNotifyCallback(strain_notify_callback);
  strain.begin();

  // set up callback for receiving measurement
  temperature.setNotifyCallback(temperature_notify_callback);
  temperature.begin();

  // Initialise pressure service
  pressure_service.begin();

  // Initialize client characteristics of HRM.
  // Note: Client Char will be added to the last service that is begin()ed.
  pressure.setNotifyCallback(pressure_notify_callback);
  pressure.begin();

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(40);  // Default value was 250

  // Callbacks for Central
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  Bluefruit.Central.setConnectCallback(connect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Filter only accept HRM service
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms // 160, 80 (default)
 // Bluefruit.Scanner.filterUuid(strain_temperature.uuid);
  Bluefruit.Scanner.useActiveScan(true);  // false
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds
  Serial.println("Started scanning");
}

void loop()
{
  // do nothing
  digitalToggle(LED_RED);
  delay(2000);
}

/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Since we configure the scanner with filterUuid()
  // Scan callback only invoked for device with hrm service advertised
  // Connect to device with HRM service in advertising
 // Serial.println("Inside scan callback");

uint8_t buffer[BLE_GAP_ADV_SET_DATA_SIZE_MAX] = { 0 };

 // Serial.print("Parsing report for Local Name ... ");
  if(Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)))
  { 
    if ( !memcmp( buffer, SENSORTAG_ADV_COMPLETE_LOCAL_NAME, sizeof(SENSORTAG_ADV_COMPLETE_LOCAL_NAME)) )
    {
      Serial.println("Name Match!");

      Serial.println("Connecting to Peripheral ... ");
      Bluefruit.Central.connect(report);
    }
    else
    {
      Bluefruit.Scanner.resume(); // continue scanning
    } 
  } 
  else
  {
    // We need to call Scanner resume() to continue scanning
    Bluefruit.Scanner.resume();
  }  
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle)
{
  Serial.println("Connected");
  Serial.print("Discovering Services ... ");

 BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("\nConnected to ");
  Serial.println(central_name);
  
  // If HRM is not found, disconnect and return
  if ( !strain_temperature.discover(conn_handle) )
  {
    Serial.println("Not found strain temperature service");   
  }
  else
  {
    // Once HRM service is found, we continue to discover its characteristic
    Serial.println("Found the strain temperature service");
  }
  
  Serial.print("Discovering Strain characteristic ... ");
  if ( !strain.discover() )
  {
    Serial.println("not found Strain characteristic!!!");   
  }
  else
  {
    Serial.println("Found Strain characteristic!!");
  }
  
  if ( strain.enableNotify() )
  {
    Serial.println("Ready to receive Strain Measurement value");
  }else
  {
    Serial.println("Couldn't enable notify for Strain Measurement. Increase DEBUG LEVEL for troubleshooting");
  }
  
  Serial.print("Discovering Temperature characteristic ... ");
  if ( temperature.discover() )
  {
    Serial.println("Found Temperature characteristic!!");    
  }
  else
  {
    Serial.println("Not found Temperature characteristic");
  }
  
  if ( temperature.enableNotify())
  {
    Serial.println("Ready to receive Temperature Measurement value");
  }
  else
  {
    Serial.println("Couldn't enable notify for Temperature Measurement. Increase DEBUG LEVEL for troubleshooting");
  }

  if ( !(pressure_service.discover(conn_handle)) )
  {
    Serial.println("Not found pressure service");   
  }
  else
  {  
    Serial.println("Found the pressure service");
  }

  if ( !pressure.discover() )
  {
    Serial.println("not found Pressure characteristic!!!");   
  }
  else
  {
    Serial.println("Found Pressure characteristic!!");
  }
  if ( pressure.enableNotify() )
  {
    Serial.println("Ready to receive Pressure Measurement value");
  }
  else
  {
    Serial.println("Couldn't enable notify for Pressure Measurement");
  }
  
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}


/**
 * Hooked callback that triggered when a measurement value is sent from peripheral
 * @param chr   Pointer client characteristic that even occurred,
 *              in this example it should be hrmc
 * @param data  Pointer to received data
 * @param len   Length of received data
 */
void strain_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
    uint8_t value[3] = {0};
    memcpy(value, data, 3);

    for(int i = 0; i < 3; i++)
    {
      //if(value[i] == 90) value[i] = 0;

      Serial.print("Strain Measurement ");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.println((uint16_t)(value[i] << 3));
      //val = map(value[i], 0, 90, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
      if(i == 1)
      {
        val = map((uint16_t)(value[i] << 3), 1000, 600, 0, 180); 
        myservo[0].write(val);
        myservo[1].write(val);
      }// sets the servo position according to the scaled value
    }
   
   // analogWrite(vib_pinT, 50 + 50 * (value + 1));      
}


void temperature_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
    uint8_t value[3] = {0};
    memcpy(value, data, 3);

    for(int i = 0; i < 3; i++)
    {
      Serial.print("Temperature Measurement ");
      Serial.print(i+1);
      Serial.print(": ");
      Serial.println((uint16_t)(value[i] << 3));
      
      if(i == 0)
      {
        if(((uint16_t)(value[i] << 3)) < 700)
          //analogWrite(vibration_pin[i], value[i] * 50 + 50);
          analogWrite(vibration_pin[i], 250);
        else
          analogWrite(vibration_pin[i], 0);
      }
    }
    /*
    val = map(value, 0, 4, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
    myservo_t.write(val);                  // sets the servo position according to the scaled value
    */      
}


void pressure_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{    
    uint8_t meas[16];
    memcpy(meas, data, 16);       

    for(int i = 0; i < 3; i++)
    {
        int32_t value = (int32_t)(((((uint32_t)(meas[4 * i + 1])) | (((uint32_t)meas[4 * i + 2]) << 8U) | (((uint32_t)meas[4 * i + 3]) << 16U))) << 8U);
        float capacitance = ((float)value / pow(2, 27));  // 19 + 8 because only 24 bits count
        Serial.print("Channel ");
        Serial.print(i+1);
        Serial.print(" Capacitance: ");
        Serial.print(capacitance,4);
        Serial.println("  pf, ");
        //int m = map((int)capacitance, 2, 10, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
        int m;
        if(i == 0)
          m = map((int)capacitance * 100, 700, 900, 180, 0);
        else if(i == 1)
          m = map((int)capacitance * 100, 700, 950, 180, 0);
        else
          m = map((int)capacitance * 100, 700, 950, 180, 0);
        myservo_pressure[i].write(m);                  // sets the servo position according to the scaled value
    }         
}
