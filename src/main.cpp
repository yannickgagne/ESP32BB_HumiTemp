/*******************************************************************************
* Copyright (c) 2023 Yannick Gagne
*
* Permission is hereby granted, free of charge, to anyone
* obtaining a copy of this document and accompanying files,
* to do whatever they want with them without any restriction,
* including, but not limited to, copying, modification and redistribution.
* NO WARRANTY OF ANY KIND IS PROVIDED.
*
* This example provides basic demonstration of all functionality available
* on the board.
*
*******************************************************************************/
#include <Arduino.h>
#include <secrets.h>
#include <OneWire.h>
#include <Wire.h>
#include <SPI.h>
#include <ADS1X15.h>
#include <SparkFunBQ27441.h>
#include <DallasTemperature.h>
#include <lmic.h>
#include <hal/hal.h>
#include <Preferences.h>
#include <helpers.h>

#define DEBUG
#define LORA

uint32_t Freq = 0;

//LMIC
static const u1_t PROGMEM APPEUI[8] = TTN_APPEUI;
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8]= TTN_DEVEUI;
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = TTN_APPKEY;
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t payload[12];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 5*60;

unsigned long stime;

bool GOTO_DEEPSLEEP = false;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 32,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 33,
    .dio = {26, 27, LMIC_UNUSED_PIN},
};

lmic_t NVS_LMIC;

//NVS Erase Pin
const int nvsKillPin = 5;

//ADS1x15
const int adsRdyPin = 17;
ADS1015 ADS(0x48);

//BQ27441
const unsigned int BATTERY_CAPACITY = 4000;
unsigned int soc = 0;
unsigned int volts = 0;
int current = 0;

//3v3
const int enablePin3v3 = 25;

//Temperature sensors (x2)
const int tempSensorsPin = 16;
OneWire ow(tempSensorsPin);
DallasTemperature ts(&ow);
DeviceAddress ts1 = { 0x28, 0xFF, 0x64, 0x2, 0xEB, 0x2F, 0x9D, 0x72 };
DeviceAddress ts2 = { 0x28, 0xFF, 0x64, 0x2, 0xEB, 0x51, 0xBE, 0xD9 };

//Preferences
Preferences prefs;

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void getSensorsData() {
  //BQ27441
  for(int i = 0; i < 6; i++) {
    // Read battery stats from the BQ27441-G1A
    soc = lipo.soc();  // Read state-of-charge (%)
    volts = lipo.voltage(); // Read battery voltage (mV)
    current = lipo.current(AVG); // Read average current (mA)
    delay(10);
  }

  float fcurrent = (current * 1000) / 100;
  uint16_t pcurrent = LMIC_f2sflt16(fcurrent);

  Serial.print("current:"); Serial.println(current);
  Serial.print("fcurrent:"); Serial.println(fcurrent);
  Serial.print("pcurrent:"); Serial.println(pcurrent);

  byte mvLow = lowByte(volts);
  byte mvHigh = highByte(volts);
  byte maLow = lowByte(pcurrent);
  byte maHigh = highByte(pcurrent);
  byte socLow = lowByte(soc);
  byte socHigh = highByte(soc);

  //ADC reading from ADS1015 for moisture sensor
  int16_t moistRaw = ADS.getValue();
  byte moistLow = lowByte(moistRaw);
  byte moistHigh = highByte(moistRaw);

  //Inside & outside temp
  ts.requestTemperatures();
  int16_t inTemp = ts.getTempC(ts1) * 100;
  byte inLow = lowByte(inTemp);
  byte inHigh = highByte(inTemp);

  int16_t outTemp = ts.getTempC(ts2) * 100;
  byte outLow = lowByte(outTemp);
  byte outHigh = highByte(outTemp);

  
  //Store the values in lmic payload
  payload[0] = mvLow;
  payload[1] = mvHigh;
  payload[2] = maLow;
  payload[3] = maHigh;
  payload[4] = socLow;
  payload[5] = socHigh;
  payload[6] = moistLow;
  payload[7] = moistHigh;
  payload[8] = inLow;
  payload[9] = inHigh;
  payload[10] = outLow;
  payload[11] = outHigh;
}

void do_send(osjob_t* j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        Serial.println("Getting sensors data...");
        getSensorsData();
        
        Serial.print("payload: ");
        for (int x = 0; x < sizeof(payload); x++) {
          Serial.print(payload[x], HEX);
          Serial.print(" ");
        }
        Serial.println(" ");
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }

            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

            GOTO_DEEPSLEEP = true;
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void SaveLMICToNVS(int deepsleep_sec)
{
    Serial.println(F("Save LMIC to NVS"));
    NVS_LMIC = LMIC;

    // ESP32 can't track millis during DeepSleep and no option to advanced millis after DeepSleep.
    // Therefore reset DutyCyles

    unsigned long now = millis();

    Serial.println(F("Adjust globalDuty"));

    NVS_LMIC.globalDutyAvail = NVS_LMIC.globalDutyAvail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
    if (NVS_LMIC.globalDutyAvail < 0)
    {
        NVS_LMIC.globalDutyAvail = 0;
    }

    prefs.putBytes("lmicst", &NVS_LMIC, sizeof(NVS_LMIC));
    prefs.putBool("isSaved", true);
    Serial.print("The size of \"lmicst\" is (in bytes): ");
    Serial.println( prefs.getBytesLength("lmicst") );
}

void deepSleepRoutine() {
    Serial.print("Size of LMIC struct: ");Serial.print(sizeof(LMIC));Serial.println(" bytes");
    Serial.print("Elapsed time (ms): "); Serial.println(millis()-stime);
    Serial.flush();
    ADS.reset();
    Wire.end();
    digitalWrite(enablePin3v3, LOW);
    SaveLMICToNVS(TX_INTERVAL);
    LMIC_shutdown();
    Serial.println("Going into Deep Sleep");
    esp_sleep_enable_timer_wakeup(TX_INTERVAL * 1000000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
}

void testSensors() {
  //ADS1x15
  int16_t raw = ADS.getValue();
  Serial.print("ADC0: ");Serial.println(raw);

  //BQ27441
  for(int i = 0; i < 6; i++) {
    // Read battery stats from the BQ27441-G1A
    soc = lipo.soc();  // Read state-of-charge (%)
    volts = lipo.voltage(); // Read battery voltage (mV)
    current = lipo.current(AVG); // Read average current (mA)
    delay(10);
  }

  // Now print out those values:
  String battInfo = String(soc) + "% | ";
  battInfo += String(volts) + " mV | ";
  battInfo += String(current) + " mA";
  
  Serial.println(battInfo);

  //Temp sensors (x2)
  ts.requestTemperatures();
  Serial.print("Sensor #1 (C): ");Serial.println(ts.getTempC(ts1));
  Serial.print("Sensor #2 (C): ");Serial.println(ts.getTempC(ts2));
}
 
void setup() {
  stime = millis();
  Serial.begin(115200);
  while(!Serial) {
    ; //Wait for serial port to connect.
  }

  Serial.println("Setup started...");

  setCpuFrequencyMhz(80);

  Freq = getCpuFrequencyMhz();
  Serial.print("CPU Freq (MHz): "); Serial.println(Freq);

  //Pin to clear preferences. LOW=KEEP, HIGH=ERASE
  pinMode(nvsKillPin, INPUT);

  //3v3
  pinMode(enablePin3v3, OUTPUT);
  digitalWrite(enablePin3v3, HIGH); //Turn ON switched 3v3
  delay(100);

  //Define Wire pins
  Wire.begin(21, 22);

  //ADS1x15
  pinMode(adsRdyPin, INPUT);
  ADS.begin();
  if(!ADS.isConnected()) {
    Serial.println("ERROR: Cannot connect to ADS1x15!");
  } else {
    ADS.setGain(1); // 4.096 volt
    ADS.setDataRate(4); //1600 SPS for ADS1015
    ADS.setMode(0); //continuous mode
    ADS.readADC(0); //first read to trigger
  }

  //BQ27441
  if(!lipo.begin()) {
    Serial.println("ERROR: Cannot connect to BQ27441!");
  }
  lipo.setCapacity(BATTERY_CAPACITY);
  
  //Temp sensors (x2)
  pinMode(tempSensorsPin, INPUT);
  ts.begin();

  //Preferences
  prefs.begin("lmic", false);
  
  #ifdef LORA
    //LMIC
    os_init_ex(&lmic_pins);
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    
    //Clear NVS if IO5 is HIGH
    if (digitalRead(nvsKillPin) == HIGH) {
      prefs.clear();
      Serial.println("Cleared data from preferences.");
    }

    //Load saved session info if present
    if (prefs.getBool("isSaved", false) == true) {
        /*
        Serial.println("Printing RTC_LMIC...");
        unsigned char cbuf1[720];
        memcpy(cbuf1, (const unsigned char*)&RTC_LMIC, sizeof(RTC_LMIC));
        for(int i = 0; i < sizeof(cbuf1); i++) {
            Serial.print(cbuf1[i], HEX);
        }
        Serial.println("");
        */
        Serial.println("Printing lmicst bytes...");
        unsigned char saved[720];
        prefs.getBytes("lmicst", saved, prefs.getBytesLength("lmicst"));
        for (int i = 0; i < sizeof(saved); i++) {
            Serial.print(saved[i], HEX);
        }
        Serial.println("");

        memcpy(&NVS_LMIC, saved, sizeof(saved));
        Serial.println("Restored LMIC from Preferences.");
    }

    if (NVS_LMIC.seqnoUp != 0)
    {
        LMIC = NVS_LMIC;
        Serial.println("Using LMIC from NVS.");
    }

    LMIC_selectSubBand(1); //from Adafruit sample
    //LMIC_setDrTxpow(DR_SF7, 14);
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);

    Serial.print("DEVADDR: "); Serial.println(LMIC.devaddr, HEX);
  #else
    Serial.println("WARNING: LoRaWAN Deactivated");
  #endif

  #ifdef DEBUG
    Serial.println("INFO: Setup done.");
  #endif
}

void loop() {
  #ifdef LORA
    static unsigned long lastPrintTime = 0;

    //LMIC
    os_runloop_once();

    const bool timeCriticalJobs = os_queryTimeCriticalJobs(ms2osticksRound((TX_INTERVAL * 1000)));
    if (!timeCriticalJobs && GOTO_DEEPSLEEP == true && !(LMIC.opmode & OP_TXRXPEND))
    {
      Serial.println("We can deep sleep");
      deepSleepRoutine();
    } else if (lastPrintTime + 2000 < millis()) {
        Serial.print(F("Cannot sleep "));
        Serial.print(F("TimeCriticalJobs: "));
        Serial.print(timeCriticalJobs);
        Serial.print(" ");

        //LoraWANPrintLMICOpmode();
        //PrintRuntime();
        lastPrintTime = millis();
    }
  #else
    testSensors();
    delay(5000);
  #endif
}