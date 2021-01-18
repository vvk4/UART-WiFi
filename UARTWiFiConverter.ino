//UART - WiFi converter.
//Developed by Viacheslav Kulakov (vvk43210@gmail.com)
//The program is intended to work with the BLDC controller "Vector".
//Serial2 of ESP32 is connected with controller Vector.
//WiFi is connected to the PC programm via UDP protocol
//The network ssid and password is set via Serial0 in configuration mode.
//To enter the configuration mode, you has to connect D27 to GND and than connect the ESP32 to USB of PC.
//The specified parameters are stored in the EEPROM

//Blue LED modes:
//Blinking slowly: connecting with WiFi net.
//A short flash with a long interval: Connection is established, data from controller is not transmitting.
//Frequent short flashes: data from controller is transmitting.
//The LED is always on: configuration mode

#include <Arduino.h>
#include "WiFi.h"
#include "AsyncUDP.h"
#include "EEPROM.h"

#define LED 2
#define MODE_PIN 27

#define LED_1000 1000
#define LED_3000 3000
#define LED_50 50
#define LED_100 100

char ssid[40] = "your default ssid";
char password[65] = "your default password";
IPAddress SendIP(192, 168, 0, 255); //default host IP. Last octet has to be 255 (broadcast)

uint8_t UDPRecBuffer[1000];
uint8_t serialRecBuffer[1000];
uint8_t serialTrmBuffer[200];
uint8_t byteFromSerial, LEDStateMachine, LEDFlashStateMachine;
uint8_t byteFromSerialPrev;
int16_t i, ledCnt, cntLEDFlash;
uint16_t cntTimeOut, cntRec, cntTrmConfigMode;
hw_timer_t *timer = NULL;
bool gettingPacket, LEDFlash, trmConfigMode, timetTick;

enum LED_STATES
{
    LED_NO,
    LED_WiFiCONNECTING,
    LED_WiFiCONNECTED,
    LED_DATA_EXCHANGE,
    LED_WiFi_CONFIG
};
enum WiFi_CONFIG
{
    SET_WiFi_NET = 2
};

WiFiUDP udp;
EEPROMClass SSID_EEPROM("eeprom0", 0x500);
EEPROMClass PASSWORD_EEPROM("eeprom1", 0x200);
EEPROMClass IP_EEPROM("eeprom2", 0x100);

bool ReadPacket(void);
uint8_t CalcCheckSumm(uint16_t N, uint8_t *Mass);
void IRAM_ATTR onTimer();
void WiFiConfigMode(void);
void TrmSerial(void);
void TrmInfoToPCSerial(void);
void LEDFunc(void);
void ProcessCMDFromPC(void);
void ReadEEPROM(void);

void setup()
{
    Serial.begin(115200);

    Serial.println();

    LEDStateMachine = LED_WiFiCONNECTING;

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);
    Serial.println("Starting...");
    Serial.println();

    if (EEPROM.begin(1000))
    {
        ReadEEPROM();
        Serial.println("EEPROM Ok");
    }
    else
    {
        Serial.println("EEPROM failed");
    }

    pinMode(LED, OUTPUT);

    pinMode(MODE_PIN, INPUT_PULLUP);
    delay(500);

    if (!digitalRead(MODE_PIN))
    {
        delay(500);
        if (!digitalRead(MODE_PIN))
            WiFiConfigMode(); //Config mode
    }

    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin((const char *)ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        //        delay(500);
        //      Serial.print(".");
        LEDFunc();
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    SendIP = WiFi.localIP();
    SendIP[3] = 0xff;

    Serial2.begin(38400);

    udp.begin(51731);

    LEDStateMachine = LED_WiFiCONNECTED;
}

void loop()
{

    int packetSize = udp.parsePacket();

    while (Serial2.available() > 0)
    {

        byteFromSerial = Serial2.read();
        if (ReadPacket())
        {
            udp.beginPacket(SendIP, 51722);
            udp.write(serialRecBuffer, serialRecBuffer[2] + 4);
            udp.endPacket();
            LEDStateMachine = LED_DATA_EXCHANGE;
            LEDFlash = true;

            asm("Nop");
        }
        // Do something
    }

    if (packetSize)
    {
        udp.read(UDPRecBuffer, packetSize);
        asm("Nop");

        for (i = 0; i < UDPRecBuffer[2] + 4; i++)
            Serial2.write(UDPRecBuffer[i]);
    }

    LEDFunc();
}

bool ReadPacket(void)
{

    cntTimeOut = 500;

    if (!gettingPacket)
    {
        if ((byteFromSerial == byteFromSerialPrev) && (byteFromSerial == 0xff))
        {
            byteFromSerialPrev = 0;
            gettingPacket = true;
            cntRec = 2;
        }
        else
        {
            byteFromSerialPrev = byteFromSerial;
        }
    }
    else
    {
        if (cntRec > 256)
            gettingPacket = false;
        else
        {
            serialRecBuffer[cntRec] = byteFromSerial;
            cntRec++;
            if (cntRec > (serialRecBuffer[2] + 3))
            {
                gettingPacket = false;

                uint8_t check = CalcCheckSumm(serialRecBuffer[2] + 1, &serialRecBuffer[2]);

                if (check != serialRecBuffer[serialRecBuffer[2] + 3])
                {
                    return false;
                }
                else
                {
                    return true;
                }
            }
        }
    }
    return false;
}

uint8_t CalcCheckSumm(uint16_t N, uint8_t *Mass)
{
    uint16_t Summ = 0, j, n = N;

    for (j = 0; j < n; j++)
        Summ = Summ + Mass[j];

    Summ = ~Summ;

    return (uint8_t)Summ;
}

void IRAM_ATTR onTimer()
{

    if (cntTimeOut)
    {
        cntTimeOut--;
        if (!cntTimeOut)
        {
            gettingPacket = false;
        }
    }

    cntTrmConfigMode++;
    if (cntTrmConfigMode > 100)
    {
        cntTrmConfigMode = 0;
        trmConfigMode = true;
    }

    timetTick = true;
}

void WiFiConfigMode(void)
{
    LEDStateMachine = LED_WiFi_CONFIG;
    Serial.begin(38400);

    while (1)
    {
        if (trmConfigMode)
        {
            trmConfigMode = false;
            TrmInfoToPCSerial();
        }
        while (Serial.available() > 0)
        {

            byteFromSerial = Serial.read();
            if (ReadPacket())
            {
                ProcessCMDFromPC();
            }
        }
        LEDFunc();
    }
}

void LEDFunc(void)
{
    if (!timetTick)
        return;
    timetTick = false;

    switch (LEDStateMachine)
    {
    case LED_NO:
        digitalWrite(LED, LOW);
        break;
    case LED_WiFiCONNECTING:
    {
        if (ledCnt < LED_1000)
            ledCnt++;
        else
        {
            ledCnt = 0;
            if (digitalRead(LED))
                digitalWrite(LED, LOW);
            else
                digitalWrite(LED, HIGH);
        }
    }
    break;
    case LED_WiFiCONNECTED:
        if (ledCnt)
        {
            ledCnt--;
            if (!ledCnt)
            {
                if (digitalRead(LED))
                {
                    ledCnt = LED_3000;
                    digitalWrite(LED, LOW);
                }
                else
                {
                    digitalWrite(LED, HIGH);
                    ledCnt = LED_50;
                }
            }
        }
        else
            ledCnt = 1;

        break;
    case LED_DATA_EXCHANGE:

        switch (LEDFlashStateMachine)
        {
        case 0:
            if (LEDFlash)
            {
                cntLEDFlash = 1000;
                LEDFlash = 0;
                LEDFlashStateMachine = 1;
                digitalWrite(LED, LOW);
                ledCnt = LED_50;
            }
            break;
        case 1:
            ledCnt--;
            if (!ledCnt)
            {
                LEDFlashStateMachine = 2;
                ledCnt = LED_50;
                digitalWrite(LED, HIGH);
            }
            break;
        case 2:
            ledCnt--;
            if (!ledCnt)
            {
                LEDFlashStateMachine = 3;
                ledCnt = LED_50;
                digitalWrite(LED, LOW);
            }
            break;
        case 3:
            ledCnt--;
            if (!ledCnt)
            {
                LEDFlashStateMachine = 0;
                ledCnt = LED_100;
            }
            break;
        }
        break;
    case LED_WiFi_CONFIG:
        digitalWrite(LED, HIGH);
        break;
    default:
        LEDStateMachine = LED_NO;
        break;
    }

    if (cntLEDFlash)
    {
        cntLEDFlash--;
        if (!cntLEDFlash)
            LEDStateMachine = LED_WiFiCONNECTED;
    }
}

void TrmInfoToPCSerial(void)
{
    uint16_t cntBytes = 4, i;
    serialTrmBuffer[0] = 0xff;
    serialTrmBuffer[1] = 0xff;

    serialTrmBuffer[3] = 21;

    serialTrmBuffer[cntBytes++] = (uint8_t)sizeof(ssid);
    for (i = 0; i < sizeof(ssid); i++)
        serialTrmBuffer[cntBytes++] = ssid[i];

    serialTrmBuffer[cntBytes++] = (uint8_t)sizeof(password);
    for (i = 0; i < sizeof(password); i++)
        serialTrmBuffer[cntBytes++] = password[i];

    serialTrmBuffer[2] = cntBytes - 3;

    serialTrmBuffer[cntBytes] = CalcCheckSumm(serialTrmBuffer[2] + 1, &serialTrmBuffer[2]);
    TrmSerial();
}

void TrmSerial(void)
{
    int i;
    for (i = 0; i < (serialTrmBuffer[2] + 4); i++)
        Serial.write(serialTrmBuffer[i]);
}

void ProcessCMDFromPC(void)
{
    uint8_t IPbyte1, IPbyte2, IPbyte3, IPbyte4, i;

    switch (serialRecBuffer[3])
    {
    case SET_WiFi_NET:

        for (i = 0; i < serialRecBuffer[8]; i++)
            ssid[i] = serialRecBuffer[i + 9];
        ssid[i] = 0;

        for (i = 0; i < serialRecBuffer[9 + serialRecBuffer[8]]; i++)
            password[i] = serialRecBuffer[i + 10 + serialRecBuffer[8]];
        password[i] = 0;

        EEPROM.writeString(0, ssid);
        EEPROM.writeString(sizeof(ssid), password);

        EEPROM.commit();

        ReadEEPROM();

        break;
    default:
        break;
    }
}

void ReadEEPROM(void)
{
    int i;
    String str;
    str = EEPROM.readString(0);

    for (i = 0; i < sizeof(ssid); i++)
        ssid[i] = 0;
    for (i = 0; i < sizeof(password); i++)
        password[i] = 0;
    SendIP[0] = SendIP[1] = SendIP[2] = SendIP[3] = 0;

    for (i = 0; i < str.length(); i++)
        ssid[i] = str[i];
    ssid[i] = 0;

    str = EEPROM.readString(sizeof(ssid));

    for (i = 0; i < str.length(); i++)
        password[i] = str[i];
    password[i] = 0;
}
