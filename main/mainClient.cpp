#include "Arduino.h"
#include "BLEDevice.h"
#include "FreeRTOS.h"
#include "sdkconfig.h"
#include "BLEUUID.h"
#include <vector>
#include <queue>
#include <tuple>
#include <map>
#include <exception>
#include <string>
#include <array>

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//                REPEATER/CLIENT                   //
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

#define INCOMING_STATUS_MSG_LEN 5
#define OUTGOING_STATUS_MSG_LEN 8       //length of status message, sent every _ seconds
#define STATUS_MSG_INTERVAL 10 //status msg interval in seconds
#define LED 2

using namespace std;

std::vector<BaseType_t> PinnedTasks;
typedef queue<tuple<uint16_t, uint8_t, uint8_t>> my_tuple;
my_tuple dataList;
static BLEUUID tempServiceUUID("181A");
static BLEUUID tempCharUUID("2A20");
static BLERemoteCharacteristic *pTempCharacteristic;

int pri = 10;
uint8_t _pData[4];

BLEClient *pClient;

//prototypes
void serialRecv();
void recvWithStartEndMarkers();
static void coreBLEClient(BLEClient *);
void scan();
void scanTask();
void showParsedData();
void resetDevice();
void sendSensorReport();
void forceScan();
void sensorSleep(int);
void parseData();
void sendStatusMessage();
void setNumSensors(int);

String dataString;
//array<char, STATUS_MSG_LEN> statusString;
array<std::string, 6> macAddresses; //holds mac addresses relative to sensor position
array<TimerHandle_t, 6> sensorTimers;
HardwareSerial MySerial(2);
const ::byte numChars = 16;
char receivedChars[numChars];
char tempChars[numChars];
bool newData = false;

//status message.  all are bool except number of sensors and sensor sleep
array<int, INCOMING_STATUS_MSG_LEN> status; //<software reset, set number of sensors, send mac and rssi, force scan, sleep>
array<int, OUTGOING_STATUS_MSG_LEN> sensorStatus = {9};
int sensorsExpected = 4; //default for 2 axle.  should be stored in memory
int sensorsConnected = 0;

class MyClientCallback : public BLEClientCallbacks
{
    void onConnect(BLEClient *pClient)
    {
        Serial.print("onConnect to device: ");
        Serial.print(pClient->getPeerAddress().toString().c_str());
        Serial.print(" ");
        sensorsConnected++;
    }

    void onDisconnect(BLEClient *pClient)
    {
        Serial.print("onDisconnect to: ");
        Serial.println(pClient->getPeerAddress().toString().c_str());
        sensorsConnected--;
        //disconnectedDevices.push(pClient);
        //pClient->disconnect();
        //delete pClient;
    }
};

static void processorStatistics()
{
    for (;;)
    {
        delay(10000);
        char ptrTaskList[250];
        vTaskGetRunTimeStats(ptrTaskList);
        Serial.println(ptrTaskList);
    }
}

static void statusCheck()
{
    for (;;)
    {
        delay(STATUS_MSG_INTERVAL * 1000);
        sendStatusMessage();
    }
}

static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                           uint8_t *pData,
                           size_t length,
                           bool isNotify)
{
    //parses incoming data for temp, position, and battery percent
    uint16_t tempData;
    _pData[1] = *pData;   //temp byte  (bit)?
    _pData[0] = *++pData; //2nd temp byte
    _pData[2] = *++pData; //position byte
    _pData[3] = *++pData; //batt pct byte
    tempData = (_pData[0] << 8) | _pData[1];
    if (_pData[2] > 6)
    {
        sensorSleep(_pData[2]);
    }
    else
    {
        dataList.push(tuple<uint16_t, uint8_t, uint8_t>(tempData, _pData[2], _pData[3]));

        //starts or restarts timer in proper position in array of timers
        xTimerStart(sensorTimers[_pData[2] - 1], NULL); //timerId = sensor position index

        //populates list of mac addresses for later use
        if (pBLERemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress().toString() !=
            macAddresses[_pData[2] - 1])
        {
            macAddresses[_pData[2] - 1] = pBLERemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress().toString();
            Serial.print("mac address stored for device ");
            Serial.print(_pData[2]);
            Serial.print(" ");
            Serial.println(macAddresses[_pData[2] - 1].c_str());
        }

        //sets status int in status message
        //0 = not connected (default)
        //1 = normal operation
        //2 = sensor timed out for unknown reason
        //3 = sensor sleeping due to inactivity or commanded
        sensorStatus[_pData[2]] = 1;
    }
}

static void coreBLEClient(BLEClient *pClient)
{
    for (;;)
    {
        if (pClient->isDisconnected() && pClient->connect())
        {
            auto *pRemoteServiceMap = pClient->getServices();
            for (auto itr : *pRemoteServiceMap)
            {
                auto *pCharacteristicMap = itr.second->getCharacteristicsByHandle();
                for (auto itr : *pCharacteristicMap)
                    if (itr.second->canNotify())
                        itr.second->registerForNotify(notifyCallback);
            }
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void vTimerCallback(TimerHandle_t xTimer)
{
    //timerID is equal to sensor position index (1-6)
    //index of the array of of timers is equal to timerId - 1
    Serial.print("time out: ");

    //sets status message in appropriate element to 2 for time out
    Serial.println((int)pvTimerGetTimerID(xTimer));

    sensorStatus[(int)pvTimerGetTimerID(xTimer)] = 2;
}

void connectSensor(BLEScanResults *sr)
{
    for (int i = 0; i < sr->getCount(); i++)
    {
        auto advertisedDevice = sr->getDevice(i);
        if (advertisedDevice.isAdvertisingService(tempServiceUUID))
        {
            pClient = BLEDevice::createClient(&advertisedDevice);
            Serial.println(pClient->getPeerAddress().toString().c_str());
            pClient->setClientCallbacks(new MyClientCallback());

            if (pClient->regist())
            {
                PinnedTasks.push_back(xTaskCreatePinnedToCore([](void *p) { coreBLEClient((BLEClient *)p); }, pClient->getPeerAddress().toString().c_str(), 4096, pClient, pri++, NULL, tskNO_AFFINITY));
            }
            delay(1000);
        }
    }
}

extern "C"
{
    void app_main(void);
}

void scan()
{
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setActiveScan(true);
    BLEScanResults pScanResults = pBLEScan->start(5);
    //pBLEScan->stop();
    Serial.println("Connectable Devices");

    //print list of found devices for debugging
    for (int i = 0; i < pScanResults.getCount(); i++)
    {
        Serial.println(pScanResults.getDevice(i).getAddress().toString().c_str());
    }
    connectSensor(&pScanResults);
}

void app_main(void)
{
    for (int i = 0; i < sensorTimers.size(); i++)
    {
        sensorTimers[i] = xTimerCreate("timer", 1000, pdTRUE, (void *)i + 1, vTimerCallback);
    }

    pinMode(LED, OUTPUT);
    Serial.begin(115200);
    MySerial.begin(9600, SERIAL_8N1);

    Serial.println("Starting Arduino BLE Client application...");
    BLEDevice::init("");

    //PinnedTasks.push_back(xTaskCreatePinnedToCore([](void *p) { processorStatistics(); }, "stats", 4096, NULL, 5, NULL, tskNO_AFFINITY));
    PinnedTasks.push_back(xTaskCreatePinnedToCore([](void *p) { statusCheck(); }, "stats", 4096, NULL, 5, NULL, tskNO_AFFINITY));

    scan();

    for (;;)
    {
        if (MySerial.available() > 0)
        {
            serialRecv();
        }
        while (!dataList.empty())
        {
            dataString = "<";
            dataString += get<1>(dataList.front());
            dataString += "i";
            dataString += get<0>(dataList.front());
            dataString += "i";
            dataString += get<2>(dataList.front());
            dataString += ">";
            MySerial.println(dataString);
            Serial.println(dataString);
            //Serial.println(PinnedTasks.size());
            dataList.pop();
            delay(10);
        }

        delay(10);

        //update number of sensors connected
        sensorStatus[7] = sensorsConnected;
        if (PinnedTasks.empty())
        {
            ESP.restart();
        }
    }
}

//////////////////////////////////////////////////////////
//                                                      //
//                  Serial Functions                    //
//    These functions are for handling incoming serial  //
//    messages to control various system functions      //
//////////////////////////////////////////////////////////
void sendStatusMessage()
{
    string statusString = "<";
    for (int i = 0; i < sensorStatus.size(); i++)
    {
        statusString += to_string(sensorStatus[i]);

        statusString += "i";
    }
    statusString += ">";

    Serial.println(statusString.c_str());
    MySerial.println(statusString.c_str());
}

void serialRecv()
{
    Serial.println("receive function called");
    recvWithStartEndMarkers();
    if (newData == true)
    {
        strcpy(tempChars, receivedChars);
        // this temporary copy is necessary to protect the original data
        //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        //showParsedData();
        newData = false;
    }
}

void recvWithStartEndMarkers()
{
    static boolean recvInProgress = false;
    static ::byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    //Serial.println("other recv fucntion called");
    while (MySerial.available() > 0 && newData == false)
    {
        //Serial.println("data found");
        rc = MySerial.read();
        //Serial.print(rc);

        if (recvInProgress == true)
        {
            if (rc != endMarker)
            {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars)
                {
                    ndx = numChars - 1;
                }
            }
            else
            {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker)
        {
            recvInProgress = true;
        }
    }
}

void parseData()
{ // split the data into its parts

    char *strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars, ","); // get the first part - the string
    status[0] = atoi(strtokIndx);        // copy it to messageFromPC

    for (int i = 1; i < status.size(); i++)
    {
        strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
        status[i] = atoi(strtokIndx);   // convert this part to an integer
    }

    showParsedData(); //for debugging

    if (status[0] > 0)
    {
        resetDevice();
    }

    else
    {
        if (status[1] > 0)
        {
            Serial.println("num sensors sent");
            if (sensorsExpected != status[1])
            {
                sensorsExpected = status[1];
                Serial.print("new number of sensors: ");
                Serial.println(sensorsExpected);

                //still need to save in memory
                setNumSensors(sensorsExpected);
            }
        }

        if (status[2] > 0)
        {
            sendSensorReport();
        }

        if (status[3] > 0)
        {
            forceScan();
        }

        //sleeps the sensor from posit index sent
        if (status[4] > 0)
        {
            sensorSleep(status[4] + 6); //needs to have the +6 so function can be used both to and from sensor
        }
    }
}

void showParsedData()
{
    for (int t = 0; t < status.size(); t++)
    {
        Serial.print(status[t]);
        Serial.print(", ");
    }
    Serial.println();
}

void resetDevice()
{
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    ESP.restart();
}

void forceScan()
{
}

void forceReconnect(string addr)
{
}

//saves expected number of sensors in memory
void setNumSensors(int s)
{
    //something with nvm goes here
}

//MAC address, RSSI
void sendSensorReport()
{
    Serial.println("sensor report requested");
}

//acknowledges the reset message and resets value in server
//may not be necessary?
void sendResetConfirm()
{
}

void sensorSleep(int i)
{
    int posit = i - 6;
    //set status message to sleep
    sensorStatus[posit] = 3;

    //stop timer
    xTimerStop(sensorTimers[posit - 1], NULL);

    Serial.print("sleeping sensor ");
    Serial.println(posit);

    //send sleep command to sensor

}
