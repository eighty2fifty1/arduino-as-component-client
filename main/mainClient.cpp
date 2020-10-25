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

#define STATUS_MSG_LEN 9       //length of status message, sent every _ seconds
#define STATUS_MSG_INTERVAL 10 //status msg interval in seconds

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
void sensorSleep();
void parseData();
bool sensorTimeout(int); //receives sensor index and returns true if notification not received in 10 seconds

String dataString;
array<char, STATUS_MSG_LEN> statusString;
array<std::string, 6> macAddresses; //holds mac addresses relative to sensor position
HardwareSerial MySerial(2);
const ::byte numChars = 16;
char receivedChars[numChars];
char tempChars[numChars];
bool newData = false;

//status message.  all are bool except number of sensors
array<int, 9> status; //<reset, set number of sensors, send mac and rssi, force scan, sleep>

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

        if (sensorsConnected != sensorsExpected)
        {
        }
    }
}

static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                           uint8_t *pData,
                           size_t length,
                           bool isNotify)
{
    uint16_t tempData;
    _pData[1] = *pData;   //temp byte  (bit)?
    _pData[0] = *++pData; //2nd temp byte
    _pData[2] = *++pData; //position byte
    _pData[3] = *++pData; //batt pct byte
    tempData = (_pData[0] << 8) | _pData[1];
    dataList.push(tuple<uint16_t, uint8_t, uint8_t>(tempData, _pData[2], _pData[3]));

    if (pBLERemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress().toString() !=
        macAddresses[_pData[2] - 1])
    {
        macAddresses[_pData[2] - 1] = pBLERemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress().toString();
        Serial.print("mac address stored for device ");
        Serial.print(_pData[2]);
        Serial.print(" ");
        Serial.println(macAddresses[_pData[2] - 1].c_str());
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
    Serial.begin(115200);
    MySerial.begin(9600, SERIAL_8N1);

    Serial.println("Starting Arduino BLE Client application...");
    BLEDevice::init("");

    //PinnedTasks.push_back(xTaskCreatePinnedToCore([](void *p) { processorStatistics(); }, "stats", 4096, NULL, 5, NULL, tskNO_AFFINITY));
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

    if (status[0])
    {
        resetDevice();
    }

    else
    {
        if (status[1] > 0)
        {
            if (sensorsExpected != status[1])
            {
                sensorsExpected = status[1];
            }
        }

        if (status[2])
        {
            sendSensorReport();
        }

        if (status[3])
        {
            forceScan();
        }

        if (status[4])
        {
            sensorSleep();
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
    ESP.restart();
}

void forceScan()
{
}

void forceReconnect(string addr)
{
}

//saves expected number of sensors.  needs to save it in memory
void setNumSensors(int s)
{
    sensorsExpected = s;
}

//MAC address, RSSI
void sendSensorReport()
{
}

//acknowledges the reset message and resets value in server
//may not be necessary?
void sendResetConfirm()
{
}

void sensorSleep()
{
}
