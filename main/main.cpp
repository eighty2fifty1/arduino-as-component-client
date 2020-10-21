
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

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//                REPEATER/CLIENT                   //
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

//#define NUMBER_OF_DEVICES 3

using namespace std;

std::map<string, BaseType_t> PinnedTasks;
std::map<string, BLEAdvertisedDevice> FoundDevices; //holds address and device
typedef queue<tuple<uint16_t, uint8_t, uint8_t>> my_tuple;
my_tuple dataList;
static BLEUUID tempServiceUUID("181A");
static BLEUUID tempCharUUID("2A20");
//static boolean doConnect = false;
static boolean connected = false;
//static boolean doScan = false;
static BLERemoteCharacteristic *pTempCharacteristic;
queue<string> disconnectedDevices;

//static BLEAdvertisedDevice *myDevice;
std::vector<BLEAdvertisedDevice *> devices;
int counter = 0;
bool looping = true;
int pri = 10;

uint8_t _pData[4];

BLEClient *pClient;

//prototypes
void recvWithStartEndMarkers();
static void coreBLEClient(BLEClient *);

String dataString;
string nameString = "iPhone Stand-in";

HardwareSerial MySerial(2);
const ::byte numChars = 16;
char receivedChars[numChars];
char tempChars[numChars];
bool newData = false;

//status message
int status[9];

class MyClientCallback : public BLEClientCallbacks
{
    void onConnect(BLEClient *pClient)
    {
        Serial.print("onConnect to device: ");
        Serial.println(pClient->getPeerAddress().toString().c_str());
    }

    void onDisconnect(BLEClient *pClient)
    {
        try
        {
            connected = false;
            Serial.print("onDisconnect to: ");

            Serial.println(pClient->getPeerAddress().toString().c_str());
            //disconnectedDevices.push(pclient->getPeerAddress().toString().c_str());
            //PinnedTasks.erase(pclient->getPeerAddress().toString().c_str());
            //delete pClient;     //should call destructor
            pClient->unregist();
            log_e("ondisconnect complete");
            delay(1000);
            //vTaskDelete(NULL);    //should delete this task
            //PinnedTasks.push_back(xTaskCreatePinnedToCore([](void *p) { coreBLEClient((BLEClient *)p); }, "coreClient", 4096, pclient, pri++, NULL, tskNO_AFFINITY));
        }
        catch (exception &e)
        {
            Serial.println(e.what());
        }
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

static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                           uint8_t *pData,
                           size_t length,
                           bool isNotify)
{
    
    Serial.printf("notifyCallback: %s %s handle: %02x value:", pBLERemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress().toString().c_str(), pBLERemoteCharacteristic->getUUID().toString().c_str(), pBLERemoteCharacteristic->getHandle());
  for (int i=0; i<length; i++)
    Serial.printf(" %02x", pData[i]);
  Serial.println();
    /*
    uint16_t tempData;
    _pData[1] = *pData;   //temp byte  (bit)?
    _pData[0] = *++pData; //2nd temp byte
    _pData[2] = *++pData; //position byte
    _pData[3] = *++pData; //batt pct byte

    Serial.println(_pData[2]);
    tempData = (_pData[0] << 8) | _pData[1];
    dataList.push(tuple<uint16_t, uint8_t, uint8_t>(tempData, _pData[2], _pData[3]));
    */
}

static void coreBLEClient(BLEClient *pClient)
{
    for(;;) {  
    if(pClient->isDisconnected() && pClient->connect()) {
      auto* pRemoteServiceMap = pClient->getServices();
      for (auto itr : *pRemoteServiceMap)  {
        auto *pCharacteristicMap = itr.second->getCharacteristicsByHandle();
        for (auto itr : *pCharacteristicMap)
          if(itr.second->canNotify())
            itr.second->registerForNotify(notifyCallback);
      }
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
    /*
    for (;;)
    {
        
        try
        {
            if (pClient->isDisconnected() && pClient->connect())
            {
                Serial.print("inside coreBLEClient of device: ");
                Serial.println(pClient->getPeerAddress().toString().c_str());
                BLERemoteService *pTempService = pClient->getService(tempServiceUUID);
                if (pTempService == nullptr)
                {
                    Serial.print("Failed to find our temp service UUID: ");
                    Serial.println(tempServiceUUID.toString().c_str());
                    pClient->disconnect();
                }
                Serial.println(" - Found our temp service");
                pTempCharacteristic = pTempService->getCharacteristic(tempCharUUID);
                if (pTempCharacteristic == nullptr)
                {
                    Serial.print("Failed to find our characteristic UUID: ");
                    Serial.println(tempCharUUID.toString().c_str());
                    pClient->disconnect();
                }
                Serial.println(" - Found our characteristic");

                if (pTempCharacteristic->canNotify())
                {
                    Serial.println("registering for notifications");
                    pTempCharacteristic->registerForNotify(notifyCallback);
                }
            }

            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        catch (exception &e)
        {
            Serial.println(e.what());
        }
        
    }
    */
}

void connectSensor(BLEScanResults *sr)
{
    for (int i = 0; i < sr->getCount(); i++)
    {
        try
        {
            Serial.print("Attempting to connect to client ");

            Serial.println(i);
            auto advertisedDevice = sr->getDevice(i);
            if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(tempServiceUUID))
            {
                FoundDevices.insert(pair<string, BLEAdvertisedDevice>(advertisedDevice.getAddress().toString().c_str(), advertisedDevice));
                pClient = BLEDevice::createClient(&advertisedDevice);
                Serial.println(pClient->getPeerAddress().toString().c_str());
                pClient->setClientCallbacks(new MyClientCallback());
                ::string taskHandle = pClient->getPeerAddress().toString().c_str();
                if (pClient->regist())
                {
                    PinnedTasks.insert(pair<string, BaseType_t>(taskHandle, xTaskCreatePinnedToCore([](void *p) { coreBLEClient((BLEClient *)p); }, "coreClient", 4096, pClient, pri++, NULL, tskNO_AFFINITY)));
                }
                Serial.print("connected to index ");
                Serial.println(i);
                delay(1000);
            }
        }

        catch (exception &e)
        {
            Serial.print("exception caught: ");
            Serial.println(e.what());
        }
    }
}

void reconnectSensor(BLEClient *pC)
{
    pC->setClientCallbacks(new MyClientCallback());
    ::string taskHandle = pC->getPeerAddress().toString().c_str();
    if (pC->regist())
    {
        PinnedTasks.insert(pair<string, BaseType_t>(taskHandle, xTaskCreatePinnedToCore([](void *p) { coreBLEClient((BLEClient *)p); }, "coreClient", 4096, pC, pri++, NULL, tskNO_AFFINITY)));
    }
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
        //parseData();
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

    for (int i = 1; i < 10; i++)
    {
        strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
        status[i] = atoi(strtokIndx);   // convert this part to an integer
    }
}

void showParsedData()
{
    for (int t = 0; t < 9; t++)
    {
        Serial.print(status[t]);
        Serial.print(", ");
    }
    Serial.println();
}



extern "C"
{
    void app_main(void);
}

void app_main(void)
{
    Serial.begin(115200);
    Serial.println("Starting Arduino BLE Client application...");
    BLEDevice::init("");

    BLEScan *pBLEScan = BLEDevice::getScan();

    pBLEScan->setActiveScan(true);
    auto pScanResults = pBLEScan->start(5);
    //pBLEScan->stop();
    Serial.println("Connectable Devices");

    connectSensor(&pScanResults);

    //PinnedTasks.insert(pair<string, BaseType_t>("stats", xTaskCreatePinnedToCore([](void *p) {processorStatistics();}, "stats", 4096, NULL, 5, NULL, tskNO_AFFINITY)));

    //BLEDevice::getScan()->stop();

    MySerial.begin(9600, SERIAL_8N1);

    for (;;)
    {
        /*
        if (MySerial.available() > 0)
        {
            serialRecv();
            //serialRelay();
        }
*/
        /*
       while(!disconnectedClients.empty()){
           Serial.println("attempting reconnect");
           reconnectSensor(disconnectedClients.front());
           disconnectedClients.pop();
       }
       */
        /*
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
            //Serial.println(dataString);
            //Serial.println(PinnedTasks.size());
            dataList.pop();
            delay(10);
            
        }
        */
        delay(50);
        if (PinnedTasks.empty()){
            ESP.restart();
        }
    }
}
