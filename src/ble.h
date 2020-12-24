/*
 * ble.h
 *
 */


#ifndef MAIN_BLE_H_
#define MAIN_BLE_H_

#include <esp_coexist.h>
#include <BLECharacteristic.h>
#include <BLEServer.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//#include <sstream>             // Part of C++ Standard library

extern struct statusData status;
extern void checkReceivedLine(char *ch_str);

BLECharacteristic *pCharacteristic;


const char *CHARACTERISTIC_UUID_DEVICENAME = "00002A00-0000-1000-8000-00805F9B34FB";
const char *CHARACTERISTIC_UUID_RXTX = "0000FFE1-0000-1000-8000-00805F9B34FB";
const char *CHARACTERISTIC_UUID_RXTX_DESCRIPTOR = "00002902-0000-1000-8000-00805F9B34FB";
const char *SERVICE_UUID = "0000FFE0-0000-1000-8000-00805F9B34FB";

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks {

	void onConnect(BLEServer* pServer) {
  		//log_d("***************************** BLE CONNECTED *****************");
		status.bluetoothStat = 2; //we have a connected client
	};

	void onDisconnect(BLEServer* pServer) {
		//log_d("***************************** BLE DISCONNECTED *****************");
		status.bluetoothStat = 1; //client disconnected
		//delay(1000);
	//	pServer->
	}

};



class MyCallbacks : public BLECharacteristicCallbacks {

	void onWrite(BLECharacteristic *pCharacteristic) {

		std::string rxValue = pCharacteristic->getValue();

		if (rxValue.length() > 0) {
			int valueLength = strlen(&rxValue[0]);
			char cstr[valueLength+1]; //+1 for 0-termination
			memcpy(cstr, rxValue.data(), valueLength);
			cstr[valueLength] = 0; //zero-termination !!
			//log_i("received:%s,%d",cstr,valueLength);
			checkReceivedLine(&cstr[0]);
		}

	}

};

void BLESendChunks(String str)
{
	String substr;
	if (status.bluetoothStat == 2) {
		for (int k = 0; k < str.length(); k += _min(str.length(), 20)) {
			substr = str.substring(k, k + _min(str.length() - k, 20));
			pCharacteristic->setValue(substr.c_str());
			pCharacteristic->notify();
			vTaskDelay(5);
		}
	}
	vTaskDelay(20);
}

void NEMEA_Checksum(String *sentence)
{

	char chksum[3];
	const char *n = (*sentence).c_str() + 1; // Plus one, skip '$'
	uint8_t chk = 0;
	/* While current char isn't '*' or sentence ending (newline) */
	while ('*' != *n && '\n' != *n) {

		chk ^= (uint8_t)*n;
		n++;
	}

	//convert chk to hexadecimal characters and add to sentence
	sprintf(chksum, "%02X\n", chk);
	(*sentence).concat(chksum);


}


void start_ble (String bleId)
{
	esp_coex_preference_set(ESP_COEX_PREFER_BT);
    BLEDevice::init(bleId.c_str());
	BLEDevice::setMTU(256); //set MTU-Size to 256 Byte
	BLEServer *pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());

	/*
	BLEService *pService = pServer->createService(SERVICE_UUID);

	pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  	pCharacteristic->addDescriptor(new BLE2902());
 
  	BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    	                                     CHARACTERISTIC_UUID_RX,
        	                                 BLECharacteristic::PROPERTY_WRITE
            	                           );
 
  	pCharacteristic->setCallbacks(new MyCallbacks());
	pService->start();
	pServer->getAdvertising()->start();
	*/

	BLEService *pService = pServer->createService(BLEUUID((uint16_t)0xFFE0));
	// Create a BLE Characteristic
	pCharacteristic = pService->createCharacteristic(BLEUUID((uint16_t)0xFFE1),
		BLECharacteristic::PROPERTY_NOTIFY| BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
	);
	pCharacteristic->addDescriptor(new BLE2902());

	//BLECharacteristic *pCharacteristic = pService->createCharacteristic(
	//CHARACTERISTIC_UUID_DEVICENAME,
	//BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
	//);
	pCharacteristic->setCallbacks(new MyCallbacks());
	pCharacteristic->setValue("esp32ble-hm10");
	log_i("Starting BLE");
	// Start the service
	pService->start();
	pServer->getAdvertising()->addServiceUUID(BLEUUID((uint16_t)0xFFE0));
	// Start advertising
	pServer->getAdvertising()->start();

	log_i("Waiting a client connection to notify...");

}

#endif /* MAIN_AIRWHERE_BLE_H_ */
