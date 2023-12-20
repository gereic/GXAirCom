/*
 * ble.h
 *
 */


#ifndef MAIN_BLE_H_
#define MAIN_BLE_H_

#include <esp_coexist.h>
/*
#include <BLECharacteristic.h>
#include <BLEServer.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
*/

#include <NimBLEDevice.h>
#include <string.h>


struct ble_data {
  char data[MAXSIZEBLE];
  uint16_t size;
};

//#include <sstream>             // Part of C++ Standard library

extern struct statusData status;
extern void checkReceivedLine(const char *ch_str);
extern SemaphoreHandle_t RxBleQueue;

NimBLECharacteristic *pCharacteristic;
uint16_t maxMtu = 0xFFFF;


/*
const char *CHARACTERISTIC_UUID_DEVICENAME = "00002A00-0000-1000-8000-00805F9B34FB";
const char *CHARACTERISTIC_UUID_RXTX = "0000FFE1-0000-1000-8000-00805F9B34FB";
const char *CHARACTERISTIC_UUID_RXTX_DESCRIPTOR = "00002902-0000-1000-8000-00805F9B34FB";
const char *SERVICE_UUID = "0000FFE0-0000-1000-8000-00805F9B34FB";

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
*/

class MyServerCallbacks : public NimBLEServerCallbacks {

	static void updateMtu(NimBLEServer* pServer) {
		uint16_t newMtu = 0xFFFF;
		for (uint16_t conn_id : pServer->getPeerDevices()){
			// remove OP-Code (1 Byte) and Attribute Handle (2 Byte)
			uint16_t mtu = pServer->getPeerMTU(conn_id) - 3;
			newMtu = std::min<uint16_t>(newMtu, mtu);
		}
		if (newMtu != maxMtu && newMtu < 0xFFFF){
			maxMtu = newMtu;
			log_i("new mtu-size=%u",maxMtu);
		}
	}

	void onConnect(NimBLEServer* pServer) override {
  		//log_d("***************************** BLE CONNECTED *****************");
		updateMtu(pServer);
		status.bluetoothStat = 2; //we have a connected client
		NimBLEDevice::startAdvertising();
	};

	void onDisconnect(NimBLEServer* pServer) override {
		//log_d("***************************** BLE DISCONNECTED *****************");
		updateMtu(pServer);
		if (pServer->getConnectedCount() == 0) {
			status.bluetoothStat = 1; //client disconnected
		}
		NimBLEDevice::startAdvertising();
	}

	void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) override {
		NimBLEServer* server = NimBLEDevice::getServer();
		updateMtu(server);
	}
};



class MyCallbacks : public NimBLECharacteristicCallbacks {

	void onWrite(NimBLECharacteristic *pCharacteristic) {
		static String sLine = "";
		std::string rxValue = pCharacteristic->getValue();
		int valueLength = rxValue.length();
		//log_i("%s",rxValue.c_str());
		if (valueLength > 0) {				
			sLine += rxValue.c_str();
			if (sLine.endsWith("\n")){
				//Serial.printf("BLE_RX:%s\n",sLine.c_str());
				if (RxBleQueue) {
					if (sLine.length() < MAXSIZEBLE){
						ble_data data;
						std::copy_n(sLine.c_str(),sLine.length()+1,std::begin(data.data));
						data.size=sLine.length();
						BaseType_t status = xQueueSendToBack(RxBleQueue,&data,0);
						if (status != pdTRUE){
							log_w("ble_queue full -> reset queue");
							xQueueReset(RxBleQueue);
						}
					}
				}
				sLine = "";
			}
			if (sLine.length() > 512){
				sLine = "";
			}
		}
	}

};

void BLESendChunks(char *buffer,int iLen)
{
	if ((status.bluetoothStat == 2) && (iLen > 0) && (maxMtu < 0xFFFF)){ //we have a connection

		const uint8_t* begin = reinterpret_cast<uint8_t*>(buffer);
		const uint8_t* end = reinterpret_cast<uint8_t*>(buffer + iLen);

		while (std::distance(begin, end) > maxMtu) {
			//log_i("ble chunk : %s", std::string(begin, std::next(begin, maxMtu)).c_str());
			pCharacteristic->notify(begin, maxMtu, true);
			std::advance(begin, maxMtu);
			taskYIELD();
		}
		if(begin < end) {
			//log_i("ble chunk : %s", std::string(begin, end).c_str());
			pCharacteristic->notify(begin, std::distance(begin, end), true);
			taskYIELD();
		}
	}
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
  NimBLEDevice::init(bleId.c_str());
	NimBLEDevice::setMTU(256); //set MTU-Size to 256 Byte
	NimBLEServer* pServer = NimBLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());
	NimBLEService *pService = pServer->createService(NimBLEUUID((uint16_t)0xFFE0));
	// Create a BLE Characteristic
	pCharacteristic = pService->createCharacteristic(NimBLEUUID((uint16_t)0xFFE1),
		NIMBLE_PROPERTY::NOTIFY| NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
	);
	pCharacteristic->setCallbacks(new MyCallbacks());
	log_i("Starting BLE");
	// Start the service
	pService->start();

	// Start advertising
	NimBLEAdvertising* pAdvertising = pServer->getAdvertising();
	pAdvertising->addServiceUUID(pService->getUUID());
	pAdvertising->start();

	log_i("Waiting a client connection to notify...");
}

void stop_ble ()
{
	NimBLEDevice::deinit(true);
}

#endif
