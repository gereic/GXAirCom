#include <WebHelper.h>

// Globals
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(1337);
char msg_buf[255];
int led_state = 0;
#define MAXCLIENTS 10
uint8_t clientPages[MAXCLIENTS];

/***********************************************************
 * Functions
 */

// Callback: receiving any WebSocket message
void onWebSocketEvent(uint8_t client_num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {

  StaticJsonDocument<300> doc;                      //Memory pool
  JsonObject root = doc.to<JsonObject>();
  DeserializationError error;
  uint8_t value = 0;
  // Figure out the type of WebSocket event
  switch(type) {

    // Client has disconnected
    case WStype_DISCONNECTED:
      if (client_num < MAXCLIENTS) clientPages[client_num] = 0;
      Serial.printf("[%u] Disconnected!\n", client_num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(client_num);
        Serial.printf("[%u] Connection from ", client_num);
        Serial.println(ip.toString());        
      }
      break;

    // Handle text messages from client
    case WStype_TEXT:

      // Print out raw message
      
      Serial.printf("[%u] Received text: %s\n", client_num, payload);      
      error = deserializeJson(doc, payload);
      if (error) {   //Check for errors in parsing
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return;
    
      }
      if (root.containsKey("page")){
        value = doc["page"];                    //Get value of sensor measurement
        if (client_num < MAXCLIENTS) clientPages[client_num] = value;
        Serial.print("page=");
        Serial.println(value);
        doc.clear();
        if (clientPages[client_num] == 1){
          doc["headline"] = MAIN_IDENT MAIN_FIRMWARE_VERSION;
          doc["myDevId"] = setting.myDevId;
          doc["ssid"] = setting.ssid;
          doc["password"] = setting.password;
          doc["PilotName"] = setting.PilotName;
          doc["UDPServerIP"] = setting.UDPServerIP;
          doc["UDPSendPort"] = setting.UDPSendPort;
          serializeJson(doc, msg_buf);
          Serial.printf("Sending to [%u]: %s\n", client_num, msg_buf);
          webSocket.sendTXT(client_num, msg_buf);
        }
      }
      break;

    // For everything else: do nothing
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

// Callback: send 404 if requested file does not exist
void onPageNotFound(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(404, "text/plain", "Not found");
}

void Web_setup(void){
  for (int i = 0;i < MAXCLIENTS;i++) clientPages[i] = 0;
  // On HTTP request for root, provide index.html file
  server.on("/settings.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html");
  });
  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/html");
  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, request->url(), "text/css");
  });

  // On HTTP request for style sheet, provide style.css
  //server.on("/style.css", HTTP_GET, onCSSRequest);

  // Handle requests for pages that do not exist
  server.onNotFound(onPageNotFound);

  // Start web server
  server.begin();

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void Web_loop(void){
  static uint32_t tLife = millis();
  static uint16_t counter = 0;
  uint32_t tAct = millis();
  // Look for and handle WebSocket data
  webSocket.loop();
  if ((tAct - tLife) >= 500){
    tLife = tAct;
    StaticJsonDocument<200> doc;                      //Memory pool
    doc.clear();
    doc["counter"] = counter;
    serializeJson(doc, msg_buf);
    for (int i = 0;i <MAXCLIENTS;i++){
      if (clientPages[i] == 1){
        //Serial.printf("Sending to [%u]: %s\n", i, msg_buf);
        webSocket.sendTXT(i, msg_buf);
      }
    }
    counter++;
  }
}