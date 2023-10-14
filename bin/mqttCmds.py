import paho.mqtt.client as mqtt
import time
import os
from struct import *
import yaml

with open('config.yml', 'r') as file:
    config = yaml.safe_load(file)
DevId = config.get('DevId', '')
#DevId="08C494" #FMG Abetzberg
SERVERIP="192.168.0.10"
PORT=1883
#SERVERIP="remote.getronix.at"
#PORT=21883

qos=0
data_block_size=10000

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
  #client.subscribe(TOPICGXAIRCOM, 2)
  client.subscribe(status)
  #print("MQTT: on_connect: Connected with result code "+str(rc))


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
  if msg.topic == status:
    print(str(msg.payload))

print("0 ... cancel")
print("1 ... Wifi ON")
print("2 ... Wifi OFF")
iCmd = int(input("Befehl eingeben (0):") or 0)
Cmd = ""
match iCmd:
  case 1:
    Cmd = "#SYC WIFI=1\r\n"
  case 2:
    Cmd = "#SYC WIFI=0\r\n"
  case _:
    print("canceled")
    time.sleep(2)
    quit()
DevId = input("Device Id eingeben (" + DevId + "):") or DevId
config['DevId'] = DevId
topic = "GXAirCom/" + DevId + "/cmd"
status = "GXAirCom/" + DevId + "/state"
print(topic + "=" + Cmd)
client = mqtt.Client(client_id="gxAircomCmd")
client.on_connect = on_connect
client.on_message = on_message
client.connect(SERVERIP, PORT, 60)
client.loop_start() # start thread for mqqt
time.sleep(2)
client.publish(topic,Cmd) #set wifi to on
time.sleep(2)
client.disconnect() #disconnect
client.loop_stop() #stop loop
with open('config.yml', 'w') as file:
    yaml.dump(config, file)