import paho.mqtt.client as mqtt
import time
import os
from struct import *
import yaml

with open('config.yml', 'r') as file:
    config = yaml.safe_load(file)
DevId = config.get('DevId', '')
Img = config.get('Img','')

qos=0
data_block_size=10000
#data_block_size=250
#data_block_size=500 #for very slow connections (500 Bytes)
#data_block_size=5000 #for very slow connections (500 Bytes)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
  client.subscribe(status)
  client.subscribe(updState)
  #print("MQTT: on_connect: Connected with result code "+str(rc))


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
  global updState
  if msg.topic == status:
    print(str(msg.payload))   
  if msg.topic == updState:
    #print("<".join('{:02x}'.format(x) for x in msg.payload))
    client.mid_value=msg.payload[0]
    client.puback_flag=True  
    lowNibble = client.mid_value & 0x0F
    #print("<-- {:02x} {:02x}".format(msg.payload[0],lowNibble))
    if (lowNibble == 1):
      print("update started")
    else:
      if (lowNibble == 2):
        print("{}% finished".format(msg.payload[1]))

def wait_for(client,msgValue,period=0.25,wait_time=480,running_loop=False):
  client.running_loop=running_loop #if using external loop
  wcount=0  
  while True:
    #print("waiting"+ msgType)
    if client.puback_flag:
      if client.mid_value == msgValue:
        time.sleep(1) #wait additinal 0.5sec
        return True
      else:
        client.puback_flag = False
    time.sleep(period)
    wcount+=1
    if wcount>wait_time:
      print("return from wait loop taken too long")
      return False
  return True 

def send_header(filename,filesize):
   #header="header"+",,"+filename+",,"
   #header=bytearray(header,"utf-8")
   #header.extend(b','*(200-len(header)))
   #print(header)
   if filename.startswith("spiffs"):
    header=b'\x01'#send end-command#send start-command
   else:
    header=b'\x11'#send end-command#send start-command
   header+=filesize.to_bytes(4,'big')
   c_publish(client,updateTopic,header,qos)

def send_end(msgCnt):
   end=b'\x03' + msgCnt.to_bytes(4,'big')#send end-command
   c_publish(client,updateTopic,end,qos)

def c_publish(client,topic,out_message,qos):
  wcount=0
  while True:
    client.puback_flag=False
    res,mid=client.publish(topic,out_message,qos)#publish
    if res==0: #published ok
      if wait_for(client,out_message[0],running_loop=True):
        if client.mid_value == out_message[0]:
          return True
        else:
          print("wrong return-message")
    time.sleep(0.25)
    wcount+=1
    print("resending package {}".format(wcount))
    if wcount>10:
      #print("return from publish loop taken too long")
      raise SystemExit("no answer from device --> Update aborted")
      return False
  return True
def writeConfigFile(config):
  with open('config.yml', 'w') as file:
    yaml.dump(config, file)

def run_Update(filename):
  global DevId
  filesize = os.path.getsize(filename)
  print("sending file {0} size={1} to device {2}".format(filename,filesize,DevId))
  fo=open(filename,"rb")
  start=time.time()
  print("publishing ")
  send_header(filename,filesize)
  Run_flag=True
  outSize=0
  msgCnt=1
  while Run_flag:
    chunk=fo.read(data_block_size) # change if want smaller or larger data blcoks
    if chunk:
        outSize+=len(chunk)
        percent = (outSize/filesize*100)
        out_message = ((msgCnt & 0x0F) << 4) + 2
        out_message=out_message.to_bytes(1,'big') + int(percent).to_bytes(1,'big') + chunk
        c_publish(client,updateTopic,out_message,qos)
        msgCnt+=1

    else:
        msgCnt+=1
        send_end(msgCnt)
        print("messages {}".format(msgCnt))
        Run_flag=False

  time_taken=time.time()-start
  print("took ",time_taken)
  fo.close()
  time.sleep(20)

config['ServerIp'] = input("MQTT-Server IP (" + str(config['ServerIp']) + "):") or config['ServerIp']
config['ServerPort'] = int(input("MQTT-Server Port (" + str(config['ServerPort']) + "):") or config['ServerPort'])
print("0 ... cancel")
print("1 ... Wifi ON")
print("2 ... Wifi OFF")
print("3 ... Firmware Update")
print("4 ... get Wifi settings")
print("5 ... get CPU-Speed settings")
print("6 ... do internet update")
print("7 ... get firmware version")
print("8 ... set CPU-Speed")
iCmd = int(input("Befehl eingeben (0):") or 0)
Cmd = ""
match iCmd:
  case 1:
    Cmd = "#SYC WIFI=1\r\n"
  case 2:
    Cmd = "#SYC WIFI=0\r\n"
  case 3:
    print("Firmware update selected")
  case 4:
    Cmd = "#SYC Wifi?\r\n"    
  case 5:
    Cmd = "#SYC FCPU?\r\n"    
  case 6:
    Cmd = "#SYC DOUPDATE\r\n"    
  case 7:
    Cmd = "#SYC VER?\r\n"    
  case 8:
    CpuSpeed = input("CPU-Speed (" + str(config['CpuSpeed']) + "):") or config['CpuSpeed']
    config['CpuSpeed'] = CpuSpeed
    Cmd = "#SYC FCPU=" + str(CpuSpeed) + "\r\n"    
  case _:
    writeConfigFile(config)
    print("canceled")
    time.sleep(2)
    quit()
DevId = input("Device Id eingeben (" + DevId + "):") or DevId
config['DevId'] = DevId
if iCmd == 3:
  Firmware = input("Firmware (" + str(config['Firmware']) + "):") or config['Firmware']
  config['Firmware'] = Firmware
  Version = input("Version (" + str(config['Version']) + "):") or config['Version']
  config['Version'] = Version
topic = "GXAirCom/" + DevId + "/cmd"
status = "GXAirCom/" + DevId + "/state"
updState = "GXAirCom/" + DevId + "/upd/state"
sysCmd = "GXAirCom/" + DevId + "/cmd"
updateTopic = "GXAirCom/" + DevId + "/upd/cmd"
writeConfigFile(config)
client = mqtt.Client(client_id="gxAircomCmd")
client.on_connect = on_connect
client.on_message = on_message
client.puback_flag=False #use flag in publish ack
client.mid_value=None
client.connect(config['ServerIp'], config['ServerPort'], 60)
client.loop_start() # start thread for mqqt
if (iCmd == 1) or (iCmd == 2) or (iCmd == 4) or (iCmd == 5) or (iCmd == 6) or (iCmd == 7) or (iCmd == 8):
  client.publish(topic,Cmd) #send CMD
  time.sleep(5)
if (iCmd == 3):
  #filename = "Firmware_" + str(config['Version']) + "_" + str(config['Firmware']) + ".bin"
  filename = str(config['Firmware']) + "/" + str(config['Version']) + "/firmware.bin"
  print(filename)
  run_Update(filename)
client.disconnect() #disconnect
client.loop_stop() #stop loop

