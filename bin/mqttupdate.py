import paho.mqtt.client as mqtt
import time
import os
from struct import *

#filename="firmware_v5.3.5_nopsRam.bin"
#filename="firmware_v5.3.5_GsNoBluetoothSim7000.bin"
#filename="firmware_v5.3.5_sim7000_psRam.bin"
#filename="firmware_v5.3.5_psRam.bin"
#filename="spiffs_v5.4.0.bin" #file to send

#DevId="08B668" #PGV-NW
#filename="firmware_v5.4.0_GsSim7000.bin"
#DevId="08C494" #GX-TEST
#DevId="08A7E4" #GetroniX
#DevId="08F968" #FMG Abetzberg
DevId="08A0A0" #Goldeck
#DevId="089144" #Prochenberg
filename="wireless-StickV3_Sim7080/v8.2.1/firmware.bin"

SERVERIP="192.168.0.1"
PORT=1883
#SERVERIP="remote.getronix.at"
#PORT=21883

qos=0
#data_block_size=10000
data_block_size=500 #for very slow connections (500 Bytes)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
   #client.subscribe(TOPICGXAIRCOM, 2)
   print("MQTT: on_connect: Connected with result code "+str(rc))
   client.subscribe(status)
   client.subscribe(updState)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
   #global msgReceived
   global updState
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
      #print("Byte0={:02x}".format(client.mid_value))
   #else:
   #   print("<--" + str(msg.payload.decode("utf-8")))
   #msgReceived = 1

#def on_publish(client, userdata, mid):
#    #print("pub ack "+ str(mid))
#    #client.mid_value=mid
#    #client.puback_flag=True  

def wait_for(client,msgValue,period=1,wait_time=120,running_loop=False):
  client.running_loop=running_loop #if using external loop
  wcount=0  
  while True:
    #print("waiting"+ msgType)
    if client.puback_flag:
      if client.mid_value == msgValue:
        return True
      else:
        client.puback_flag = False
    #if msgType=="PUBACK":
    #    if client.on_publish:        
    #        if client.puback_flag:
    #            return True
    #if not client.running_loop:
    #    client.loop(.01)  #check for messages manually
    time.sleep(period)
    #print("loop flag ",client.running_loop)
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
   c_publish(client,topic,header,qos)

def send_end(msgCnt):
   end=b'\x03' + msgCnt.to_bytes(4,'big')#send end-command
   c_publish(client,topic,end,qos)

def c_publish(client,topic,out_message,qos):
  #global sysCmd
  wcount=0
  while True:
    client.puback_flag=False
    #if (wcount == 5):
    #  client.publish(sysCmd,"#SYC VER?\r",0)#publish
    res,mid=client.publish(topic,out_message,qos)#publish
    if res==0: #published ok
      if wait_for(client,out_message[0],running_loop=True):
        if client.mid_value == out_message[0]:
          return True
        else:
          print("wrong return-message")
          #return False
      #else:
      #  raise SystemExit("not got puback so quitting")
    time.sleep(0.25)
    wcount+=1
    print("resending package {}".format(wcount))
    if wcount>10:
      #print("return from publish loop taken too long")
      raise SystemExit("no answer from device --> Update aborted")
      return False
  return True

topic = "GXAirCom/" + DevId + "/upd/cmd"
status = "GXAirCom/" + DevId + "/state"
updState = "GXAirCom/" + DevId + "/upd/state"
sysCmd = "GXAirCom/" + DevId + "/cmd"
filesize = os.path.getsize(filename)
print("sending file {0} size={1} to device {2}".format(filename,filesize,DevId))
fo=open(filename,"rb")
#create mqtt client
client = mqtt.Client(client_id="gxAircomUpdater")
client.on_connect = on_connect
client.on_message = on_message
#client.on_publish=on_publish
client.puback_flag=False #use flag in publish ack
client.mid_value=None
client.connect(SERVERIP, PORT, 60)
client.loop_start() # start thread for mqqt
time.sleep(2)
start=time.time()
print("publishing ")
send_header(filename,filesize)
Run_flag=True
outSize=0
msgCnt=1
#msgReceived = 0
while Run_flag:
   chunk=fo.read(data_block_size) # change if want smaller or larger data blcoks
   if chunk:
      #out_hash_md5.update(chunk)
      #out_message = bytearray(((msgCnt & 0x0F) << 4) + 2)
      outSize+=len(chunk)
      percent = (outSize/filesize*100)
      out_message = ((msgCnt & 0x0F) << 4) + 2
      out_message=out_message.to_bytes(1,'big') + int(percent).to_bytes(1,'big') + chunk
      #print(" length =",type(out_message))
      c_publish(client,topic,out_message,qos)
      msgCnt+=1
      #for x in range(1000):
      #   time.sleep(0.01)
      #   if msgReceived == 1:
      #      break    # break here
      #msgReceived = 0
      #print("-->%.1f%% finished" % percent)
   else:
      #send hash
      #out_message=out_hash_md5.hexdigest()
      msgCnt+=1
      send_end(msgCnt)
      print("messages {}".format(msgCnt))
      #print("out Message ",out_message)
      #res,mid=client.publish("data/files",out_message,qos=1)#publish
      Run_flag=False

#while(True):
#	#endless loop
#	time.sleep(1)
time_taken=time.time()-start
print("took ",time_taken)
#time.sleep(4)
fo.close()
time.sleep(20)
client.disconnect() #disconnect
client.loop_stop() #stop loop
