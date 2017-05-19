import paho.mqtt.client as paho
 
def on_connect(client, userdata, flags, rc):
    print("connected")
    client.subscribe('chat/2', qos=0)
    client.publish('chat/1', 'Hello Chat, Number 2 is here!', qos=0)    
    
def on_subscribe(client, userdata, mid, granted_qos):
    print("Subscribed: "+str(mid)+" "+str(granted_qos))
    
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.qos)+" "+str(msg.payload))       
    
def on_publish(client, userdata, mid):
    print("mid: "+str(mid))
    
def on_disconnect(client, userdata, rc):
    print("disconnected")
 
client = paho.Client()
client.on_connect = on_connect
client.on_subscribe = on_subscribe
client.on_message = on_message
client.on_disconnect = on_disconnect
client.connect("tensor2.ddns.net", 1883)
client.loop_start()

while 1:
	a = 0