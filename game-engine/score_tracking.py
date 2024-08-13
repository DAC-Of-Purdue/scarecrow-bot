import json
import paho.mqtt.client as mqtt
from datetime import datetime, timedelta


def on_connect(client, user_data, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("purdue-dac/carrot")


# The callback for when a PUBLISH message is received from the server.
def on_message(client, user_data, msg):
    uid = msg.payload.hex().upper()
    with open("rfid.json", "r") as fp:
        tags = json.load(fp)
    tags[uid] = 5
    with open("rfid.json", "w") as fp:
        json.dump(tags, fp)
    print(uid)
    global last_time  # Bad practice. Only use in emergency
    if last_time is None:
        last_time = datetime.now()
    if (datetime.now() - last_time) < timedelta(seconds=4, microseconds=500):
        client.publish("purdue-dac/sound", "1")
    elif (datetime.now() - last_time) > timedelta(seconds=7):
        last_time = None
    else:
        client.publish("purdue-dac/sound", "2")
        last_time = None


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
last_time = None

client.connect("localhost", 1883, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()
