import json
import time
import paho.mqtt.client as mqtt
import requests
from datetime import datetime, timedelta


def on_connect(client, user_data, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("purdue-dac/carrot")
    client.subscribe("purdue-dac/cmd")


# The callback for when a PUBLISH message is received from the server.
def on_message(client, user_data, msg):
    match msg.topic:
        case "purdue-dac/carrot":
            uid = msg.payload.hex().upper()
            with open("rfid.json", "r") as fp:
                tags = json.load(fp)
            carrot_id = tags[uid]
            global carrot_bytes  # Bad practice. Only use in emergency. We could use user_data
            bite = carrot_bytes[carrot_id - 1]
            if bite > 0:
                bite -= 1
                if bite == 0:
                    client.publish("purdue-dac/sound", "2")
                else:
                    client.publish("purdue-dac/sound", "1")

            carrot_bytes[carrot_id - 1] = bite

            map = ["1" if bite == 0 else "0" for bite in carrot_bytes]
            map = "".join(map)
            client.publish("purdue-dac/map", map)
        case "purdue-dac/cmd":
            match msg.payload.decode("utf-8"):
                case "0":
                    global game_on
                    game_on = False


# if (datetime.now() - last_time) < timedelta(seconds=4, microseconds=500):
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
carrot_bytes = [5, 5, 5, 5, 5]

passcode = "0101KurzweiL99"
team_id = "1"

client.connect("localhost", 1883, 60)
game_on = True
start_time = datetime.now()
client.publish("purdue-dac/map", "00000")

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_start()

while (datetime.now() - start_time) < timedelta(minutes=5) and game_on:
    time.sleep(1)

end_time = datetime.now()
client.publish("purdue-dac/sound", "3")
client.loop_stop()

time_use = (end_time - start_time).seconds
score = 0 if time_use > 300 else 300 - time_use

for bite in carrot_bytes:
    if bite > 0:
        score += 50 * (5 - bite)
    else:
        score += 300

print(time_use)
print(carrot_bytes)
print(score)

res = requests.get(
    "https://rabbitrun.digitalagclub.org/api/stopGame", params={"passcode": passcode}
)

if input("Trial or get caught: ").lower() in ["yes", "y"]:
    score = 0
    print(f"Score overwrite to {score}")


res = requests.get(
    "https://rabbitrun.digitalagclub.org/api/pushScore",
    params={"teamid": team_id, "passcode": passcode, "score": score},
)
