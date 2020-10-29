from multiprocessing.dummy import Pool
import time
import requests

import sys
import paho.mqtt.client as mqtt
import time

# -----------------  MQTT SUB -------------------
def iot_func_callback_sub(client, userdata, message):
    print("message received ", str(message.payload.decode("utf-8")))
    print("message topic=", message.topic)
    print("message qos=", message.qos)
    print("message retain flag=", message.retain)


def mqtt_subscribe_thread_start(
    arg_callback_func, arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_qos
):
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1)  # wait
        # mqtt_client.loop_forever() # starts a blocking infinite loop
        mqtt_client.loop_start()  # starts a new thread
        return 0
    except:
        return -1


# -----------------  MQTT PUB -------------------
def mqtt_publish(
    arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos
):
    try:
        mqtt_client = mqtt.Client("mqtt_pub")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.loop_start()

        print("Publishing message to topic", arg_mqtt_topic)
        mqtt_client.publish(arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos)
        time.sleep(0.1)  # wait

        mqtt_client.loop_stop()  # stop the loop
        return 0
    except:
        return -1


def push_to_google_sheet(sheet_id, parameters):
    url = "https://script.google.com/macros/s/{}/exec".format(sheet_id)
    parameters["id"] = "Sheet1"
    response = requests.get(url, parameters)

    submission_parametrs = parameters
    submission_parametrs["id"] = "task1"
    submission_parametrs["team_id"] = 1402
    submission_parametrs["unique_id"] = "sVaPtIeT"
    sub_url = "https://script.google.com/macros/s/AKfycbw850dk4moVgebU2GGe0PUQUvvg8jTpSjBQCawJt3_13vgujLk/exec"
    sub_response = requests.get(sub_url, params=submission_parametrs)
    return response.status_code
