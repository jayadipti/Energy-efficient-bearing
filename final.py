import time
import numpy as np
import tensorflow as tf
import random  

class ADC:
    def __init__(self, pin):
        self.pin = pin

    def read(self):
        return random.randint(500, 3000) 

class Pin:
    def __init__(self, pin_number):
        self.pin_number = pin_number

class WDT:
    def __init__(self, timeout):
        self.timeout = timeout

    def feed(self):
        print("Watchdog fed")

def unique_id():
    return b"123456789ABC"

def reset():
    print("Simulated Reset")


class WLAN:
    def __init__(self, mode):
        self.connected = False

    def active(self, state):
        pass

    def connect(self, ssid, password):
        self.connected = True 

    def isconnected(self):
        return self.connected

    def ifconfig(self):
        return ("192.168.1.10", "255.255.255.0", "192.168.1.1", "8.8.8.8")

STA_IF = "STA_IF"


class MQTTClient:
    def __init__(self, client_id, broker):
        self.client_id = client_id
        self.broker = broker

    def connect(self):
        print(f"Simulated MQTT connection to {self.broker}")

    def publish(self, topic, message):
        print(f"Published to {topic}: {message}")

    def disconnect(self):
        print("MQTT Disconnected")


SSID = "your_SSID"
PASSWORD = "your_PASSWORD"


MQTT_BROKER = "your_mqtt_broker"
MQTT_TOPIC = "bearing/sensor_data"
ALERT_TOPIC = "bearing/alerts"
CLIENT_ID = unique_id()


def connect_wifi():
    wlan = WLAN(STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)

    max_retries = 10
    retry_count = 0
    while not wlan.isconnected() and retry_count < max_retries:
        time.sleep(1)
        retry_count += 1

    if wlan.isconnected():
        print("Connected to Wi-Fi: ", wlan.ifconfig())
    else:
        print("Failed to connect to Wi-Fi, restarting...")
        reset()


temp_sensor = ADC(Pin(34))
vibration_sensor = ADC(Pin(35))
load_sensor = ADC(Pin(32))

def calibrate_sensor(sensor):
    try:
        baseline = sum(sensor.read() for _ in range(10)) / 10
        return baseline
    except Exception as e:
        print("Sensor calibration failed:", e)
        return 0

temp_baseline = calibrate_sensor(temp_sensor)
vib_baseline = calibrate_sensor(vibration_sensor)
load_baseline = calibrate_sensor(load_sensor)

def moving_average(values, window=5):
    if len(values) == 0:
        return 0
    return sum(values[-window:]) / min(len(values), window)

sensor_history = {"temp": [], "vib": [], "load": []}


def read_sensors():
    try:
        raw_temp = (temp_sensor.read() - temp_baseline) * (3.3 / 4095.0) * 100
        raw_vib = (vibration_sensor.read() - vib_baseline) * (3.3 / 4095.0) * 10
        raw_load = (load_sensor.read() - load_baseline) * (3.3 / 4095.0) * 50

        sensor_history["temp"].append(raw_temp)
        sensor_history["vib"].append(raw_vib)
        sensor_history["load"].append(raw_load)

        return (
            round(moving_average(sensor_history["temp"]), 2),
            round(moving_average(sensor_history["vib"]), 2),
            round(moving_average(sensor_history["load"]), 2)
        )
    except Exception as e:
        print("Error reading sensors:", e)
        return 0, 0, 0


def detect_anomaly(temp, vib, load):
    threshold_temp = max(80, moving_average(sensor_history["temp"]) * 1.2)
    threshold_vib = max(500, moving_average(sensor_history["vib"]) * 1.3)
    threshold_load = max(900, moving_average(sensor_history["load"]) * 1.1)

    if temp > threshold_temp or vib > threshold_vib or load > threshold_load:
        print("Anomaly detected! Immediate inspection required.")
        return True
    return False


def load_model():
    try:
        
        interpreter = tf.lite.Interpreter(model_path="bearing_model.tflite")
        interpreter.allocate_tensors()
        return interpreter
    except Exception as e:
        print("Failed to load AI model:", e)
        return None

def predict_failure(temp, vib, load, interpreter):
    if not interpreter:
        return 0
    try:
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        input_data = np.array([[temp, vib, load]], dtype=np.float32)
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        prediction = interpreter.get_tensor(output_details[0]['index'])

        return round(float(prediction[0]), 2)  # ✅ Extract the scalar value and round it
    except Exception as e:
        print("Prediction error:", e)
        return 0


def send_alert(temp, vib, load):
    try:
        client = MQTTClient(CLIENT_ID, MQTT_BROKER)
        client.connect()
        timestamp = time.time()
        payload = f"ALERT: {timestamp} | Temp: {temp}C, Vib: {vib}, Load: {load}. Immediate attention needed."
        client.publish(ALERT_TOPIC, payload)
        client.disconnect()
        print("Alert sent to MQTT broker")
        with open("alert_log.txt", "a") as log_file:
            log_file.write(f"ALERT: {timestamp} - Temp: {temp}C, Vib: {vib}, Load: {load}\n")
    except Exception as e:
        print("Failed to send alert:", e)


connect_wifi()
model = load_model()
wdt = WDT(timeout=60000)  # 60-second watchdog timer

while True:
    temp, vib, load = read_sensors()
    print(f"Temp: {temp}C, Vibration: {vib}, Load: {load}")
    prediction = predict_failure(temp, vib, load, model)
    print(f"Failure Risk: {prediction}")
    anomaly = detect_anomaly(temp, vib, load)
    if anomaly:
        send_alert(temp, vib, load)
    wdt.feed()  # Reset watchdog timer
    time.sleep(10)  # Send data every 10 seconds
