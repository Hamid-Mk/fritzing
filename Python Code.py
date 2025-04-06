import wiringpi as wp
import spidev
import smbus2
import time
import paho.mqtt.client as mqtt
from bmp280 import BMP280

wp.wiringPiSetupPhys()

LED_PIN = 7        # this is for the LED
HEATER_RELAY_PIN = 8   # this is for the relay

# Set up the pins
wp.softPwmCreate(LED_PIN, 0, 200)  # make the LED dimming
wp.pinMode(HEATER_RELAY_PIN, wp.OUTPUT) #to send signals not to read (input)
SAFE_EXIT_STATE = wp.HIGH #relay is Active low device
wp.digitalWrite(HEATER_RELAY_PIN, SAFE_EXIT_STATE)

# Set up the MCP3008 for reading the knobs
spi = spidev.SpiDev() #SPI device object used to communicate with the MCP3008.
try:
    spi.open(1, 0)  # spi.open(bus number, device number = mcp3008 )  
except:
    print("SPI not working, check if it's on")
    exit(1)
spi.max_speed_hz = 1350000 #the amount of data (bits) sent per sec.

# Function to read the knobs
def read_adc(channel):
    # Read from MCP3008
    if channel < 0 or channel > 7:
        return -1
    r = spi.xfer2([1, (8 + channel) << 4, 0]) #command byte for channel #
    data = ((r[1] & 3) << 8) + r[2] # command byte to convert data in range of 0 -1023
    return data

# Set up I2C for sensors
i2c_bus = smbus2.SMBus(0) # creates an I2C bus object for bus 0.
bmp280 = BMP280(i2c_addr=0x77, i2c_dev=i2c_bus)  #object = bmp280 using the BMP280 class (address of the BMP280 on the I²C bus + which bus to use)
# Set up light sensor
BH1750_ADDR = 0x23  # address of light sensor

def read_lux(): # Read light level
    try:
        i2c_bus.write_byte(BH1750_ADDR, 0x20) #sends a command byte to the device.
        time.sleep(0.18)  #waits for the sensor to perform the measurement.
        data = i2c_bus.read_i2c_block_data(BH1750_ADDR, 0, 2) # data send back from the sensor
        lux = ((data[0] << 8) | data[1]) / 1.2 #combines two bytes into a 16-bit value
        return lux
    except:
        print("Light sensor not working")
        return -1

# Read knobs for initial values
pot_temp_raw = read_adc(0)
target_temp = 15 + ((pot_temp_raw / 1023.0) * 15)  # convert to 15-30 degrees

pot_led_raw = read_adc(1)
target_led_manual = int((pot_led_raw / 1023.0) * 200)  # convert to 0-200

# MQTT stuff for ThingSpeak
MQTT_HOST = "mqtt3.thingspeak.com"  
MQTT_PORT = 1883
MQTT_TOPIC = "channels/2890246/publish"
MQTT_CLIENT_ID = "LRQMEx4oMgwMHAwIFD0tJjo"
MQTT_USER = "LRQMEx4oMgwMHAwIFD0tJjo"
MQTT_PWD = "iIUzzLFW2Iyn0wYF7o4SYwMN"

# Variables for ThingSpeak timing
last_publish_time = 0
PUBLISH_INTERVAL = 15  # Minimum 15 seconds between ThingSpeak updates

# Connect to MQTT
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, MQTT_CLIENT_ID)
mqtt_client.username_pw_set(MQTT_USER, MQTT_PWD)

# Function for when it connects
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Connected to ThingSpeak!")
    else:
        print("Connection failed with code:", rc)

mqtt_client.on_connect = on_connect

# Try to connect
try:
    print("Connecting to MQTT...")
    mqtt_client.connect(MQTT_HOST, MQTT_PORT, 60)
    mqtt_client.loop_start()
except:
    print("MQTT connection error")
    print("I'll continue anyway")

# Print starting values
print("System starting with initial values:")
print("Target Temperature:", target_temp, "°C")
print("Manual LED Setting:", target_led_manual)
print("If light level is 10 or more, LED will be off.")

# Main loop
try:
    while True:
        # Keep reading knobs
        pot_temp_raw = read_adc(0)
        target_temp = 15 + ((pot_temp_raw / 1023.0) * 15)
        
        pot_led_raw = read_adc(1)
        target_led_manual = int((pot_led_raw / 1023.0) * 200)
        
        # Read sensors
        current_temp = bmp280.get_temperature() # calls a method on the BMP280 sensor to get the current temperatur
        current_lux = read_lux()
        
        # Control temperature
        if current_temp < target_temp:
            wp.digitalWrite(HEATER_RELAY_PIN, wp.LOW)  # send low signal to turn on the heater
            heater_status = 1
            heater_str = "ON"
        else:
            wp.digitalWrite(HEATER_RELAY_PIN, wp.HIGH)  # turn heater off
            heater_status = 0
            heater_str = "OFF"
        
        if current_temp > target_temp + 1:
            temp_status = "WARNING: Temp too high! Current: " + str(current_temp) + "°C"
        else:
            temp_status = "Temperature OK. Current: " + str(current_temp) + "°C"
        
        # Control LED
        # If it's dark (lux < 10), use the knob value, else turn off
        if current_lux < 10:
            led_pwm = target_led_manual
        else:
            led_pwm = 0
        
        wp.softPwmWrite(LED_PIN, led_pwm)
        
        # Format values to 2 decimal places
        formatted_target_temp = f"{target_temp:.2f}"
        formatted_current_temp = f"{current_temp:.2f}"
        formatted_current_lux = f"{current_lux:.2f}"

        # Print values with better formatting
        print("\n" + "=" * 50)
        print("📊 SYSTEM STATUS:")
        print("=" * 50)
        print(f"🌡️  TEMPERATURE")
        print(f"   Target: {formatted_target_temp} °C")
        print(f"   Current: {formatted_current_temp} °C")
        print(f"   Heater: {'🔥 ON' if heater_status == 1 else '❄️ OFF'}")

        # Temperature status with better visibility
        if current_temp > target_temp + 1:
            print(f"   ⚠️  WARNING: Temperature too high!")
        else:
            print(f"   ✅ Temperature OK")

        print("\n💡 LIGHTING")
        print(f"   Light Level: {formatted_current_lux} lux")
        print(f"   Manual LED Setting: {target_led_manual}")
        print(f"   Current LED Power: {led_pwm}")
        print("=" * 50)

        # ThingSpeak status
        current_time = time.time()
        if current_time - last_publish_time >= PUBLISH_INTERVAL:
            print("📡 Ready to send data to ThingSpeak")
        else:
            remaining = PUBLISH_INTERVAL - (current_time - last_publish_time)
            print(f"📡 ThingSpeak update in {remaining:.1f} seconds")

        print("=" * 50)
        
        # Send data to ThingSpeak
        try:
            # make string to send
            # field1 = target temp
            # field2 = current temp
            # field3 = target LED
            # field4 = LED PWM
            # field5 = current lux
            
            current_time = time.time()
            # Only publish if enough time has passed since last publish
            if current_time - last_publish_time >= PUBLISH_INTERVAL:
                mqtt_payload = "field1=" + str(target_temp) + "&field2=" + str(current_temp) + "&" + "field3=" + str(target_led_manual) + "&field4=" + str(led_pwm) + "&" + "field5=" + str(current_lux)
                
                print("Sending:", mqtt_payload)
                mqtt_client.publish(MQTT_TOPIC, mqtt_payload)
                last_publish_time = current_time
        except:
            print("Couldn't send to MQTT")
        
        # Wait 2 seconds before checking again
        time.sleep(2)

except KeyboardInterrupt:
    print("\nQuitting program. Turning things off...")
    wp.softPwmWrite(LED_PIN, 0)
    wp.digitalWrite(HEATER_RELAY_PIN, SAFE_EXIT_STATE)
finally:
    # Clean up
    try:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
    except:
        pass
    i2c_bus.close()
    spi.close()