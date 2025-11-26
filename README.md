# Heron-Alarm
Heron Detection and Alarm System


For five years, one great blue heron has devastated the fish population in my parents' fish pond in their backyard.  
Once these birds locate a source of prey, they never forget that location throughout their entire lifespan of around  
15 years. Several measures have been placed to prevent the heron from taking the fish, but none have proved to be  
absolutely successful.  

Nets have been placed roughly 6 feet above the ground and over the water to prevent the heron from coming above. When  
this proved to be unsuccessful, nets were placed around the pond, creating a fully enclosed pond. This does seem to  
function but comes with visual costs. This has inspired me to create an alarm system to inhibit the heron from  
consuming the fish and remove the nets for a better backyard aesthetic. 


## *"To know your enemy, you must become your enemy."*  
### — Sun Tzu

The great blue heron is a skittish bird that primarily eats at dawn and dusk. These times range from **06:00 - 08:00** and  
**20:00 - 22:00**, depending on the time of year in which the sun rises and sets.  

Their diet includes fish, particularly slow-moving fish.  
Natural predators include bobcats, coyotes, large snakes, and red-tailed hawks.  
Adult herons typically lack natural predators but are sensitive to pitches within **1–5 kHz**.  
They are smarter than the average bird — known to recognize repetitive frequencies over a period of time, losing their  
fear factor and allowing them to feel safe before eating.

## Components & General Concept ##
I began drawing a simple concept from tinkercad, using an arduino uno r3, one motion sensor, and a piezo buzzer. 
These are all simple components, as I do plan to increase the scope of the porject step by step. 
Hopefully, more than three sensors will be installed along with the piezo buzzer being substituted with a 
passive speaker that is capable of playing .mp3 files, eventually moving from UART to I^2C. 
Other components included are a clock to ensure that the alarm system only goes off during heron hunting hours, 
and an LED screen with a potentiometer to change pitch or make other possible adjustments. 
Many of these components are parts that I already have that are leftover from previous projects that I had worked
on throughout my college courses. However, I misjudged what microcontrollers I had in my possession, currently having
the ATMEGA328pb Xplained Miniboard and not the expected arduino r3. The ATMEGA328pb is not ideal for the experimentation
process but will likely be the MCU I use for the final product. Arduino Uno R3 is much more ideal for experimentation. 

## BOM ##

1. 2 pi pico 
2. PIR HC-SR501 motion detector
3. active piezo buzzer
4. 2x LM2596 buck converter 
5. 2x RS-485 
6. CAT5e
7. CAN Receiver 


# Pi pico, PIR HC-SR501 motion detector, active piezo buzzer.#


Uploaded basic code from ArduinoIDE to test. Everything works. 
Currently, the active buzzer will **click** (quietly) once the motion sensor is triggered.
The buzzer will eventually be changed to a passive buzzer. The motion sensors are too sensitive over a larger area, 
must find a good method for adjusting its total perception, perhaps use a pipe or maybe even a 3D printed object in the 
future.   


# Sending and Receiving Through RS-485s #
RS-485
Two RS-485 transceiver boards are used: one at the sender Pico and one at the receiver Pico. The transceivers convert the Pico’s UART (3.3 V logic) into a differential signal on the CAT5e pair. The RS-485 uses six onboard pins, TX, RX, DE, RE, GND and Power. TX and RX are wired to GP0 and GP1, these are the transmission and receiving pins. DE and RE stand for driver enable and receive enable, these are wired together as RE and DE are turned on and off at the same time, only requiring one shared pulse.GND goes to GND and, and power goes to power. On both MCUs the RS-485s are wired the exact same. Between the RS-485s are cat5e wires pinched into both RS-485s, labeled A, B, and GND. There is a ground pin and a ground (blue thing on rs-485). Ensure that the RS-485s are connected from A->A, B->B, and GND->GND. 
GP0 (TX) → DI / TXD
GP1 (RX) ← RO / RXD
GP2 → DE and /RE (tie these two together)
3V3  → VCC
GND → GND
# PIR Motion Sensor #
The sending MCU has a  motion sensor with three pins, GND, Power, and Out. Naturally, GND and power are sent to GND and the pico’s power supply, and the out pin will be plugged into another GP pin, GP8 in this case. The MCU was powered by a USB. 
PIR VCC → Pico 3V3
PIR GND → Pico GND
PIR OUT → GP8
# Buck Converter and LED #
The receiving MCU is powered by a 12V power supply. The internal voltage of the pi pico cannot handle 12V, so it is taken down to about 5V with a buck converter. The buck converter has ports on each side of it, one containing the positive and negative OUTPUT, and the positive and negative INPUT. The power supply is connected to the INPUT side. Once a power supply is detected, adjust the potentiometer on the buck converter to lower the output voltage to 5V. Attach the buck converter’s OUTPUT ports to + VSYS on the pico and - GND.  
GP16 is used to send a pulse to drive the LED when indicated. 

# RS-485 #
Two RS-485 transceiver boards are used: one at the sender Pico and one at the receiver Pico. The transceivers convert the Pico’s UART (3.3 V logic) into a differential signal on the CAT5e pair. The RS-485 uses six onboard pins, TX, RX, DE, RE, GND and Power. TX and RX are wired to GP0 and GP1, these are the transmission and receiving pins. DE and RE stand for driver enable and receive enable, these are wired together as RE and DE are turned on and off at the same time, only requiring one shared pulse. GND goes to GND and, and power goes to power. On both MCUs the RS-485s are wired the exact same. Between the RS-485s are cat5e wires pinched into both RS-485s, labeled A, B, and GND. There is a ground pin and a ground (blue thing on rs-485). Ensure that the RS-485s are connected from A->A, B->B, and GND->GND. 
GP0 (TX) → DI / TXD
GP1 (RX) ← RO / RXD
GP2 → DE and /RE (tie these two together)
3V3  → VCC
GND → GND
# PIR Motion Sensor #
The sending MCU has a  motion sensor with three pins, GND, Power, and Out. Naturally, GND and power are sent to GND and the Pico’s power supply, and the out pin will be plugged into another GP pin, GP8 in this case. The MCU was powered by a USB. 
PIR VCC → Pico 3V3
PIR GND → Pico GND
PIR OUT → GP8
# Buck Converter and LED #
The receiving MCU is powered by a 12V power supply. The internal voltage of the pi pico cannot handle 12V, so it is taken down to about 5V with a buck converter. The buck converter has ports on each side of it, one containing the positive and negative OUTPUT, and the positive and negative INPUT. The power supply is connected to the INPUT side. Once a power supply is detected, adjust the potentiometer on the buck converter to lower the output voltage to 5V. Attach the buck converter’s OUTPUT ports to + VSYS on the pico and - GND.  
GP16 is used to send a pulse to drive the LED when indicated. 
# System Overview #
Two pi picos are assigned a send and a receive role, the sender reacting to a PIR motion trigger and the receiver responding by lighting an LED. To do this, the system uses a fixed-length frame over UART through the RS-485. The sync byte, the payload byte, and the checksum byte. The receiver explicitly waits for the sync byte, staying in an idle state throughout the loop until the sync byte has been located. The payload byte is assigned to 0x01 in the receiver code, so, when the specified payload byte is sent, it indicates for the LED to flash. The checksum byte ensures that the payload byte was uncorrupted and arrived at the right location. The sender  will compute and send a check value, the receiver then recomputes that check value and matches it with the received check byte.  If the check values match, then the received frame is accepted and acted upon, otherwise it is discarded. Because the RS-485 modules are half-duplex, the Pico ties DE and RE together and drives that node from a GPIO: LOW keeps the transceiver in receive mode, and HIGH enables transmit mode only while the bytes are being sent. The sender will initialize UART, monitor the PIR motion sensors with a small debounce, and build then send the three 3-byte frame. Then the receiver initializes UART, and runs a tiny state machine that waits for the sync byte. It reads the payload byte and check byte, then verifies that the check byte values are the same, if they are the same, then the LED light is driven. RS-485 can support multiple nodes on the same A/B pair, if expanded beyond two devices, add an address byte so only the intended node acts, and if it is ever needed to be simultaneous, two-way traffic can switch to 4-wire RS-485. 


# Wireless Sending and Receiving Pico W #

# Description #
Two pi pico Ws communicate by sending a trigger event and receiving that trigger event to light an LED over wifi using UDP. The client (sender) watches a PIR motion sensor; on a rising edge it transmits a tiny trigger message. The server (receiver) listens on a UDP port, validates the message, and drives an LED.

# Wiring #
GP16 → 330 Ω → LED → GND
PIR (sender): VCC → 3V3, GND → GND, OUT → GP8

# System Overview #
The Pico W has a CYW43439 wireless chip which adds a Wifi and Bluetooth module to the board, allowing the microcontroller to wireless connect to the internet. The client has a PIR motion sensor, that upon a PIR edge will activate a trigger that is sent to the server side, which checks and then acts upon that event. Each board will connect to an SSID, the receiver binds a socket, and the client sends a fixed 3-byte frame.  The data sent is composed of a sync byte, payload byte, and a checksum byte. The receiver loops over the UDP socket to continuously check for sent signals. 



# Final Design #
After testing different communication protocols, it has become obvious that this alarm system can function with a single MCU.  
2x PIR Sensors (cross aimed)
1 Siren
1 Clock (external clock powered by volts, won't miss ticks) 
12V DC solenoid valve 
12V Power supply + buck converter











