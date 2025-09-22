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
5. 2x mutual conversion model 3.3V
6. CAT5e
7. CAN Receiver 


# Pi pico, PIR HC-SR501 motion detector, active piezo buzzer.#


Uploaded basic code from ArduinoIDE to test. Everything works. 
Currently, the active buzzer will **click** (quietly) once the motion sensor is triggered.
The buzzer will eventually be changed to a passive buzzer. The motion sensors are too sensitive over a larger area, 
must find a good method for adjusting its total perception, perhaps use a pipe or maybe even a 3D printed object in the 
future. 

#


# Communicating MCUs
 UART,CAN, RS-485.  









