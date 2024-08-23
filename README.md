The included Arduino Program is designed for a Adafruit Trinket M0. If you want to use a different microcontroller, you may have to modify the code.

Meshtastic Heltec Sensor Config (For Meshtastic App on Tablet):
1. Connect to the Meshtastic Radio that you want to configure. For any config import/modification, the Meshtastic device will have to restart before the config is set.
2. Press three dots on top right corner
3. Go to "Radio configuration"
4. Scroll down until you see "Import configuration" and press it
5. Import the config file named "All Sensor Final Config". This is the current configuration of the current sensor Heltecs.
   There is one called "All Sensor Final Cong" as well. This should also work but I just made a typo.
6. Go back to the "Radio configuration" and find the "Power" tab. This is where you can enable/disable power saving mode. This should be disabled in the imported configuration.
7. Under the "User" tab, change the name of the device to a fitting name. Ex: All Sensor 4
8. If you want to double check if the configuration is right or if you do not have access to the file, the following are the important settings that need to be changed
   - On the main screen, go to the gear tab on the right and make sure your region is US. You can also change your name here as well.
   - On the channels tab (the circle with a sideways wifi Logo), there should be a place to enter a URL. Enter the following URL:
     https://meshtastic.org/e/#CjESIJX6cYPXaTCkQ90MgZlJb7ExMa7bxA76L3uKXQBcwBAvGglNZW5kZXpMYWI6AgggEgwIATgBQANIAVAeaAE 
     The name of the private channel should be "MendezLab"
   - The following settings will be under "Radio configuration" section
     - Go to "Device" and change the role to CLIENT_HIDDEN and enable Serial output
     - Go to "Power" and do the following. **These values may need tinkering**
         - Enable Power Saving Mode (if you see fit)
         - Change the wait for bluetooth setting to 30 seconds
         - Change both super deep sleep and light sleep duration to 300 seconds
         - Change minimum wake time to 10 seconds
      - Go to "Display" and change screen timeout to 5 seconds
      - Go to "Bluetooth" and enable bluetooth (this should already be enabled)
     - Under the "Module Configuration" go to "Serial" and do the following
         - Enable Serial
         - Enable Echo (optional)
         - Change RX pin to 18 and TX pin to 17 (change as needed, just make sure selected pins are actually UART pins or else an error will occur)
         - Change Baud rate to 9600 (this can be changed if needed, just make sure to do so in the Arduino code as well)
         - Set timeout to 1
         - Change the "Serial mode" setting to TEXTMSG

  For the Sensor Receiver Heltec, you can import the same "All Sensor Final Config" file but change the following settings:
  1. Under "Device", change role to CLIENT_MUTE
  2. Under "Serial", change baud rate to 115200
