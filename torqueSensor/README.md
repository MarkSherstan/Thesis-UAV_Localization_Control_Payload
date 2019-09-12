# Low Cost Torque Sensor
A low cost torque sensor made from a DLC611N load cell, HX711 amplifier board, and an Arduino Uno.

## Use
1. Open the PlatformIO project.
2. Modify `#define mode` variable to either be 0 for calibration or 1 for measuring.
3. Modify `float calibrationFactor` variable based off calibration results (or leave as 0 if calibrating).
4. Build and upload to Arduino device.
5. Follow the onscreen prompts taking special note of when to load and unload the apparatus. 

## References
Bogde's awesome HX711 library was used and can be referenced [here](https://github.com/bogde/HX711). The [Sparkfun](https://github.com/sparkfun/HX711-Load-Cell-Amplifier) tutorial and library was also used during development.
