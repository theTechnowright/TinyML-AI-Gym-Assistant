
/* Includes ---------------------------------------------------------------- */
#include <Repitition-counter_inferencing.h> //Library Downloaded from Edge Impulse
#include <Arduino_LSM9DS1.h> //Click here to get the library: https://www.arduino.cc/reference/en/libraries/arduino_bmi270_bmm150/
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

#define OLED_RESET 4

BLEService batteryService("468c28f3-35a2-4fe8-9e68-be7942c4f152");

// Bluetooth® Low Energy Battery Level Characteristic
BLEStringCharacteristic batteryLevelChar("bd38d6a4-d815-4e50-a9c0-7b9420043151",  // standard 16-bit characteristic UUID
    BLERead | BLENotify,8); // remote clients will be able to get notifications if this characteristic changes

Adafruit_SSD1306 display(128, 64, &Wire);
// this is the bitmap, change this variable for your specification.
const unsigned char myBitmap [] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00, 
	0x01, 0xff, 0xc0, 0x00, 0x01, 0xff, 0xe0, 0x00, 0x01, 0xfd, 0xf0, 0x00, 0x00, 0x00, 0xf8, 0x00, 
	0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x3f, 0x80, 
	0x00, 0x00, 0x3f, 0xc0, 0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00, 0x1f, 0xf0, 
	0x00, 0x00, 0x1f, 0xf8, 0x00, 0x1f, 0x9f, 0xfc, 0x0f, 0x3f, 0xff, 0xfc, 0x1f, 0xff, 0xff, 0xfe, 
	0x3f, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xe0, 
	0x3f, 0xff, 0xff, 0x80, 0x3f, 0xff, 0xfe, 0x00, 0x1f, 0xff, 0xf8, 0x00, 0x0f, 0xff, 0xe0, 0x00, 
	0x07, 0xfe, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  4.0f        // starting 03/2022, models are generated setting range to +-2, but this example use Arudino library which set range to +-4g. If you are using an older model, ignore this value and use 4.0f instead
static const float means[] = {1.841, -4.7237, 1.3253, 13.3613, 3.7575, 6.2802};
static const float std_devs[] = {7.7014, 5.7553, 3.1505, 53.7929, 40.4529, 96.1022};
float acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
int myExercises[3] = {0}; //array created in the order : bicepCurlCount, chestPressCount, tricepPulldownCount
String repCounts ="0,0,0";

unsigned long startTime;
unsigned long currentTime;

const float accelerationThreshold = 2.3;

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // here the 0x3c is the I2C address, check your i2c address if u have multiple devices.
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.print("Perform excercise to run the inference!");
    display.display();
    // comment out the below line to cancel the wait for USB connection (needed for native USB)
    //while (!Serial);
    Serial.println("Edge Impulse Inferencing: Move the watch to run the inference!");
    
    if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }
    if (!IMU.begin()) {
        ei_printf("Failed to initialize IMU!\r\n");
    }
    else {
        IMU.setContinuousMode();
        ei_printf("IMU initialized\r\n");
    }
    BLE.setLocalName("GymRepCounter");
    BLE.setAdvertisedService(batteryService); // add the service UUID
    batteryService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
    BLE.addService(batteryService); // Add the battery service
    BLE.advertise();

    Serial.println("Bluetooth® device active, waiting for connections...");

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
        return;
    }
}

/**
 * @brief Return the sign of the number
 * 
 * @param number 
 * @return int 1 if positive (or 0) -1 if negative
 */
float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop(){
  BLEDevice central = BLE.central();
  //updateRepCounts();
  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // check the battery level every 200ms
    // while the central is connected:
    startTime=millis();
    while (central.connected()) {
      //Serial.println(repCounts);
      //delay(2000);
      batteryLevelChar.writeValue("0,0,0");
      currentTime = millis();
      if(currentTime-startTime> 4000){
        break;
      }
    }
    while(central.connected()){
      updateRepCounts();
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }  
}

void updateRepCounts()
{   
    while(true){
      if (IMU.accelerationAvailable()) {
      // read the acceleration data
      IMU.readAcceleration(acc_x, acc_y, acc_z);

      // sum up the absolutes
      float aSum = fabs(acc_x) + fabs(acc_y) + fabs(acc_z);

      // check if it's above the threshold
      if (aSum >= accelerationThreshold) {
        break;
        }
      }
    }
    //ei_printf("\nStarting inferencing in 2 seconds...\n");
    ei_printf("Sampling...\n");

    // Allocate a buffer here for the values we'll read from the IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        //IMU.readAcceleration(buffer[ix], buffer[ix + 1], buffer[ix + 2]);
        IMU.readAcceleration(acc_x, acc_y, acc_z);
        acc_x *= CONVERT_G_TO_MS2;
        acc_y *= CONVERT_G_TO_MS2;
        acc_z *= CONVERT_G_TO_MS2;

        /*for (int i = 0; i < 3; i++) {
            if (fabs(buffer[ix + i]) > MAX_ACCEPTED_RANGE) {
                buffer[ix + i] = ei_get_sign(buffer[ix + i]) * MAX_ACCEPTED_RANGE;
            }
        }*/
        acc_x = (acc_x - means[0]) / std_devs[0];
        acc_y = (acc_y - means[1]) / std_devs[1];
        acc_z = (acc_z - means[2]) / std_devs[2];
       /* gyr_x = (gyr_x - means[3]) / std_devs[3];
        gyr_y = (gyr_y - means[4]) / std_devs[4];
        gyr_z = (gyr_z - means[5]) / std_devs[5];*/

        buffer[ix + 0] = acc_x;
        buffer[ix + 1] = acc_y;
        buffer[ix + 2] = acc_z;
        /*buffer[ix + 3] = gyr_x;
        buffer[ix + 4] = gyr_y;
        buffer[ix + 5] = gyr_z;*/

        delayMicroseconds(next_tick - micros());
    }

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (result.classification[ix].value > 0.6) myExercises[ix]++;
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }
        repCounts = String(myExercises[0])+","+String(myExercises[1])+","+String(myExercises[2]);

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 20);
  // Display static text
        display.print("Bicep Curl:");
        display.print(myExercises[0]);
        display.setCursor(0, 30);
        display.print("Chest Press:");
        display.print(myExercises[1]);
        display.setCursor(0, 40);
        display.print("Tricep Pulls:");
        display.print(myExercises[2]);
        display.drawBitmap(96, 0, myBitmap, 32, 32, WHITE);
        display.display();
        batteryLevelChar.writeValue(repCounts);

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
}
#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER
#error "Invalid model for current sensor"
#endif
