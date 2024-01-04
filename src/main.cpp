 
#include <Arduino.h>  // needed in PlatformIO
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>

// Function declarations
void calibrateMPU6050();
float calculateDistance();

// Initialize BME280
Adafruit_BME280 bme;

// Initialize OLED display & MPU6050
Adafruit_SSD1306 display(128, 64, &Wire);
Adafruit_MPU6050 mpu;

bool recording = false;
unsigned long startTime, endTime;

// Global variables for storing offset values, used for Calibration
float ax_offset, ay_offset, az_offset;
float gx_offset, gy_offset, gz_offset;

// Global variables for implementing function switching/distance calculation/speed calculation
const int buttonPin = 3;  // Pin for the switch button
bool displayMode = false;       // Display mode, false for temperature and humidity, true for motion distance and average pace
unsigned long lastTime = 0;
float totalDistance = 0;
float averageSpeed = 0;




void setup() {
  Serial.begin(115200);
  
  // Set up the switch button pin
  pinMode(buttonPin, INPUT_PULLUP);

  // Begin BME280
  if (!bme.begin(0x76)) {  // Change to 0x77 depending on your wiring
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Begin OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Change address if needed
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  display.ssd1306_command(0xA0); // Horizontally mirror the content
  
  display.display();
  delay(2000);
  display.clearDisplay();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  calibrateMPU6050();  // Calibrate MPU6050

}

void loop() {
  // Read the state of the switch button
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(buttonPin);
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    displayMode = !displayMode;  // Switch display mode
    Serial.print("Display mode switched to: "); // To check if the button works
    Serial.println(displayMode ? "Distance/Speed" : "Temperature/Humidity");
    delay(200);  // Debounce delay
  }
  lastButtonState = currentButtonState;

  if (displayMode) {
    // Display motion distance and average pace
    if (millis() - lastTime >= 5000) {
      // Calculate the total distance and average speed every 5000 milliseconds
      totalDistance += calculateDistance();
      averageSpeed = totalDistance / ((millis() - startTime) / 1000.0);

      lastTime = millis();
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Distance: ");
    display.print(totalDistance);
    display.println(" m");

    display.setCursor(0, 24);
    display.print("Avg Speed: ");
    display.print(averageSpeed);
    display.println(" m/s");
    display.display();
  } else {
  
  // Code to display temperature and humidity
  // Read data from BME280
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();

  // Send data to serial port
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(", Humidity: ");
  Serial.println(humidity);

  // Display data on OLED
  display.clearDisplay();
  
  // Display the data on OLED
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("Temp: ");

  display.setCursor(0, 24);
  display.print("Hum: ");

  // Display the values and units of temperature and humidity (text size 2)
  display.setTextSize(2);
  display.setCursor(36, 0); // Adjust the position of the temperature value
  display.print(temperature);
  display.println(" C");

  display.setCursor(36, 24); // Adjust the position of the humidity value
  display.print(humidity);
  display.println(" %");

  display.display();
  delay(500);
  
  // Serial communication monitoring
  // Read data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply offset to correct the data
  float ax_corrected = a.acceleration.x - ax_offset;
  float ay_corrected = a.acceleration.y - ay_offset;
  float az_corrected = a.acceleration.z - az_offset;

  // Print the corrected acceleration data
  Serial.print("X: "); Serial.print(ax_corrected);
  Serial.print(", Y: "); Serial.print(ay_corrected);
  Serial.print(", Z: "); Serial.println(az_corrected);
  }

  delay(100);

  
  
}


void calibrateMPU6050() {
  int numReadings = 100;
  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;
  sensors_event_t a, g, temp;

  for (int i = 0; i < numReadings; i++) {
    mpu.getEvent(&a, &g, &temp);
    ax_sum += a.acceleration.x;
    ay_sum += a.acceleration.y;
    az_sum += a.acceleration.z;
    gx_sum += g.gyro.x;
    gy_sum += g.gyro.y;
    gz_sum += g.gyro.z;
    delay(10);
  }

  ax_offset = ax_sum / numReadings;
  ay_offset = ay_sum / numReadings;
  az_offset = az_sum / numReadings;
  gx_offset = gx_sum / numReadings;
  gy_offset = gy_sum / numReadings;
  gz_offset = gz_sum / numReadings;
}

float calculateDistance() {
  const float accelerationThreshold = 0.2; // Acceleration threshold
  static unsigned long previousTime = 0;
  static float velocity = 0; // Current velocity
  static float distance = 0; // Total distance

  unsigned long currentTime = millis();
  float timeDelta = (currentTime - previousTime) / 1000.0; // Time difference in seconds

  if (previousTime == 0) {
    previousTime = currentTime;
    return 0;
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float acceleration = sqrt(a.acceleration.x * a.acceleration.x +
                            a.acceleration.y * a.acceleration.y +
                            a.acceleration.z * a.acceleration.z) - 1.0; // Subtract gravity

  // Filter out small acceleration changes
  if (abs(acceleration) > accelerationThreshold) {
    velocity += acceleration * timeDelta; // Calculate velocity
  } else {
    velocity = 0; // Assume velocity is zero if acceleration is below threshold
  }

  distance += velocity * timeDelta; // Calculate distance
  previousTime = currentTime;

  return distance; // Return calculated distance
}
