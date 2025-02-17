#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_INA219.h>
#include <Bonezegei_DHT11.h>
#include <Servo.h>    //interfaces with the servo, frequent adjustment
#include <Stepper.h>  //interfaces with the stepper, infrequent adjustment

//constructor for bmp085 pressure and temperature sensor.           ID 10085  I2C ADDR: 0x77
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

//INA A is the CONTROL. It is the cell that DOES NOT move
//INA B is the TEST. It is the cell that TRACKS the sun

//constructor for ina219 current sensor. We are using two of these. ID: A     I2C ADDR: 0x40
Adafruit_INA219 ina219A(0x40);
//constructor for ina219 current sensor. We are using two of these. ID: B     I2C ADDR: 0x4C
Adafruit_INA219 ina219B(0x4C);
//constructor for dht11 temperature and humidity sensor. Sensor pin 3
Bonezegei_DHT11 dht(3);

// Stepper motor parameter. Used for Stepper Library.
const int stepsPerRevolution = 2048; 
// initialize the stepper library on pins 10 through 13.
Stepper base_motor(stepsPerRevolution, 13, 11, 12, 10);
// create the servo variable.
Servo servo;
// initalize the servo library on pin 5.
servo.attach(5);
//initalize servo position variable. Start at 90 degrees, facing straight out.
short xvalue = 90;

void onoff(bool x) {
  //used for human readable output of sensor initalization.
  //if init TRUE: sensor is ONLINE, else OFFLINE
  if (x) {
    Serial.println("online");
  } else {
    Serial.println("offline");
  }
}


void serial_init() {
  //serial and sensor init.
  Serial.println("\n\n\n\n\n\n\nPROGRAM START: self check");
  Serial.print("\tTime: ");
  Serial.print(millis());
  Serial.println(" milliseconds since program start");
  Serial.println("Sensor check:");

  //store the return value of init functions.
  char bmpinit = bmp.begin();
  bool ina219a = ina219A.begin();
  bool ina219b = ina219B.begin();
  char dhtinit = dht.begin();

  //print the status of the init functions.
  //Note: the onoff function converts a return code into a human readable form, then prints it.
  Serial.print("\tBMP085 status: ");
  onoff(bmpinit);
  Serial.print("\tINA219 A status: ");
  onoff(ina219a);
  Serial.print("\tINA219 B status: ");
  onoff(ina219b);
  Serial.print("\tDHT11 status: ");
  onoff(dhtinit);
  Serial.print("Sensor check complete. ");

  //print result summary.
  //if all sensors online...
  if ((bmpinit && ina219a && ina219b && dhtinit)) {
    Serial.println("All sensors online.");
  }
  //else, one or more sensors offline... 
  else {
    //print the number of offline sensors.
    Serial.print((int)!bmpinit + (int)!ina219a + (int)!ina219b + (int)!dhtinit);
    Serial.println(" sensors offline.");
  }

  //reduce the max measurable voltage/current of the INA219 sensors, in order to get a greater resolution.
  ina219A.setCalibration_16V_400mA();
  ina219B.setCalibration_16V_400mA();
  Serial.println("INA219 Precision updated to 16V 400mA");

  //print column header, this makes it easy to import data into a spreadsheet.
  setup_columns();
}

void setup_columns() {
  //this code creates the header columns for spreadsheet manipulation.
  print_tab("Time");
  print_tab("Non-movimg solar cell power");
  print_tab("Moving solar cell power");
  print_tab("Non-moving solar cell voltage");
  print_tab("moving solar cell voltage");
  print_tab("Non-moving solar cell current");
  print_tab("Moving solar cell current");
  print_tab("BMP085 Temperature (C)");
  print_tab("BMP085 Pressure (hPa)");
  print_tab("DHT11 Temperature (C)");
  print_tab("DHT11 Humidity (%)");
  Serial.print("\n");  //print newline to ensure first row of data is spaced and placed correctly
  // | Time | con P | mov P | con V | mov V | con I | mov I | BMP T | BMP P | DHT T | DHT H |
}


//This template function standardizes the printing of strings and non-floats, and automatically includes a tab for spreadsheet formatting.
template<typename printable> void print_tab(printable message) {
  Serial.print("\t");
  Serial.print(message);
}

//this template function standardizes the printing of floats, and automatically includes a tab for spreadsheet formatting.
template<typename printable> void print_tab_float(printable message) {
  Serial.print("\t");
  Serial.print(message, 10);
}

void setup() {
  //init serial library at 9600 baud.
  Serial.begin(9600);

  while (!Serial) {  //do not run program until serial comms established
    delay(100);
  }
  delay(5000);  //avoid scheduling conflicts when establishing communication

  //begin pinouts for dht11 power
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  digitalWrite(2, LOW);
  //end pinouts for dht11 power

  serial_init();  // init sensors
  //the stepper motor's speed must be set low so that it does not skip.
  base_motor.setSpeed(10);
}

//this function prints the measured electrical values from the INA219 sensors.
void print_powers() {
  // | con P | mov P | con V | mov V | con I | mov I |

  //initalize variables to print with zero values.
  float shuntvoltage_a, shuntvoltage_b = 0;
  float busvoltage_a, busvoltage_b = 0;
  float current_mA_a, current_mA_b = 0;
  float loadvoltage_a, loadvoltage_b = 0;
  float power_mW_a, power_mW_b = 0;

  //the below code simply requests the data available from the INA219 sensors, and stores it in variables.
  shuntvoltage_a = ina219A.getShuntVoltage_mV();
  shuntvoltage_b = ina219B.getShuntVoltage_mV();
  busvoltage_a = ina219A.getBusVoltage_V();
  busvoltage_b = ina219B.getBusVoltage_V();
  current_mA_a = ina219A.getCurrent_mA();
  current_mA_b = ina219B.getCurrent_mA();
  power_mW_a = ina219A.getPower_mW();
  power_mW_b = ina219B.getPower_mW();

  //we must calculate loadvoltage based on collected data, as the INA219 library does not have a built in method for this.
  loadvoltage_a = busvoltage_a + (shuntvoltage_a / 1000);
  loadvoltage_b = busvoltage_b + (shuntvoltage_b / 1000);

  //finally, print the data we would like to record.
  print_tab_float(power_mW_a);
  print_tab_float(power_mW_b);
  print_tab_float(loadvoltage_a);
  print_tab_float(loadvoltage_b);
  print_tab_float(current_mA_a);
  print_tab_float(current_mA_b);
}

//this function prints all weather sensor data.
void print_atmospherics(char dhtevent = 0) {
  //| BMP T | BMP P | DHT T | DHT H |

  //initalize variable to store bmp data.
  sensors_event_t event;
  //set variable to bmp data.
  bmp.getEvent(&event);
  
  //if we have recieved good data (pressure has been measured, and is not zero), print it, otherwise, print ERR.
  if (event.pressure) {
    print_tab_float(event.temperature);
    print_tab_float(event.pressure);
  } else {
    print_tab("ERR");
    print_tab("ERR");
  }

  //check to make sure DHT has sent data, print ERR if not.
  if (dhtevent) {
    //initalize variable with temperature reading, and print it.
    float tempDeg = dht.getTemperature();
    print_tab_float(tempDeg);
    //initalize variable with humidity reading, and print it.
    float humidity = dht.getHumidity() / (float)100;  //divide by one hundred because humidity is a percentage
    print_tab_float(humidity);
  } else {
    print_tab("ERR");
    print_tab("ERR");
  }
}

/*
print_all()

  This function is designed to run all other functions that log data.
  It runs them in the correct order, to preserve spreadsheet formatting.
  
order as follows:

print time

print voltage/current/wattage

print temperature/pressure/humidity
*/
void print_all() {  
  char dhtevent = dht.getData();  //trigger dht capture first due to lag in sensor measurements.
  delay(2000);                    //delay atleast 2 seconds for DHT11 to read the data
  print_tab(millis());            // record time column
  print_powers();                 // record voltages, currents, wattages
  print_atmospherics(dhtevent);   // record temperature, pressure, humidity.

  Serial.print("\n");//newline character, move to next row of spreadsheet/next line of log file.
}

// this function reads LDR data, and uses it to update the tracker position.
void move_axis() {
  //initalize LDR variables, these store the readings from the analog pins.
  int ldr_right = 0;
  int ldr_left = 0;
  int ldr_top = 0;

  //netX and netY are used for vector math. Initalize them.
  float netX;
  float netY;

  //read every value twice and average for more accurate readings
  ldr_top = analogRead(A1) / 2;
  ldr_top += analogRead(A1) / 2;
  ldr_left = analogRead(A2) / 2;
  ldr_left += analogRead(A2) / 2;
  ldr_right = analogRead(A3) / 2;
  ldr_right += analogRead(A3) / 2;

  //calcluate vector magnitudes.
  netX = ldr_right - ldr_left;
  netY = (ldr_top * 2) - (ldr_right + ldr_left);

  /*
These Net X and Net Y equations may look different from the research paper,
but they are just derivations of them, and equate to the same values.

V(1) is the left LDR value
V(2) is the right LDR value
V(3) is the top LDR value
V(Y) is the resultant Y vector
V(X) is the resultant X vector

Starting from the complete vector version of the y component...
    V(Y) = V(3) + sin(330) * V(1) + sin(330) * V(2)
    V(Y) = V(3) + (sin(330) * V(1) + sin(330) * V(2))
    V(Y) = V(3) + sin(330)(V(1) + V(2))
    V(Y) = V(3) + -.5(V(1) + V(2))
    V(Y) = V(3) - .5(V(1) + V(2))
2 * V(Y) = V(3) * 2 - (V(1) + V(2))
2 * V(Y) = (V(3) * 2) - (V(1) + V(2))
We have therefore derived the final, programmatic, version.
Note that the doubling of V(Y) has no effect on the final movement due to the convertion into a unit vector.

Starting from the complete vector version of the x component...
V(X) = cos(330) * V(2) + cos(210) * V(1)
V(X) = .866 * V(2) + -.866 * V(1)
V(X) = .866 * V(1) - .866 * V(2)
V(X) = .866(V(1) - V(2))
V(X) / .866 = V(1) - V(2)
-1 * (V(X) / .866) = V(2) - V(1)
We have therefore derived the final, programmatic, version.
Note that V(X) being divided by .866 has no effect on the final movement due to the convertion into a unit vector.
Also note that the final value is inverted, this is solely due to the fact that the servo motor's rotation is defined as
positive for clockwise, which would move the solar cell left. Therefore, we simply invert the value to move correctly.

*/

  if (netX != 0) {               //avoid pesky divide by zero errors.
    xvalue += netX / abs(netX);  //convert differential into unit vector, and update servo position variable.
    servo.write(xvalue);         //move servo motor.
  }

  if (netY != 0) {                      //avoid pesky divide by zero errors.
    base_motor.step(netY / abs(netY));  //convert differential into unit vector, and move stepper motor.
  }
}

//main program loop, runs forever.
void loop() {
  //gather all sensor data, and print it.
  print_all();
  //gather LDR data, and use it to update position.
  move_axis();
  //repeat.
}
