#include <ArduinoBLE.h>

// =============== CUSTOMIZE THE VARIABLES BELOW ===============

#define DEBUG false // set to true when debugging, false otherwise

#define DEBUG_SERIAL if(DEBUG)Serial // https://forum.arduino.cc/t/serial-debug-toggle/632623/8

// during lock & unlock, latch bolt's motor runs for a set period of time since there is currently no sensor feedback to indicate the position of the latch bolt
const word activeMotorTime = 800; // milliseconds for latch bolt to fully extend - can be any integer from 0 to 65535
const byte motorLeadPos = 2;      // D2
const byte motorLeadNeg = 3;      // D3
const byte doorSensor = A7;       // D21 - used as interrupt pin to detect when sensor's state changes. Available interrupt pins: https://www.arduino.cc/reference/cs/language/functions/external-interrupts/attachinterrupt/
const byte debounceDelay = 20;    // door sensor (magnetic reed switch) debounce milliseconds - can be any integer from 0 to 255
const char* displayName = "BLELock";

// =============== REFRAIN FROM CHANGING THINGS BELOW ============

volatile unsigned long timeWhenSensorTriggered = -1; // holds the millis() value when the doorSensor interrupt triggers, initialized to ULONG_MAX

BLEBoolCharacteristic latchCharacteristic("2AE2", BLERead | BLEIndicate | BLEWrite); // create 'latch' characteristic, allow remote device to read, indicate & write
BLEBoolCharacteristic sensorCharacteristic("2AE2", BLERead | BLEIndicate);           // create 'sensor' characteristic, allow remote device to read & indicate

void setup() {
  DEBUG_SERIAL.begin(9600);

  while(DEBUG && !Serial); // prevents any code from running until you open the serial monitor

  // configure GPIO pins
  pinMode(motorLeadPos, OUTPUT);
  pinMode(motorLeadNeg, OUTPUT);
  pinMode(doorSensor, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(doorSensor), sensorChangeHandler, CHANGE);

  // initialize BLE
  if (!BLE.begin()) {
    DEBUG_SERIAL.println("starting BLE failed!");
    NVIC_SystemReset(); // reset the microcontroller
  }

  BLE.setDeviceName(displayName);
  BLE.setLocalName(displayName);

  BLE.setEventHandler(BLEConnected, blePeripheralConnectionHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralConnectionHandler);

  latchCharacteristic.setValue(true); // initialize latch characteristic's value -> locked
  latchCharacteristic.setEventHandler(BLERead, latchReadHandler);
  latchCharacteristic.setEventHandler(BLEWritten, latchWriteHandler);

  sensorCharacteristic.setValue(digitalRead(doorSensor)); // initialize sensor characteristic's value
  sensorCharacteristic.setEventHandler(BLESubscribed, sensorSubscriptionHandler);
  sensorCharacteristic.setEventHandler(BLEUnsubscribed, sensorSubscriptionHandler);
  sensorCharacteristic.setEventHandler(BLERead, sensorReadHandler);

  BLEService doorService("1815"); // create 'automation io' service - can't find a more appropriate GATT service
  doorService.addCharacteristic(latchCharacteristic);
  doorService.addCharacteristic(sensorCharacteristic);

  BLE.addService(doorService);
  BLE.setAdvertisedService(doorService); // the UUID for the service this peripheral advertises
  BLE.advertise();

  DEBUG_SERIAL.println(("Bluetooth device active, waiting for connections..."));

  // make sure latch is in locked position at startup
  driveMotorForPeriod(latchCharacteristic.value(), activeMotorTime);
}

void blePeripheralConnectionHandler(BLEDevice central) {
  DEBUG_SERIAL.println("Central device " + central.address() + (central.connected() ? " connected" : " disconnected"));
}

void sensorSubscriptionHandler(BLEDevice central, BLECharacteristic characteristic) {
  DEBUG_SERIAL.println("Central device " + central.address() + (characteristic.subscribed() ? " subscribed to " : " unsubscribed from ") + "characteristic " + characteristic.uuid());
}

void sensorReadHandler(BLEDevice central, BLECharacteristic characteristic) {
  DEBUG_SERIAL.println("Central device " + central.address() + " read that the door is " + (sensorCharacteristic.value() ? "closed" : "open"));
}

void latchReadHandler(BLEDevice central, BLECharacteristic characteristic) {
    DEBUG_SERIAL.println("Central device " + central.address() + " read that the latch is " + (latchCharacteristic.value() ? "locked" : "unlocked"));
}

void latchWriteHandler(BLEDevice central, BLECharacteristic characteristic) {
    DEBUG_SERIAL.println("Central device " + central.address() + " commanded the latch to " + (latchCharacteristic.value() ? "lock" : "unlock"));
    driveMotorForPeriod(latchCharacteristic.value(), activeMotorTime);
}

void sensorChangeHandler() {
  timeWhenSensorTriggered = millis(); // using millis() in the interrupt is ok since we spend very little time in this function - https://forum.arduino.cc/t/can-i-use-millis-when-i-using-hardware-interrupts/353114/3
}

void driveMotor(boolean clockwise) { // if clockwise is truthy, motor should rotate CW when viewed from the shaft end, otherwise it should rotate CCW
  digitalWrite(motorLeadPos, clockwise);
  digitalWrite(motorLeadNeg, !clockwise);
}

void stopMotor() {
  digitalWrite(motorLeadPos, LOW);
  digitalWrite(motorLeadNeg, LOW);
}

void driveMotorForPeriod(boolean clockwise, word timePeriod) {
  driveMotor(clockwise);
  delay(timePeriod);
  stopMotor();
}

void loop() {
  BLE.poll(); // poll for BLE events

  if (millis() > timeWhenSensorTriggered && millis() - timeWhenSensorTriggered > debounceDelay) { // the first check is needed because millis() - timeWhenSensorTriggered can't be negative (it's an unsigned long)
    sensorCharacteristic.writeValue(digitalRead(doorSensor)); // unfortunately writeValue() crashes the BLE when called in the interrupt callback
    DEBUG_SERIAL.println((String)"Door is now " + (sensorCharacteristic.value() ? "closed" : "open"));
    timeWhenSensorTriggered = -1; // set to ULONG_MAX, ensures this if-block runs one-time-only after the sensor state changes - even in the case where millis() overflows
  }
}
