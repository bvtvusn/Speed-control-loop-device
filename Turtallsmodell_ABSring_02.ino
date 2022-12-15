#include <Chrono.h>
#include <Arduino_JSON.h>

Chrono chrono_SerialSend;

int SerialSendInterval = 20*64; // delay * 64 because of timer prescaler

const int pwmPin = 10;
const int dirPin = 11;
const int forcePin = 14;

const int encoderAPin = 2;
const int encoderBPin = 3;
bool encoderStateA_prev = false;
bool encoderStateB_prev = false;
int externalMotorSignal = 0;
volatile long encoderPos = 0;
long encoderPosPrev = 0;
float encoderSpeed = 0;
float encoderSpeedFilt = 0;
int measuredTime = 0;
//
//float angle_cur = 0; // Measured angle
//float angle_prev = 0;
//float angle_speed = 0;// Measured rotation speed

String msg = "";
char character;

volatile unsigned long timeTminus1 = 0;
volatile unsigned long timeTminus0 = 1;

void setup() {
  // put your setup code here, to run once:

  // INcreases the clock by a factor of 64.
  TCA0.SINGLE.CTRLA = (TCA_SINGLE_CLKSEL_DIV1_gc) | (TCA_SINGLE_ENABLE_bm);
  
  Serial.begin(115200);

  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  pinMode(encoderAPin, INPUT_PULLUP);
  pinMode(encoderBPin, INPUT_PULLUP);

  pinMode(forcePin, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderAPin), ISR_ChA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBPin), ISR_ChB, CHANGE);
}

void loop() {


  //if (encoderStateA_prev != digitalRead(encoderAPin)){
  //  encoderStateA_prev = !encoderStateA_prev;
  //  Serial.print("CH A: ");
  //  Serial.println(encoderStateA_prev);
  //}
  // Send state over serial port:
  if (chrono_SerialSend.hasPassed(SerialSendInterval)) {
    measuredTime = chrono_SerialSend.elapsed();
    chrono_SerialSend.restart();



    int diff = encoderPos - encoderPosPrev;
    encoderPosPrev = encoderPos;
    encoderSpeed = (float)diff / (float)measuredTime * 1000.0 * 64.0;
    float filterFactor = 0.9;
    encoderSpeedFilt = encoderSpeedFilt * filterFactor + encoderSpeed * (1- filterFactor);

    JSONVar myObject;
    myObject["speed"] = encoderSpeed; //round2(encoderSpeed);
    myObject["speedFilt"] = round2(encoderSpeedFilt);
    myObject["angle"] = encoderPos; // round2(encoderPos);
    myObject["diff"] = diff;
    myObject["dt"] = measuredTime;
    myObject["force"] = analogRead(forcePin);
    //myObject["1_t"] = 1.0/(timeTminus0 - timeTminus1);
    Serial.println(myObject); //Serial.write("\r\n");
    //      Serial.print("Pos: ");
    //      Serial.print(encoderPos);
    //      Serial.print(", Speed: ");
    //      Serial.println(encoderSpeed);

  }

  //  Serial.print("CH A: ");
  //  Serial.print(digitalRead(encoderAPin));
  //  Serial.print(", CH B:");
  //  Serial.println(digitalRead(encoderBPin));



  while (Serial.available()) {
    character = Serial.read();
    if (character == '\n') {
      HandleJson(msg); // Set motor speed
      msg = "";
    }
    else {
      msg.concat(character);
    }
  }
}


void ISR_ChA() {
  bool chA = digitalRead(encoderAPin);
  bool chB = digitalRead(encoderBPin);
  if (chA == chB) {
    encoderPos++;
  } else {
    encoderPos--;
  }

  timeTminus1 = timeTminus0;
  timeTminus0 = micros()/64;
  //  Serial.print("CH A: ");
  // Serial.println(digitalRead(encoderAPin));
}
void ISR_ChB() {
  bool chA = digitalRead(encoderAPin);
  bool chB = digitalRead(encoderBPin);
  if (chA != chB) {
    encoderPos++;
  } else {
    encoderPos--;
  }

  timeTminus1 = timeTminus0;
  timeTminus0 = micros()/64;
  //  Serial.print("CH B: ");
  // Serial.println(digitalRead(encoderBPin));
}

void HandleJson(String msg) {
  JSONVar myObject = JSON.parse(msg);
  if (myObject.hasOwnProperty("mv")) {
    externalMotorSignal = (int) myObject["mv"];
    Serial.println(externalMotorSignal);
    RunMotor(externalMotorSignal);

    //Serial.println(externalMotorSignal);
  }
}
double round2(double value) {
  return (int)(value * 100 + 0.5) / 100.0;
}

void RunMotor(int mspeed ) {
  if (mspeed > 0) {
    digitalWrite(dirPin, HIGH);
  }
  else {
    digitalWrite(dirPin, LOW);
  }

  analogWrite(pwmPin, abs(mspeed));
}
