// IR Sensors
int sensor1 = 12;     // Left most sensor
int sensor2 = 13;    
int sensor3 = 3;
int sensor4 = 4;
int sensor5 = 5;    // Right most sensor

// Ultrasonic sensor
int trig = A0;
int echo1 = 2;
int echo2 = 18; // blm kepake

int counter = 0;

// Initial Values of Sensors
int sensor[5] = { 0, 0, 0, 0, 0 };

// Motor Variables
int ENA = 6; // Right motor
int motorInput1 = 7;
int motorInput2 = 8;
int motorInput3 = 9;
int motorInput4 = 10;
int ENB = 11; // Left motor

//Initial Speed of Motor
int initial_motor_speed = 70;

// PID Constants
float Kp = 25;
float Ki = 0;
float Kd = 15;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

int flag = 0;

int payloadState = 0;
int payloadCategory = 0;

volatile float distanceFront, distanceGripper;
volatile unsigned long timestart1, timeend1, timestart2, timeend2;

float distance;


enum movementState {
    FORWARD = 1,
    REVERSE = 2,
    LEFT = 3,
    RIGHT = 4,
    SHARPLEFT = 5,
    SHARPRIGHT = 6,
    STOP = 7,
};

enum payloadCategory {
    CATEGORY1 = 1,
    CATEGORY2 = 2,
    CATEGORY3 = 3,
    CATEGORY4 = 4,
};

void printSensor() {
    Serial.println(String(sensor[0]) + String(sensor[1]) + String(sensor[2]) + String(sensor[3]) + String(sensor[4]));
}

void moveLeft() {
    Serial.println("Left");
    while (error != FORWARD && error != STOP) {
        sendSignalUltrasonic(trig);
        readLineFollowerSensor();
        analogWrite(ENA, 140);  //Right Motor Speed
        analogWrite(ENB, 60);   //Left Motor Speed
    }
    Serial.println("End Left");
}

void moveRight() {
    Serial.println("Right");
    while (error != FORWARD && error != STOP) {
        sendSignalUltrasonic(trig);
        readLineFollowerSensor();
        analogWrite(ENA, 60);   // Right motor speed
        analogWrite(ENB, 140);   // Left motor speed
    }
    Serial.println("End Right");
}

void moveSharpLeft() {
    sharpLeftTurn();
    Serial.println("Sharp Left");
    while (error != FORWARD) {
        sendSignalUltrasonic(trig);
        readLineFollowerSensor();
        analogWrite(ENA, 80);  //Right Motor Speed
        analogWrite(ENB, 70);   //Left Motor Speed
    }
    Serial.println("End Sharp Left");
    forward();
}

void moveSharpRight() {
    sharpRightTurn();
    Serial.println("Sharp Right");
    while (error != FORWARD) {
        sendSignalUltrasonic(trig);
        readLineFollowerSensor();
        analogWrite(ENA, 70);  //Right Motor Speed
        analogWrite(ENB, 80);   //Left Motor Speed
    }
    Serial.println("End Sharp Right");
    forward();
}

void forward() {
    /*The pin numbers and high, low values might be different depending on your connections */
    digitalWrite(motorInput1, LOW);
    digitalWrite(motorInput2, HIGH);
    digitalWrite(motorInput3, LOW);
    digitalWrite(motorInput4, HIGH);
}
void reverse() {
    /*The pin numbers and high, low values might be different depending on your connections */
    digitalWrite(motorInput1, HIGH);
    digitalWrite(motorInput2, LOW);
    digitalWrite(motorInput3, HIGH);
    digitalWrite(motorInput4, LOW);
}
void right() {
    /*The pin numbers and high, low values might be different depending on your connections */
    digitalWrite(motorInput1, LOW);
    digitalWrite(motorInput2, HIGH);
    digitalWrite(motorInput3, LOW);
    digitalWrite(motorInput4, LOW);
}
void left() {
    /*The pin numbers and high, low values might be different depending on your connections */
    digitalWrite(motorInput1, LOW);
    digitalWrite(motorInput2, LOW);
    digitalWrite(motorInput3, LOW);
    digitalWrite(motorInput4, HIGH);
}
void sharpRightTurn() {
    /*The pin numbers and high, low values might be different depending on your connections */
    digitalWrite(motorInput1, LOW);
    digitalWrite(motorInput2, HIGH);
    digitalWrite(motorInput3, HIGH);
    digitalWrite(motorInput4, LOW);
}
void sharpLeftTurn() {
    /*The pin numbers and high, low values might be different depending on your connections */
    digitalWrite(motorInput1, HIGH);
    digitalWrite(motorInput2, LOW);
    digitalWrite(motorInput3, LOW);
    digitalWrite(motorInput4, HIGH);
}
void stopBot() {
    /*The pin numbers and high, low values might be different depending on your connections */
    // Serial.println("STOP");
    digitalWrite(motorInput1, LOW);
    digitalWrite(motorInput2, LOW);
    digitalWrite(motorInput3, LOW);
    digitalWrite(motorInput4, LOW);
}

void readLineFollowerSensor() {
    sensor[0] = !digitalRead(sensor1);
    sensor[1] = !digitalRead(sensor2);
    sensor[2] = !digitalRead(sensor3);
    sensor[3] = !digitalRead(sensor4);
    sensor[4] = !digitalRead(sensor5);

    if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 0) {
        error = FORWARD;
    } else if ((sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 0) || 
               (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 0) ||
               (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1) ||
               (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 1 && sensor[4] == 1) ||
               (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 1)) {
        error = RIGHT;
    } else if ((sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 0) ||
               (sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0) ||
               (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 0) ||
               (sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 0 && sensor[4] == 0) ||
               (sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0)) {
        error = LEFT;
    } else if ((sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0) ||
               (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)) {
        error = STOP;
    } else if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1) {
        if (payloadCategory == CATEGORY1) {
            error = SHARPLEFT;
        } else if (payloadCategory == CATEGORY2) {
            error = SHARPRIGHT;
        } else {
            error = FORWARD;
        }
        error = SHARPLEFT;
    } else {
        error = FORWARD;
    }
}

void calculate_pid() {
    P = error;
    I = I + previous_I;
    D = error - previous_error;

    PID_value = (Kp * P) + (Ki * I) + (Kd * D);

    previous_I = I;
    previous_error = error;
}

void motor_control() {
    // Calculating the effective motor speed:
    int left_motor_speed = initial_motor_speed - PID_value;
    int right_motor_speed = initial_motor_speed + PID_value;

    // The motor speed should not exceed the max PWM value
    left_motor_speed = constrain(left_motor_speed, 0, 255);
    right_motor_speed = constrain(right_motor_speed, 0, 255);

    analogWrite(ENA, right_motor_speed);     //Right Motor Speed
    analogWrite(ENB, left_motor_speed);    //Left Motor Speed

    //following lines of code are to make the bot move forward
    forward();
}

void readFrontSensor() {
    if(digitalRead(echo1)) {
        timestart1 = micros(); 
    } else {
        timeend1 = micros();
        int elapsedTime1 = timeend1 - timestart1;
        distanceFront = elapsedTime1 * 0.0343/2;
    }
}

void readGripperSensor() {
    if(digitalRead(echo2)) {
        timestart2 = micros(); 
    } else {
        timeend2 = micros();
        int elapsedTime2 = timeend2 - timestart2;
        distanceGripper = elapsedTime2 * 0.0343/2;
    }
}

void sendSignalUltrasonic(int trigPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
}

void setup() {
    attachInterrupt(digitalPinToInterrupt(echo1), readFrontSensor, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(echo2), readGripperSensor, CHANGE);

    pinMode(trig, OUTPUT);
    pinMode(echo1, INPUT);
    // pinMode(echo2, OUTPUT);

    pinMode(sensor1, INPUT);
    pinMode(sensor2, INPUT);
    pinMode(sensor3, INPUT);
    pinMode(sensor4, INPUT);
    pinMode(sensor5, INPUT);

    pinMode(motorInput1, OUTPUT);
    pinMode(motorInput2, OUTPUT);
    pinMode(motorInput3, OUTPUT);
    pinMode(motorInput4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    Serial.begin(9600);    //setting serial monitor at a default baund rate of 9600
    delay(500);
    Serial.println("Started !!");
    delay(1000);
}

void loop() {
    readLineFollowerSensor();
    sendSignalUltrasonic(trig);
    if (error == LEFT) {    // Left turn until straight path
        moveLeft();
    } else if (error == RIGHT) {    // Right turn until straight path
        moveRight();
    } else if (error == SHARPLEFT) {    // Make left turn until it detects straight path
        sharpLeftTurn();
    } else if (error == SHARPRIGHT) {   // Make right turn until it detects straight path
        sharpRightTurn();
    } else if (error == STOP || distanceFront <= 10.00) {
        stopBot();
    } else {
        calculate_pid();
        motor_control();
    }
}



