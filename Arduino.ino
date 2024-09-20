#define enA 5  // Enable1 L298 Pin enA 
#define in1 6   // Motor1 L298 Pin in1 
#define in2 7   // Motor1 L298 Pin in2 
#define in3 8   // Motor2 L298 Pin in3 
#define in4 9   // Motor2 L298 Pin in4 
#define enB 10   // Enable2 L298 Pin enB 
#define echo 2  // Echo pin
#define trigger 4 // Trigger pin

int Set = 35;  // Distance threshold in cm

void setup() {
  Serial.begin(9600);  // start serial communication at 9600bps

  pinMode(echo, INPUT);     // declare ultrasonic sensor Echo pin as input
  pinMode(trigger, OUTPUT); // declare ultrasonic sensor Trigger pin as Output  
  pinMode(enA, OUTPUT); // declare as output for L298 Pin enA 
  pinMode(in1, OUTPUT); // declare as output for L298 Pin in1 
  pinMode(in2, OUTPUT); // declare as output for L298 Pin in2 
  pinMode(in3, OUTPUT); // declare as output for L298 Pin in3   
  pinMode(in4, OUTPUT); // declare as output for L298 Pin in4 
  pinMode(enB, OUTPUT); // declare as output for L298 Pin enB 

  analogWrite(enA, 200); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
  analogWrite(enB, 200); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed 
}

void loop() {
  int distance_F = Ultrasonic_read();
  Serial.print("D F="); Serial.println(distance_F);

  if (distance_F < Set) {
    Stop();
    delay(2000); // Wait for 3 seconds

    turnRight();
    delay(1600); // Turn right for 1 second

    forword();
    delay(2000); // Move forward for 1 second

    turnLeft();
    delay(1600); // Turn left for 1 second

    forword();
    delay(4000); // Move forward for 1 second

    turnLeft();
    delay(1600); // Turn left for 1 second

    forword();
    delay(1500); // Move forward for 1 second

    turnRight();
    delay(1700); // Turn right for 1 second

    forword(); // Continue moving forward
  } else {
    forword(); // Move forward if no obstacle
  }

  delay(10);
}

long Ultrasonic_read() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  long time = pulseIn(echo, HIGH);
  return time / 29 / 2;
}

void forword() {
  digitalWrite(in1, LOW);  // Left Motor backward Pin 
  digitalWrite(in2, HIGH); // Left Motor forward Pin 
  digitalWrite(in3, HIGH); // Right Motor forward Pin 
  digitalWrite(in4, LOW);  // Right Motor backward Pin 
}

void backword() {
  digitalWrite(in1, HIGH); // Left Motor backward Pin 
  digitalWrite(in2, LOW);  // Left Motor forward Pin 
  digitalWrite(in3, LOW);  // Right Motor forward Pin 
  digitalWrite(in4, HIGH); // Right Motor backward Pin 
}

void turnRight() {
  digitalWrite(in1, LOW);  // Left Motor backward Pin 
  digitalWrite(in2, HIGH); // Left Motor forward Pin 
  digitalWrite(in3, LOW);  // Right Motor forward Pin 
  digitalWrite(in4, HIGH); // Right Motor backward Pin 
}

void turnLeft() {
  digitalWrite(in1, HIGH); // Left Motor backward Pin 
  digitalWrite(in2, LOW);  // Left Motor forward Pin 
  digitalWrite(in3, HIGH); // Right Motor forward Pin 
  digitalWrite(in4, LOW);  // Right Motor backward Pin 
}

void Stop() {
  digitalWrite(in1, LOW);  // Left Motor backward Pin 
  digitalWrite(in2, LOW);  // Left Motor forward Pin 
  digitalWrite(in3, LOW);  // Right Motor forward Pin 
  digitalWrite(in4, LOW);  // Right Motor backward Pin 
}
