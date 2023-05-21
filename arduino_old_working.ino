int engine_one_PWM_pin {2};
int engine_one_rotation_pin_1 {3};
int engine_one_rotation_pin_2 {4};

int engine_two_PWM_pin {5};
int engine_two_rotation_pin_1 {6};
int engine_two_rotation_pin_2 {7};


int number_of_sensors {4}; // Z TYM COS NIE TAK

int ultrasonic_sensor_trigger_pins [4] {22, 24, 26, 28};
int ultrasonic_sensor_echo_pins [4] {23, 25, 27, 29};

int max_velocity {120};
int sensor_activation_distance = 20;

int measure_sensor_distance(int trigger_pin, int echo_pin){
  digitalWrite(trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pin, LOW);
  
  int readout = pulseIn(echo_pin, HIGH);
  int distance = readout / 58;
  return distance;
}

void print_values(){
  for(int i=0; i<number_of_sensors; i++) {
    Serial.print("Pin: ");
    Serial.print(i);
    Serial.print(" Distance: ");
    Serial.print(measure_sensor_distance(ultrasonic_sensor_trigger_pins[i], ultrasonic_sensor_echo_pins[i]));
    Serial.print("\n");
  }
}

void setup_engines(){
  pinMode(engine_one_PWM_pin, OUTPUT);
  pinMode(engine_two_PWM_pin, OUTPUT);
  pinMode(engine_one_rotation_pin_1, OUTPUT);
  pinMode(engine_one_rotation_pin_2, OUTPUT);
  pinMode(engine_two_rotation_pin_1, OUTPUT);
  pinMode(engine_two_rotation_pin_2, OUTPUT);

  digitalWrite(engine_one_rotation_pin_1, LOW);
  digitalWrite(engine_one_rotation_pin_2, LOW);
  digitalWrite(engine_two_rotation_pin_1, LOW);
  digitalWrite(engine_two_rotation_pin_2, LOW);
}

void launch_engines(int state){
  analogWrite(engine_one_PWM_pin, max_velocity);
  analogWrite(engine_two_PWM_pin, max_velocity);

  digitalWrite(engine_one_rotation_pin_1, state);
  digitalWrite(engine_two_rotation_pin_1, state);
}

void setup() {
  Serial.begin(115200);

  for(auto pin : ultrasonic_sensor_trigger_pins){
    pinMode(pin, OUTPUT);
  }
  
  for(auto pin: ultrasonic_sensor_echo_pins){
    pinMode(pin, INPUT);
  }
  
 setup_engines();

}


void loop() {
  int sensor_result = measure_sensor_distance(ultrasonic_sensor_trigger_pins[3], ultrasonic_sensor_echo_pins[3]);

  if(sensor_result <= sensor_activation_distance){
    launch_engines(HIGH);
  }
  else {
    launch_engines(LOW);
  }
}


