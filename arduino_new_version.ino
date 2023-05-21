static const int MAX_ENGINE_VELOCITY{120};
static const int SENSOR_ACTIVATION_MIN_DISTANCE{30};
static const int SENSOR_ACTIVATION_MAX_DISTANCE {10};

class UltrasonicSensor {
public:
    UltrasonicSensor(int trigger_pin, int echo_pin)
            : trigger_pin(trigger_pin),
              echo_pin(echo_pin),
              readout_value(10000)
    {
        pinMode(trigger_pin, OUTPUT);
        pinMode(echo_pin, INPUT);
    }
    int trigger_pin;
    int echo_pin;
    int readout_value;
};

class Engine {
public:
    int PWM_pin;
    int forward_rotation_pin;
    int backward_rotation_pin;

    int forward_motion_pin_state;
    int backward_motion_pin_state;

    Engine(int PWM_pin, int forward_rotation_pin, int backward_rotation_pin)
            : PWM_pin(PWM_pin),
              forward_rotation_pin(forward_rotation_pin),
              backward_rotation_pin(backward_rotation_pin),
              forward_motion_pin_state(LOW),
              backward_motion_pin_state(LOW)
    {
        apply_default_state_to_pins();
    }

    void update_motion_pin_states(int forward_pin_state, int backward_pin_state, int PWM_velocity) {
        analogWrite(this->PWM_pin, PWM_velocity);

        this->forward_motion_pin_state = forward_pin_state;
        this->backward_motion_pin_state = backward_pin_state;

        digitalWrite(this->forward_rotation_pin, this->forward_motion_pin_state);
        digitalWrite(this->backward_rotation_pin, this->backward_motion_pin_state);
    }

    void apply_default_state_to_pins() {
        pinMode(this->PWM_pin, OUTPUT);
        pinMode(this->forward_rotation_pin, OUTPUT);
        pinMode(this->backward_rotation_pin, OUTPUT);
    }

    void set_forward_motion() {
        update_motion_pin_states(HIGH, LOW);
    }

    void set_backward_motion() {
        update_motion_pin_states(LOW, HIGH);
    }

    void turn_engine_off() {
        update_motion_pin_states(LOW, LOW);
    }
};

static const int NUMBER_OF_SENSORS {4};
UltrasonicSensor FRONT_SENSOR(22, 23);
UltrasonicSensor FRONT_LEFT_SENSOR(24, 25);
UltrasonicSensor FRONT_RIGHT_SENSOR(26, 27);
UltrasonicSensor REAR_SENSOR(28, 29);
UltrasonicSensor SENSORS[NUMBER_OF_SENSORS] {FRONT_SENSOR, FRONT_LEFT_SENSOR, FRONT_RIGHT_SENSOR, REAR_SENSOR};

Engine ENGINE_LEFT(2, 3, 4);
Engine ENGINE_RIGHT(5, 6, 7);


int measure_sensor_distance(UltrasonicSensor sensor) {
    digitalWrite(sensor.trigger_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensor.trigger_pin, LOW);

    int readout = pulseIn(sensor.echo_pin, HIGH);
    int distance = readout / 58;
    return distance;
}

void print_values() {
    for (int i = 0; i < sizeof(SENSORS) / sizeof(*SENSORS); i++) {
        Serial.print("Pin: ");
        Serial.print(i);
        Serial.print(" Distance: ");
        Serial.print(measure_sensor_distance(SENSORS[i]));
        Serial.print("\n");
    }
}

int convert_sensor_readout_to_engine_velocity(int sensor_readout){
    int current_distance_relatively_to_max_distance = (SENSOR_ACTIVATION_MIN_DISTANCE - sensor_readout) /
                                                      (SENSOR_ACTIVATION_MIN_DISTANCE - SENSOR_ACTIVATION_MAX_DISTANCE);
    int result_velocity = current_distance_relatively_to_max_distance * MAX_ENGINE_VELOCITY;
    return result_velocity;
}

void setup() {
    Serial.begin(115200);
}

void update_sensors_readout() {
    for(auto sensor : SENSORS){
        sensor.readout_value = measure_sensor_distance(sensor);
    }
}

void update_engines_motion() {
    //TODO logika ruchu
}

void calculate_resultant_velocity() {
    //TODO obliczenie predkosci wypadkowej
}

void update_position() {
    update_sensors_readout();
    calculate_resultant_velocity();
    update_engines_motion();
}

void loop() {
    update_position();
}


//TODO dodac wskazniki zamiast kopiowania wartosci, i może inne rzeczy jak const itp
// wyliczanie predkosci wypadkowej poza klasą i potem w klasie tylko update prędkości