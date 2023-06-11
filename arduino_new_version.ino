class Sensor {
public:
    int trigger_pin;
    int echo_pin;

    int readout_distance;
    int min_activation_distance;
    int max_activation_distance;

    int relative_readout_factor;

    Sensor(int trigger_pin, int echo_pin, int min_activation_distance, int max_activation_distance)
            : trigger_pin(trigger_pin),
              echo_pin(echo_pin),
              readout_distance(400),
              min_activation_distance(min_activation_distance),
              max_activation_distance(max_activation_distance) {}

    void initialize_default_pin_state() {
        pinMode(trigger_pin, OUTPUT);
        pinMode(echo_pin, INPUT);
    }

    void measure_distance() {
        digitalWrite(trigger_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigger_pin, LOW);

        int readout = pulseIn(echo_pin, HIGH);
        int distance_centimeters = readout / 58;
        this->readout_distance = distance_centimeters;
    }

    void relative_measure_distance() {
        if (this->readout_distance > this->max_activation_distance) {
            this->relative_readout_factor = 0;
        } else {
            double relative_factor = (double) (this->max_activation_distance - this->readout_distance) /
                                     (double) (this->max_activation_distance - this->min_activation_distance);
            this->relative_readout_factor = (int) relative_factor;
        }
    }
};

class Engine {
public:
    int PWM_pin;
    int forward_rotation_pin;
    int backward_rotation_pin;

    Engine(int PWM_pin, int forward_rotation_pin, int backward_rotation_pin)
            : PWM_pin(PWM_pin),
              forward_rotation_pin(forward_rotation_pin),
              backward_rotation_pin(backward_rotation_pin) {
        digitalWrite(this->forward_rotation_pin, LOW);
        digitalWrite(this->backward_rotation_pin, LOW);
    }

    void initialize_default_pin_state() {
        pinMode(PWM_pin, OUTPUT);
        pinMode(forward_rotation_pin, OUTPUT);
        pinMode(backward_rotation_pin, OUTPUT);
    }

    void update_motion_pin_states(int PWM_velocity, int forward_pin_state) {
        analogWrite(PWM_pin, PWM_velocity);
        digitalWrite(forward_rotation_pin, forward_pin_state);
    }
};

Sensor FRONT_SENSOR(22, 23, 20, 50);
Sensor FRONT_LEFT_SENSOR(24, 25, 5, 50);
Sensor FRONT_RIGHT_SENSOR(26, 27, 5, 50);
Sensor REAR_SENSOR(28, 29, 30, 150);

constexpr int NUMBER_OF_SENSORS{4};
Sensor SENSORS[NUMBER_OF_SENSORS]{FRONT_SENSOR, FRONT_LEFT_SENSOR, FRONT_RIGHT_SENSOR, REAR_SENSOR};

constexpr int MAX_ENGINE_VELOCITY{120};
constexpr int DESIRED_PRECISION{12}; //10% max velocity

Engine ENGINE_LEFT(2, 3, 4);
Engine ENGINE_RIGHT(5, 6, 7);

void setup() {
    Serial.begin(115200);

    ENGINE_LEFT.initialize_default_pin_state();
    ENGINE_RIGHT.initialize_default_pin_state();

    for (auto &sensor: SENSORS) {
        sensor.initialize_default_pin_state();
    }
}

int calculate_resultant_velocity_from_sensors(Sensor& sensor, Sensor& sensor2) {
    int resultant_velocity = sensor.relative_readout_factor*MAX_ENGINE_VELOCITY - sensor2.relative_readout_factor*MAX_ENGINE_VELOCITY;
    if (abs(resultant_velocity < DESIRED_PRECISION)) {
        return 0;
    } else {
        return resultant_velocity;
    }
}

void adjust_left_right_motion(int& velocity) {
    if (velocity < 0) {
        ENGINE_LEFT.update_motion_pin_states(velocity, LOW);
        ENGINE_RIGHT.update_motion_pin_states(velocity, HIGH);
    } else {
        ENGINE_LEFT.update_motion_pin_states(velocity, HIGH);
        ENGINE_RIGHT.update_motion_pin_states(velocity, LOW);
    }
}

void hardcode_right_turn_when_obstacle_in_front() {
    //TODO to jest do dopracowania
    ENGINE_LEFT.update_motion_pin_states(MAX_ENGINE_VELOCITY, HIGH);
    ENGINE_RIGHT.update_motion_pin_states(MAX_ENGINE_VELOCITY, LOW);
}

void adjust_front_back_motion(int& velocity) {
    ENGINE_LEFT.update_motion_pin_states(velocity, HIGH);
    ENGINE_RIGHT.update_motion_pin_states(velocity, LOW);
}

void update_engines_motion() {
    int left_right_resultant_velocity = calculate_resultant_velocity_from_sensors( FRONT_LEFT_SENSOR, FRONT_RIGHT_SENSOR);
    if (left_right_resultant_velocity != 0) {
        adjust_left_right_motion(left_right_resultant_velocity);
        return;
    }
    int front_back_resultant_velocity = calculate_resultant_velocity_from_sensors(FRONT_SENSOR, REAR_SENSOR);
    if (front_back_resultant_velocity > 0) {
        hardcode_right_turn_when_obstacle_in_front();
        return;
    }
    adjust_front_back_motion(front_back_resultant_velocity);
}

// moze trzeba ustawiac stan obu silnikom naraz i na tym polega problem?

void loop() {
    for (auto &sensor: SENSORS) {
        sensor.measure_distance();
        Serial.print(sensor.readout_distance);
    }

    update_engines_motion();
}