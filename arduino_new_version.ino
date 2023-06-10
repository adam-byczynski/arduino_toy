class Sensor {
public:
    int trigger_pin;
    int echo_pin;

    int readout_distance;
    int min_activation_distance;
    int max_activation_distance;

    Sensor(int trigger_pin, int echo_pin, int min_activation_distance, int max_activation_distance)
            : trigger_pin(trigger_pin),
              echo_pin(echo_pin),
              readout_distance(100000),
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
        readout_distance = distance_centimeters;
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
              backward_rotation_pin(backward_rotation_pin) {}

    void initialize_default_pin_state() {
        pinMode(PWM_pin, OUTPUT);
        pinMode(forward_rotation_pin, OUTPUT);
        pinMode(backward_rotation_pin, OUTPUT);
    }

    void set_forward_motion(int PWM_velocity) {
        update_motion_pin_states(PWM_velocity, HIGH, LOW);
    }

    void set_backward_motion(int PWM_velocity) {
        update_motion_pin_states(PWM_velocity, LOW, HIGH);
    }

    void turn_off() {
        update_motion_pin_states(0, LOW, LOW);
    }

private:
    void update_motion_pin_states(int PWM_velocity, int forward_pin_state, int backward_pin_state) {
        Serial.print(PWM_velocity);
        Serial.print("\n");
        analogWrite(PWM_pin, PWM_velocity);
        digitalWrite(forward_rotation_pin, forward_pin_state);
        digitalWrite(backward_rotation_pin, backward_pin_state);
    }
};

Sensor FRONT_SENSOR(22, 23, 20, 50);
Sensor FRONT_LEFT_SENSOR(24, 25, 5, 5);
Sensor FRONT_RIGHT_SENSOR(26, 27, 5, 5);
Sensor REAR_SENSOR(28, 29, 30, 150);

constexpr int NUMBER_OF_SENSORS{4};
Sensor SENSORS[NUMBER_OF_SENSORS]{FRONT_SENSOR, FRONT_LEFT_SENSOR, FRONT_RIGHT_SENSOR, REAR_SENSOR};

constexpr int MAX_ENGINE_VELOCITY{120};
constexpr float DESIRED_PRECISION{5.0};

Engine ENGINE_LEFT(2, 3, 4);
Engine ENGINE_RIGHT(5, 6, 7);



int get_converted_velocity(Sensor& sensor) {
    if (sensor.readout_distance > sensor.max_activation_distance) {
        return 0;
    } else {
        int proportional_distance = (sensor.max_activation_distance - sensor.readout_distance) /
                                    (sensor.max_activation_distance - sensor.min_activation_distance);
        return proportional_distance * MAX_ENGINE_VELOCITY;
    }
}

void setup() {
    Serial.begin(115200);

    ENGINE_LEFT.initialize_default_pin_state();
    ENGINE_RIGHT.initialize_default_pin_state();
    for (auto &sensor: SENSORS) {
        sensor.initialize_default_pin_state();
    }
}

void update_sensors_state() {
    for (auto &sensor: SENSORS) {
        sensor.measure_distance();
    }
}

int calculate_resultant_velocity_from_sensors(Sensor& sensor, Sensor& sensor2) {
    int resultant_velocity = get_converted_velocity(sensor) - get_converted_velocity(sensor2);
    if (abs(resultant_velocity < DESIRED_PRECISION)) {
        return 0;
    } else {
        return resultant_velocity;
    }
}

void adjust_left_right_motion(int velocity) {
    if (velocity < 0) {
        ENGINE_LEFT.turn_off();
        ENGINE_RIGHT.set_forward_motion(velocity);
    } else {
        ENGINE_LEFT.set_forward_motion(velocity);
        ENGINE_RIGHT.turn_off();
    }
}

void hardcode_right_turn_when_obstacle_in_front() {
    //TODO to jest do dopracowania
    ENGINE_LEFT.set_forward_motion(MAX_ENGINE_VELOCITY);
    ENGINE_RIGHT.turn_off();
}

void adjust_front_back_motion(int velocity) {
    ENGINE_LEFT.set_forward_motion(velocity);
    ENGINE_RIGHT.set_forward_motion(velocity);
}

void update_engines_motion() {
    int left_right_resultant_velocity = calculate_resultant_velocity_from_sensors( FRONT_LEFT_SENSOR, FRONT_RIGHT_SENSOR);
    if (left_right_resultant_velocity != 0) {
        adjust_left_right_motion(left_right_resultant_velocity);
        return;
    }
    int front_back_resultant_velocity = calculate_resultant_velocity_from_sensors(FRONT_SENSOR, REAR_SENSOR);
    if (front_back_resultant_velocity >= 0) {
        hardcode_right_turn_when_obstacle_in_front();
        return;
    }
    adjust_front_back_motion(front_back_resultant_velocity);
}

void update_vehicle_position() {
    update_sensors_state();
    update_engines_motion();
}

void loop() {
    update_vehicle_position();
}