class Sensor {
public:
    int trigger_pin;
    int echo_pin;

    int readout_distance;
    int min_activation_distance;
    int max_activation_distance;

    double relative_readout_factor;

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

    int measure_distance() {
        digitalWrite(trigger_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigger_pin, LOW);

        int readout = pulseIn(echo_pin, HIGH);
        int distance_centimeters = readout / 58;
        return distance_centimeters;
    }

    double relative_measure_distance() {
        int local_readout_distance = this->measure_distance();
        double factor;
        if (local_readout_distance > this->max_activation_distance) {
            factor = 0;
        }
        else if (local_readout_distance <= this->min_activation_distance) {
            factor = 1;
        }
        else {
            double relative_factor = (double) (this->max_activation_distance - local_readout_distance) /
                                     (double) (this->max_activation_distance - this->min_activation_distance);
            factor = relative_factor;
        }
        return factor;
    }
};

class Engines {
public:
    int PWM_pin1;
    int forward_rotation_pin1;
    int backward_rotation_pin1;

    int PWM_pin2;
    int forward_rotation_pin2;
    int backward_rotation_pin2;

    Engines(int PWM_pin1, int forward_rotation_pin1, int backward_rotation_pin1,
            int PWM_pin2, int forward_rotation_pin2, int backward_rotation_pin2)
            : PWM_pin1(PWM_pin1),
              forward_rotation_pin1(forward_rotation_pin1),
              backward_rotation_pin1(backward_rotation_pin1),
              PWM_pin2(PWM_pin2),
              forward_rotation_pin2(forward_rotation_pin2),
              backward_rotation_pin2(backward_rotation_pin2)
    {}

    void initialize_default_pin_state() {
        pinMode(PWM_pin1, OUTPUT);
        pinMode(forward_rotation_pin1, OUTPUT);
        pinMode(backward_rotation_pin1, OUTPUT);
        digitalWrite(this->forward_rotation_pin1, LOW);
        digitalWrite(this->backward_rotation_pin1, LOW);

        pinMode(PWM_pin2, OUTPUT);
        pinMode(forward_rotation_pin2, OUTPUT);
        pinMode(backward_rotation_pin2, OUTPUT);
        digitalWrite(this->forward_rotation_pin2, LOW);
        digitalWrite(this->backward_rotation_pin2, LOW);
    }

    void update_motion_pin_states(int PWM_velocity, int forward_pin_state1, int forward_pin_state2) {
        analogWrite(PWM_pin1, PWM_velocity);
        digitalWrite(forward_rotation_pin1, forward_pin_state1);

        analogWrite(PWM_pin2, PWM_velocity);
        digitalWrite(forward_rotation_pin2, forward_pin_state2);
    }
};

Sensor FRONT_SENSOR(22, 23, 20, 50);
Sensor FRONT_LEFT_SENSOR(24, 25, 5, 50);
Sensor FRONT_RIGHT_SENSOR(26, 27, 5, 50);
Sensor REAR_SENSOR(28, 29, 30, 150);

constexpr int NUMBER_OF_SENSORS{4};
Sensor SENSORS[NUMBER_OF_SENSORS]{FRONT_SENSOR, FRONT_LEFT_SENSOR, FRONT_RIGHT_SENSOR, REAR_SENSOR};

constexpr int MAX_ENGINE_VELOCITY{150};
constexpr int DESIRED_PRECISION{int(0.1 * MAX_ENGINE_VELOCITY)}; //10% of MAX_ENGINE_VELOCITY

Engines ENGINES(2, 3, 4,
                5, 6, 7);

void setup() {
    Serial.begin(115200);

    ENGINES.initialize_default_pin_state();

    for (auto &sensor: SENSORS) {
        sensor.initialize_default_pin_state();
    }
}

int calculate_resultant_velocity_from_sensors(double factor1, double factor2) {
    int resultant_velocity = MAX_ENGINE_VELOCITY * (factor1 - factor2);
    abs(resultant_velocity) < DESIRED_PRECISION ? resultant_velocity = 0 : resultant_velocity;
    return resultant_velocity;
}

void adjust_left_right_motion(int& velocity) {
    if (velocity < 0) {
        ENGINES.update_motion_pin_states(velocity, LOW, HIGH);
    } else {
        ENGINES.update_motion_pin_states(velocity, HIGH, LOW);
    }
}

void hardcode_right_turn_when_obstacle_in_front() {
    ENGINES.update_motion_pin_states(MAX_ENGINE_VELOCITY, HIGH, LOW);
}

void adjust_front_back_motion(int& velocity) {
    ENGINES.update_motion_pin_states(velocity, HIGH, HIGH);
}

void loop() {
    double factors[4];
    for(int index=0; index<4; index++){
        factors[index] = SENSORS[index].relative_measure_distance();
        Serial.print(factors[index]);
        Serial.print("\n");
    }


    int left_right_resultant_velocity = calculate_resultant_velocity_from_sensors(factors[1], factors[2]);
    int front_back_resultant_velocity = calculate_resultant_velocity_from_sensors(factors[0], factors[3]);
    Serial.print("front_back_resultant_velocity: ");
    Serial.print(front_back_resultant_velocity);
    Serial.print("\n");
    Serial.print("left_right_resultant_velocity: ");
    Serial.print(left_right_resultant_velocity);
    Serial.print("\n");
    if (left_right_resultant_velocity != 0) {
        adjust_left_right_motion(left_right_resultant_velocity);
    }
    else if(front_back_resultant_velocity > 0) {
        hardcode_right_turn_when_obstacle_in_front();
    }
    else adjust_front_back_motion(front_back_resultant_velocity);

    delay(5000);
}

