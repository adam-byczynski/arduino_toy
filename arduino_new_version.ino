static const int MAX_ENGINE_VELOCITY{120};
static const int ACTIVATION_MODE_FULL {2};
static const int ACTIVATION_MODE_HALF {1};
static const int ACTIVATION_MODE_OFF {0};

static const int DEFAULT_UNIT_OF_TIME_LENGTH_microseconds {100};


class UltrasonicSensor {
public:
    int trigger_pin;
    int echo_pin;

    int readout_value;
    int activation_mode;

    int min_activation_distance;
    int max_activation_distance;

    UltrasonicSensor(int trigger_pin, int echo_pin, int min_activation_distance, int max_activation_distance)
            : trigger_pin(trigger_pin),
              echo_pin(echo_pin),
              readout_value(100000),
              min_activation_distance(min_activation_distance),
              max_activation_distance(max_activation_distance)
    {}

    void apply_default_pin_state() {
        pinMode(this->trigger_pin, OUTPUT);
        pinMode(this->echo_pin, INPUT);
    }

    void set_activation_mode(){
        if(this->readout_value <= this->min_activation_distance){
            this->activation_mode = ACTIVATION_MODE_FULL;
        }
        else if(this->readout_value <= max_activation_distance && this->readout_value > this->min_activation_distance){
            this->activation_mode = ACTIVATION_MODE_HALF;
        }
        else
            this->activation_mode = ACTIVATION_MODE_OFF;
    }
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
    {}

    void apply_default_pin_state(){
        this->forward_motion_pin_state(LOW);
        this->backward_motion_pin_state(LOW);
        pinMode(this->PWM_pin, OUTPUT);
        pinMode(this->forward_rotation_pin, OUTPUT);
        pinMode(this->backward_rotation_pin, OUTPUT);
    }
    void set_forward_motion(int PWM_velocity) {
        update_motion_pin_states(PWM_velocity, HIGH, LOW);
    }

    void set_backward_motion(int PWM_velocity) {
        update_motion_pin_states(PWM_velocity, LOW, HIGH);
    }

    void turn_engine_off() {
        update_motion_pin_states(0, LOW, LOW);
    }
private:
    void update_motion_pin_states(int PWM_velocity, int forward_pin_state, int backward_pin_state) {
        analogWrite(this->PWM_pin, PWM_velocity);

        this->forward_motion_pin_state = forward_pin_state;
        this->backward_motion_pin_state = backward_pin_state;

        digitalWrite(this->forward_rotation_pin, this->forward_motion_pin_state);
        digitalWrite(this->backward_rotation_pin, this->backward_motion_pin_state);
    }

};

static const int NUMBER_OF_SENSORS {4};
UltrasonicSensor FRONT_SENSOR(22,23,
                              20, 50);
UltrasonicSensor FRONT_LEFT_SENSOR(24, 25,
                                   5, 5);
UltrasonicSensor FRONT_RIGHT_SENSOR(26, 27,
                                    5, 5);
UltrasonicSensor REAR_SENSOR(28, 29,
                             30, 150);
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

//int convert_sensor_readout_to_engine_velocity(int sensor_readout){
//    int current_distance_relatively_to_max_distance = (DEFAULT_SENSOR_ACTIVATION_MIN_DISTANCE - sensor_readout) /
//                                                      (DEFAULT_SENSOR_ACTIVATION_MIN_DISTANCE - DEFAULT_SENSOR_ACTIVATION_MAX_DISTANCE);
//    int result_velocity = current_distance_relatively_to_max_distance * MAX_ENGINE_VELOCITY;
//    return result_velocity;
//}

void apply_sensors_default_state(){
    for(auto sensor : SENSORS){
        sensor.apply_default_pin_state();
    }
}

void setup() {
    Serial.begin(115200);

    ENGINE_LEFT.apply_default_pin_state();
    ENGINE_RIGHT.apply_default_pin_state();
    apply_sensors_default_state();
}


void update_sensors_activation_mode() {
    for(auto sensor : SENSORS){
        sensor.readout_value = measure_sensor_distance(sensor);
        sensor.set_activation_mode();
    }
}

void update_engines_motion() {
    if(FRONT_SENSOR.activation_mode == ACTIVATION_MODE_FULL){
        // HARDCODED skręt w prawo
        if(REAR_SENSOR.activation_mode == ACTIVATION_MODE_FULL || REAR_SENSOR.activation_mode == ACTIVATION_MODE_HALF) {
            // tutaj skręt w prawo, czyli uruchomienie lewego silnika na full a zatrzymanie prawego
            // wywalic do osobnej funkcji, jako hardcoded skret w prawo?
            ENGINE_LEFT.set_forward_motion(MAX_ENGINE_VELOCITY);
            ENGINE_RIGHT.turn_engine_off();
            //tutaj chyba delay by było ze pojazd skręca przez jakiś czas, tak by uzyskał np kąt prosty
            delayMicroseconds(500);
        }
    }
    // aktywacja tylnego sensora -> jazda do przodu, TODO wyjebac do osobnej funkcji
    switch(REAR_SENSOR.activation_mode){
        case ACTIVATION_MODE_FULL:
            ENGINE_LEFT.set_forward_motion(MAX_ENGINE_VELOCITY);
            ENGINE_RIGHT.set_forward_motion(MAX_ENGINE_VELOCITY);
        case ACTIVATION_MODE_HALF:
            ENGINE_LEFT.set_forward_motion(MAX_ENGINE_VELOCITY / 2);
            ENGINE_RIGHT.set_forward_motion(MAX_ENGINE_VELOCITY / 2);
        case ACTIVATION_MODE_OFF:
            ENGINE_LEFT.turn_engine_off();
            ENGINE_RIGHT.turn_engine_off();
    }
    /* 3 sytuacje ->
     1. lewy sensor wykrywa więc ustawia backward pin prawego silnika na HIGH co spowoduje ze prawy silnik sie zatrzyma
     i pojazd skreci w prawo
     2. odwrotna sytuacja
     3. oba wykrywają ->
    */
}

void update_vehicle_position() {
    update_sensors_activation_mode();
    update_engines_motion();
}

void loop() {
    update_vehicle_position();
}


//TODO dodac wskazniki zamiast kopiowania wartosci, i może inne rzeczy jak const itp
// wyliczanie predkosci wypadkowej poza klasą i potem w klasie tylko update prędkości