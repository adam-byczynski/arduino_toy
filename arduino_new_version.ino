static const int MAX_ENGINE_VELOCITY{120};
static const int ACTIVATION_MODE_FULL {2};
static const int ACTIVATION_MODE_HALF {1};
static const int ACTIVATION_MODE_OFF {0};

static const int DEFAULT_UNIT_OF_TIME_microseconds {100};


class Sensor {
public:
    int trigger_pin;
    int echo_pin;

    int readout_distance;
    int activation_mode;

    int min_activation_distance;
    int max_activation_distance;

    Sensor(int trigger_pin, int echo_pin, int min_activation_distance, int max_activation_distance)
            : trigger_pin(trigger_pin),
              echo_pin(echo_pin),
              readout_distance(100000),
              min_activation_distance(min_activation_distance),
              max_activation_distance(max_activation_distance)
    {}

    void initialize_default_pin_state() {
        pinMode(this->trigger_pin, OUTPUT);
        pinMode(this->echo_pin, INPUT);
    }

    void measure_distance() {
        digitalWrite(this->trigger_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(this->trigger_pin, LOW);

        int readout = pulseIn(this->echo_pin, HIGH);
        int distance_centimeters = readout / 58;
        this->readout_distance = distance_centimeters;
    }

    void set_activation_mode(){
        if(this->readout_distance <= this->min_activation_distance){
            this->activation_mode = ACTIVATION_MODE_FULL;
        }
        else if(this->readout_distance <= max_activation_distance &&
                this->readout_distance > this->min_activation_distance){
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

    void initialize_default_pin_state(){
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

    void turn_off() {
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
Sensor FRONT_SENSOR(22, 23,
                    20, 50);
Sensor FRONT_LEFT_SENSOR(24, 25,
                         5, 5);
Sensor FRONT_RIGHT_SENSOR(26, 27,
                          5, 5);
Sensor REAR_SENSOR(28, 29,
                   30, 150);
Sensor SENSORS[NUMBER_OF_SENSORS] {FRONT_SENSOR, FRONT_LEFT_SENSOR, FRONT_RIGHT_SENSOR, REAR_SENSOR};

Engine ENGINE_LEFT(2, 3, 4);
Engine ENGINE_RIGHT(5, 6, 7);




void print_values() {
    for (int i = 0; i < NUMBER_OF_SENSORS; i++) {
        Serial.print("Pin: ");
        Serial.print(i);
        Serial.print(" Distance: ");
        Serial.print(SENSORS[i].readout_distance);
        Serial.print("\n");
    }
}

int convert_sensor_readout_to_engine_velocity(Sensor sensor){
    int commensurate_distance = (sensor.max_activation_distance - sensor.readout_distance) /
                                (sensor.max_activation_distance - sensor.min_activation_distance);
    int commensurate_velocity = commensurate_distance * MAX_ENGINE_VELOCITY;
    return commensurate_velocity;
}

void setup() {
    Serial.begin(115200);

    ENGINE_LEFT.initialize_default_pin_state();
    ENGINE_RIGHT.initialize_default_pin_state();
    for(auto sensor : SENSORS){
        sensor.initialize_default_pin_state();
    }
}


void update_sensors_activation_mode() {
    for(auto sensor : SENSORS){
        sensor.measure_distance();
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
            ENGINE_RIGHT.turn_off();
            //tutaj chyba delay by było ze pojazd skręca przez jakiś czas, tak by uzyskał np kąt prosty
            delayMicroseconds(500);
        }
    }
    switch(REAR_SENSOR.activation_mode){
        case ACTIVATION_MODE_FULL:
            ENGINE_LEFT.set_forward_motion(MAX_ENGINE_VELOCITY);
            ENGINE_RIGHT.set_forward_motion(MAX_ENGINE_VELOCITY);
        case ACTIVATION_MODE_HALF:
            ENGINE_LEFT.set_forward_motion(MAX_ENGINE_VELOCITY / 2);
            ENGINE_RIGHT.set_forward_motion(MAX_ENGINE_VELOCITY / 2);
        case ACTIVATION_MODE_OFF:
            ENGINE_LEFT.turn_off();
            ENGINE_RIGHT.turn_off();
    }

    if(FRONT_LEFT_SENSOR.activation_mode == ACTIVATION_MODE_FULL)
    {
        ENGINE_LEFT.set_forward_motion(MAX_ENGINE_VELOCITY);
        ENGINE_RIGHT.set_backward_motion(MAX_ENGINE_VELOCITY);
    }
    else if(FRONT_RIGHT_SENSOR.activation_mode == ACTIVATION_MODE_FULL){
        ENGINE_LEFT.set_backward_motion(MAX_ENGINE_VELOCITY);
        ENGINE_RIGHT.set_forward_motion(MAX_ENGINE_VELOCITY);
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

// TODO lepiej obliczyć wypadkową wartość przed konwersją dystansu na prędkość, ale łatwiej najpierw zrobić konwersje

//TODO dodac wskazniki zamiast kopiowania wartosci, i może inne rzeczy jak const itp
