#include "Adafruit_BNO055.h"
#include "colour_sensor.h"
#include "Config.h"
#include "LRFs.h"
#include "Motors.h"
#include "VL53L4CD.h"
#include "Servo.h"
#include <Arduino.h>
#include "mapping.h"

//////////////////////////////////// Objects ///////////////////////////////////

Motors motor;
LRFs lrfs;
ColourSensor colour;
Servo dropper_servo;
Adafruit_BNO055 bno(55, BNO055_ADDRESS_B, &Wire1);
Map maze;

////////////////////////////////////// FSM /////////////////////////////////////

typedef enum {
    FORWARD,
    ROTATE_L,
    ROTATE_R,
    ROTATE_180,
    NAV,
    BT_BACK,
    BT_PAUSE,
    BT_ROTATE,
    VICTIMS,
    END,
    PAUSE,
    VIC_PAUSE,
    VIC_MOVE,
} State;

/////////////////////////////// Global Variables ///////////////////////////////

uint8_t state;
uint16_t target_dist;

float target_bearing = 0.0f;
float current_bearing = 0.0f;
float bearing_adjustment = 0.0f;
unsigned long pause_start = millis();

uint8_t harmed_victim_cond;
uint8_t unharmed_victim_cond;
uint8_t silver_tile_cond;
uint8_t harmed_victim_counter;
uint8_t unharmed_victim_counter;

////////////////////////////// Function Prototypes /////////////////////////////

void forward();
void rotate_left();
void rotate_right();
void rotate_180();
void navigation();
void black_tile_backwards();
void black_tile_pause();
void black_tile_rotate();
void victims();
void end();
void pause();
void dropper_left();
void dropper_right();
void vic_pause();
void vic_move();
float wrap_angle(float angle);
String stateToName(int st);

/////////////////////////////////// Functions //////////////////////////////////

void setup()
{
    delay(2000);
    pause_start = millis();
    motor.init();
    lrfs.init();
    colour.init();
    maze.init();
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(DROPPER, OUTPUT);

    dropper_servo.attach(DROPPER);
    dropper_servo.write(90);

    while(!bno.begin(OPERATION_MODE_IMUPLUS)) {
        Serial.println("No BNO055 detected. Check your wiring or I2C ADDR.");
        delay(1000);
    }

    // Maze setup
    state = NAV;
}

void loop()
{
    static sensors_event_t event;

    lrfs.update();
    // lrfs.test_lrf(); //new test function because I am sick of changing the library
    // colour.update();
    bno.getEvent(&event);

    current_bearing = event.orientation.x;
    current_bearing = wrap_angle(current_bearing);

    // Serial.println(stateToName(state));

    // FSM
    switch (state) {
    case FORWARD:
        forward();
        break;
    case ROTATE_L:
        rotate_left();
        break;
    case ROTATE_R:
        rotate_right();
        break;
    case ROTATE_180:
        rotate_180();
        break;
    case NAV:
        navigation();
        break;
    case BT_BACK:
        black_tile_backwards();
        break;
    case BT_PAUSE:
        black_tile_pause();
        break;
    case BT_ROTATE:
        black_tile_rotate();
        break;
    case VICTIMS:
        victims();
        break;
    case END:
        end();
        break;
    case PAUSE:
        pause();
        break;
    case VIC_PAUSE:
        vic_pause();
        break;
    case VIC_MOVE:
        vic_move();
        break;
    default:
        state = NAV;
        break;
    }
}

void forward()
{
    // TRANSITIONS TO DIFFERENT STATE
    uint16_t front = (lrfs.get_value(LRF_FL) + lrfs.get_value(LRF_FR)) / 2;
    uint16_t back = (lrfs.get_value(LRF_BL) + lrfs.get_value(LRF_BR)) / 2;

    if (colour.detect_black()) {
        state = BT_BACK;
        maze.found_black_tile();
        harmed_victim_cond = 0;
        unharmed_victim_cond = 0;
        return;
    }

    if (front < MIN_TARGET_DIST) {
        pause_start = millis();
        state = PAUSE;
        return;
    }
    
    //LRF use
    if (front < target_dist) {
        pause_start = millis();
        state = PAUSE;
        return;
    }

    uint16_t leftlf = lrfs.get_value(LRF_LF);
    uint16_t leftlb = lrfs.get_value(LRF_LB);

    uint16_t left;
    if ((leftlf < CHECK_WALL) && (leftlb < CHECK_WALL)) {
         left = (leftlf + leftlb) / 2;
    } else if (leftlf < CHECK_WALL) {
        left = leftlf;
    } else if (leftlb < CHECK_WALL) {
        left = leftlb;
    } else {
        left = IMAG_WALL_DIST;
    }

    uint16_t rightlf = lrfs.get_value(LRF_RF);
    uint16_t rightlb = lrfs.get_value(LRF_RB);

    uint16_t right;
    if ((rightlf < CHECK_WALL) && (rightlb < CHECK_WALL)) {
        right = (rightlf + rightlb) / 2;
    } else if (rightlf < CHECK_WALL) {
        right = rightlf;
    } else if (rightlb < CHECK_WALL) {
        right = rightlb;
    } else {
        right = IMAG_WALL_DIST;
    }

    // CENTERING AND ALIGNMENT CODE
    int16_t error = right - left;
    bearing_adjustment = atanf((float)error / (float)TILE_DIST) * RAD_TO_DEG;
    
    if (bearing_adjustment > 20.0f) {
        bearing_adjustment = 20.0f;
    } else if (bearing_adjustment < -20.0f) {
        bearing_adjustment = -20.0f;
    }

    float new_bearing = wrap_angle(target_bearing + bearing_adjustment);

    float bearing_error = wrap_angle(new_bearing - current_bearing);
    
    Serial.print("Error: ");
    Serial.print(error); 
    Serial.print("\t");
    Serial.print("Bearing Adj.: ");
    Serial.print(bearing_adjustment); 
    Serial.print("\t");
    Serial.print("current bearing: ");
    Serial.print(current_bearing); 
    Serial.print("\t");
    Serial.print("target bearing: ");
    Serial.print(new_bearing);
    Serial.print("\t");
    Serial.print("bearing error: ");
    Serial.print(bearing_error); 
    Serial.print("\t");
    Serial.println();

    float correction = bearing_error * BEARING_KP;

    // ACTUAL LINE THAT MOVES MOTORS
    motor.move(MOVE_SPEED - correction, MOVE_SPEED + correction);
}

void rotate_left()
{
    if (fabs(current_bearing - target_bearing) < ROTATION_MARGIN) {
            state = FORWARD;
    
    } else {
        motor.move(-ROTATE_SPEED, ROTATE_SPEED);
    }
}

void rotate_right()
{
    if (fabs(current_bearing - target_bearing) < ROTATION_MARGIN) {
            state = FORWARD;

        
    } else {
        motor.move(ROTATE_SPEED, -ROTATE_SPEED);
    }
}

void rotate_180()
{
    if (fabs(current_bearing - target_bearing) < ROTATION_MARGIN) {
            state = FORWARD;

    } else {
        motor.move(ROTATE_SPEED, -ROTATE_SPEED);
    }

}

void navigation()
{
    uint16_t front = (lrfs.get_value(LRF_FL) + lrfs.get_value(LRF_FR)) / 2;
    uint16_t right = (lrfs.get_value(LRF_RF) + lrfs.get_value(LRF_RB)) / 2;
    uint16_t back = (lrfs.get_value(LRF_BL) + lrfs.get_value(LRF_BR)) / 2;
    uint16_t left = (lrfs.get_value(LRF_LB) + lrfs.get_value(LRF_LF)) / 2;

    maze.update(front, right, back, left);
    uint8_t next_state = maze.navigate();
    
    switch(next_state) {
        case(state_forward):
            state = FORWARD;
            break;

        case(state_left):
            target_bearing -= 90.0f;
            state = ROTATE_L;
            break;

        case(state_right):
            target_bearing += 90.0f;
            state = ROTATE_R;
            break;

        case(state_180):
            target_bearing -= 180.0f;
            state = ROTATE_180;
            break;
        }

    target_bearing = wrap_angle(target_bearing);
}

void black_tile_backwards()
{

    uint16_t front = (lrfs.get_value(LRF_FL) + lrfs.get_value(LRF_FR)) / 2;
    uint16_t back = (lrfs.get_value(LRF_BL) + lrfs.get_value(LRF_BR)) / 2;
    
    if (front > (target_dist + TILE_DIST)) {
        pause_start = millis();
        state = BT_PAUSE;
        return;
     }


    motor.move(-MOVE_SPEED, -MOVE_SPEED);
}

void black_tile_pause()
{
    if ((millis() - pause_start) > 250) {
        state = BT_ROTATE;
        target_bearing -= 180.0f;
        if (target_bearing <= -180.0f) { 
            target_bearing += 360.0f;
        }
    } else {
        motor.move(0.0f, 0.0f); //NOTE CHANGE THIS TO BEFORE IF STATEMENT FOR EFFICIENCY - Rory
    }
}

void black_tile_rotate()
{
    if (fabs(current_bearing - target_bearing) < ROTATION_MARGIN) {
        state = NAV;
    } else {
        motor.move(ROTATE_SPEED, -ROTATE_SPEED);
    }
}

void victims()
{   
    motor.move(0.0f, 0.0f);
    uint8_t vic_seen_bef = maze.map_victim();
    uint16_t right = (lrfs.get_value(LRF_RF) + lrfs.get_value(LRF_RB)) / 2;
    
    switch(vic_seen_bef) {
    case(NEW_VIC):
        if (harmed_victim_cond) {
            harmed_victim_cond = 0;
            harmed_victim_counter++;
            digitalWrite(LED_RED, HIGH);
            delay(3000);
            digitalWrite(LED_RED, LOW);

            if (right < TARGET_WALL_DIST) {
                dropper_right();
            } else {
                dropper_left();
            }
            state = VIC_MOVE;

        } else if (unharmed_victim_cond) {
            unharmed_victim_cond = 0;
            unharmed_victim_counter++;
            digitalWrite(LED_GREEN, HIGH);
            delay(3000);
            digitalWrite(LED_GREEN, LOW);
            state = VIC_MOVE;
        }
        maze.victim_update();
        break;

    case(SEEN_VIC):
        unharmed_victim_cond = 0;
        harmed_victim_cond = 0;
        state = FORWARD;
        break;
    }
}

void end()
{
    motor.move(0.0f, 0.0f);
    uint8_t i;
    for (i = 0; i < (harmed_victim_counter + 1); i++) {
        digitalWrite(LED_RED, HIGH);
        delay(1000);
        digitalWrite(LED_RED, LOW);
        delay (500);
    }

    for (i = 0; i < (unharmed_victim_counter + 1); i++) {
        digitalWrite(LED_GREEN, HIGH);
        delay(1000);
        digitalWrite(LED_GREEN, LOW);
        delay(500);
    }
    delay(60000);
    state = END;
    return;
}

void pause()
{
    if ((millis() - pause_start) > 250) {
        if (silver_tile_cond) {
            state = END;
        } else if (unharmed_victim_cond || harmed_victim_cond) {
            state = VICTIMS;
        
        } else {
            state = NAV;
        }
    } else {
        motor.move(0.0f, 0.0f);
    }
}

void vic_pause() {

    if ((millis() - pause_start) > 250) {
        if (colour.detect_red()) {
            harmed_victim_cond = 1;
            unharmed_victim_cond = 0;
            state = VICTIMS;
            return;
        } else if (colour.detect_green()) {
            unharmed_victim_cond = 1;
            harmed_victim_cond = 0;
            state = VICTIMS;
            return;
        } else if (colour.detect_black()) {
            state = BT_BACK;
            maze.found_black_tile();
            harmed_victim_cond = 0;
            unharmed_victim_cond = 0;
            silver_tile_cond = 0;
            state = BT_BACK;
            return;
        }
    } else {
        motor.move(0.0f, 0.0f);
    }
    
}

void dropper_left() {
    //90 is centre, 180 is right, 0 is left.
    delay(100);
    dropper_servo.write(160); //turns motor right (drops single package)
    delay(1000);
    dropper_servo.write(80); // Pushes package outside
    delay(1000);
    dropper_servo.write(90); //returns to original position
}

void dropper_right() {
    delay(100);
    dropper_servo.write(20); //turns motor left (drops single package)
    delay(1000);
    dropper_servo.write(100); // Pushes package outside
    delay(1000);
    dropper_servo.write(90); //returns to original position
}

void vic_move() {
    if (!colour.detect_green() && !colour.detect_red()) {
        state = FORWARD;
    } else {
        motor.move(MOVE_SPEED, MOVE_SPEED);
    }
}

float wrap_angle(float angle) 
{
    if (angle <= -180.0f) {
        angle += 360.0f;
    } else if (angle > 180.0f) {
        angle -= 360.0f;
    }
    return angle;
}

String stateToName(int st) {
    switch (state) {
    case FORWARD:
        return "Forward";
        break;
    case ROTATE_L:
        return "Turn Left";
        break;
    case ROTATE_R:
        return "Turn Right";
        break;
    case ROTATE_180:
        return "Turn 180";
        break;
    case NAV:
        return "Nav State";
        break;
    case BT_BACK:
        return "BT_BACK";
        break;
    case BT_PAUSE:
        return "BT_pause";
        break;
    case BT_ROTATE:
        return "BT_Rotate";
        break;
    case VICTIMS:
       return "V";
        break;
    case END:
        return "End";
        break;
    case PAUSE:
        return "Pause";
        break;
    case VIC_MOVE:
        return "vic_move";
        break;
    case VIC_PAUSE:
        return "vic_pause";
        break;
    default:
        return "NAN";
        break;
    }
}