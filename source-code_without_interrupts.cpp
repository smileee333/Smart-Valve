#include <Servo.h>
#include "PinChangeInterrupt.h"
/* Assumptions:
    -> water flow is considered to be laminar in nature so flow_rate remains constant even if it changes
    -> In bucket mode : Bucket's base distance from tap doesn't change that 
    -> Components used should be control logic, hardware timer, distance_measuring sensor(may be IR/Ultrasonic), Actuaror(to close/open valve)


*/
#define flow_rate 10
#define crosssection_area 2 
#define time_units 60000 // 1 min
#define decremental_cal_delay 1000 // 1000 is taken as arbitirary value

//------------------------------------------------------------------------------------------------------------------
//arduino micro specific definitions

//pin numbers
typedef struct{
    const int Bucket =3;
    const int Timer = 2;
    const int Static = 0;
    const int Reset = 1;
}interrupt;

typedef struct{
    const int Bucket = 9;
    const int Timer = 8;
    const int Static = 7;
}LED;

const int valve_pin = 11;

typedef struct{
    int max_hand_proximity_macro = 10.00
    int max_proximity = 300.00
    const int trigPin = 5;
    const int echoPin = 6;
}proximity;



//-----------------------------------------------------------------Registers Definition--------------------------------------------------------------

//-------------Controlstate_reg definiton--------------
//definition
typedef enum {
    handwash_state,
    bucket_state,
    timer_state,
    static_state,
    reset_state
}state;

typedef struct {
    state present:3;
    state past:3;
}state_reg;

//declaration
state_reg C_state;// C_state refers to controller state here



//------------Capability_reg definiton-----------------
//definition
typedef struct {
    bool handwash_status_bit;
    bool proximity_bit;
    int max_hand_proximity;
}handwash_cap;

typedef struct{
    int decremental_rate;
    int past_value;
}filling_info;

typedef struct {
    bool bucketfull_bit;
    bool buckfilling_bit;
    filling_info filling_stat;
}bucket_cap;

typedef struct {
    bool timer_status_bit;
    bool timer_start_bit;
    int timer_input;
}timer_cap;

typedef struct {
    bool static_status_bit;
    bool static_running_bit;
    bool static_input;
}static_cap;

typedef struct {
    bool status_bit;
    bool control_bit;
    int data;
}reg;

typedef union {
    reg cap_reg;
    handwash_cap handwash;
    bucket_cap bucket;
    timer_cap timer;
    static_cap static_mode; 
}capability_reg;
//declaration
capability_reg cap;

typedef struct{
    state active;
    state capture; 
}modeselection;
//declaration
volatile modeselection modeSel;

//---------------------------------------------------------------Distance_Sensor_Funtion----------------------------------------------------------------------

float d_measure(){
    digitalWrite(proximity.trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(proximity.trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(proximity.trigPin, LOW);

    duration = pulseIn(proximity.echoPin, HIGH);
    distance = (duration*.0343)/2;
    return distance
}
//---------------------------------------------------------------Valve_Actuator_Function----------------------------------------------------------------------
//a boolen variable save the current state of valve ,  false means closed and true means open
Servo valve;

valve.attach(valve_pin);
bool valve_state=false;
void valve_action(bool valve_state){
    open_angle=90; //measured in degree
    closing_angle=0; //measured in degree
    if(valve_state){
        //opening of valve
        valve.write(open_angle);
        //waiting for valve to get there
        delay(15);
    }
    else{
        //closing of valve
        valve.write(closing_angle);
        //waiting for valve to get there
        delay(15);
    }  
};
//--------------------------------------------------------------LED_Actuation_Function---------------------------------------------------------------


typdef struct{
    bool Bucket = false;
    bool Timer = false;
    bool Static = false;
}LED_state;

void toggleLED(cont int LED){
    switch(LED){
        case LED.Bucket:
            if(LED_state.Bucket == false){
                digitalwrite(LED.Bucket,HIGH)
            }
            else{
                digitalwrite(LED.Bucket,LOW)
            }
            break;
        case LED.Timer:
            if(LED_state.Timer == false){
                digitalWrite(LED.Timer,HIGH)
            }
            else{
                digitalWrite(LED.Timer,LOW)
            }
            break;
        case LED.Static:
            if(LED_state.Static == false){
                digitalWrite(LED.Static,HIGH)
            }
            else{
                digitalWrite(LED.Static,LOW)
            } 
    }

}
//---------------------------------------------------------------Capability Function definition ------------------------------------------------------------------------------
//Handwash-capability---------------------------------------------------------------------------------
bool isInProximity(){
    return (d_measure()<=cap.handwash.max_hand_proximity);
};

void handwash_handler(){
    if(isInProximity()!= cap.handwash.proximity_bit){
        toggleControlBit();
        toggleValveState();
        valve_action(valve_state);
    }
};




//-------------------------------------------------------Bucket-capabiltiy------------------------------------------------------------------------------------

bool isBucketFull(){
    if(cap.bucket.buckfilling_bit==0)
        return 0;
    if(cap.bucket.filling_stat.decremental_rate==0){
        uint8_t d1=d_measure();
        delay(decremental_cal_delay);
        uint8_t d2=d_measure();
        cap.bucket.filling_stat.decremental_rate=d1-d2;
        cap.bucket.filling_stat.past_value=d2;
    }
    delay(decremental_cal_delay);
    if(d_measure()>cap.bucket.filling_stat.past_value-cap.bucket.filling_stat.decremental_rate){
        cap.bucket.bucketfull_bit=1;
        return 1;
    }
    else
        return 0;
};
void bucket(){

    if(isBucketFull()==cap.bucket.buckfilling_bit)
    {
        toggleControlBit();
        toggleValveState();
        valve_action(valve_state);
    }
};
void bucket_handler(){
    if(cap.bucket.bucketfull_bit==0)
        bucket();
    else
        toggleModeSelectionInput();
        toggleLED(LED.Bucket);

};

//----------------------------------------------------------timer capability--------------------------------------------
bool isTimesUp(){
    if(cap.timer.timer_start_bit==0)
        return 0;
    if(cap.timer.timer_input!=0){
        delay(time_units);
        cap.timer.timer_input--;
        return 0;
    }
    else{
        cap.timer.timer_status_bit=0;
        return 1;
    }
};
void timer(){
    if(isTimesUp()==cap.timer.timer_start_bit){
        toggleControlBit();
        toggleValveState();
        valve_action(valve_state);
    }
};
void timer_handler(){
    if(cap.timer.timer_status_bit==1){
        timer();
    }
    else
        toggleModeSelectionInput();
        toggleLED(LED.Timer);
};
//-----------------------------------static_capability-------------------------------------------

bool isStaticDisable(){
    if(cap.static_mode.static_input!=readStaticInput()){
        cap.static_mode.static_status_bit=0;
        return 1;
    }
    return 0;
};



void static_handler(){
    if(isStaticDisable()==cap.static_mode.static_running_bit){
        toggleControlBit();
        toggleValveState();
        valve_action(valve_state);
    }
    else
        toggleModeSelectionInput();
        toggleLED(LED.Static);
};


//-----------------------Reset Capablity------------------------------------------------
void reset_handler(){
    //reset handler should reset everything to handwash state
    cap.cap_reg.control_bit=0;
    cap.cap_reg.status_bit=0;
    cap.cap_reg.data=max_hand_proximity_macro;
    valve_state=false;//initializing valve state to be closed
    valve_action(valve_state);
    
    toggleModeSelectionInput();

};

//----------------------------------------------Function Pointer array to handler all capability handlers in one place-------------------------------
typedef void(*handlers)();
handlers state_handler[]={&handwash_handler,&bucket_handler,&timer_handler,&static_handler,&reset_handler};

//---------------------------------------------------------------Implicit Function Definitions--------------------------------------------------------------------

void toggleModeSelectionInput(){
    //toggling back to handwash_state
    modeSel.active=handwash_state
};


//toggle control bit of capability register
void toggleControlBit(){
    cap.cap_reg.control_bit=!cap.cap_reg.control_bit;
};
void toggleValveState(){
    if(valve_state==false){
        valve_state=true;
    }
    else{
        valve_state=false;
    }
    
};
//function to reinitialize Capability Register when a state change occurs
void reinit(state present){
    switch(present){
        case handwash_state:
            cap.handwash.handwash_status_bit=0;
            cap.handwash.proximity_bit=0;
            cap.handwash.max_hand_proximity=max_hand_proximity_macro;
            valve_state=false;//initializing valve state to be closed
            valve_action(valve_state);
            break;

        case bucket_state:
            cap.bucket.bucketfull_bit=0;
            cap.bucket.buckfilling_bit=0;
            cap.bucket.filling_stat.decremental_rate=0;
            cap.bucket.filling_stat.past_value=0;
            valve_state=false;//initializing valve state to be closed
            valve_action(valve_state);
            break;

        case timer_state:
            cap.timer.timer_status_bit=1;
            cap.timer.timer_start_bit=0;
            cap.timer.timer_input=0;
            valve_state=false;//initializing valve state to be closed
            valve_action(valve_state);
            break;

        case static_state:
            cap.static_mode.static_status_bit=1;
            cap.static_mode.static_running_bit=0;
            cap.static_mode.static_input=0;
            valve_state=false;//initializing valve state to be closed
            valve_action(valve_state);
    }
};


//-----------------------------------------------------------ISRs to capture the mode of usage in run time---------------------------------------------

void bucket_isr(){
    modeSel.capture=bucket_state;
    if(modeSel.active==modeSel.capture){
        modeSel.active=handwash_state
        toggleLED(LED.Bucket);
    }
    else{
        modeSel.active=bucket_state;
        toggleLED(LED.Bucket);
    }
    
};


void timer_isr(){
    modeSel.capture=timer_state;
    if(modeSel.active==modeSel.capture){
        modeSel.active=handwash_state
        toggleLED(LED.Timer);
    }
    else{
        modeSel.active=bucket_state;
        toggleLED(LED.Timer);
    }
};+



void static_isr(){
    modeSel.capture=static_state;
    if(modeSel.active==modeSel.capture){
        modeSel.active=handwash_state;
        toggleLED(LED.Timer);
    }
    else{
        modeSel.active=static_state;
        toggleLED(LED.Static);
    }
};

void reset_isr(){
    modeSel.active=reset_state;
};

// function to run Statemachine
void runStateMachine(){
    //To check if same previous state is in continuation or not (by comparing past and present Contoller state), if not then reinitialize for new state and update past System state with new System state as well.
    if(C_state.past!=C_state.present){
        reinit(C_state.present);
        C_state.past=C_state.present;
    }
    //invoke the appropriate handler
    state_handler[C_state.present]();
};


void setup(){
    //state-machine initialization ------>
    C_state.past=handwash_state;//initializing to handwash state
    reset_handler();
    modeSel.active=handwash_state;
    //pinmodes configuration
    pinMode(LED.Bucket, OUTPUT);
    pinMode(LED.Timer, OUTPUT);
    pinMode(LED.Static, OUTPUT);

    pinmode(proximity.trigPin, OUTPUT);
    pinmode(proximity.echoPin, INPUT);

    pinMode(valve_pin,OUTPUT);

    pinMode(interrupt.Bucket, INPUT_PULLUP);
    pinMode(interrupt.Timer, INPUT_PULLUP);
    pinMode(interrupt.Static, INPUT_PULLUP);
    pinMode(interrupt.Reset, INPUT_PULLUP);
    //attaching interrupts----->
    attachPCINT(digitalPinToPCINT(interrupt.Bucket),bucket_isr,Rising)
    attachPCINT(digitalPinToPCINT(interrupt.Timer),timer_isr,Rising)
    attachPCINT(digitalPinTOPCINT(interrupt.Static),static_isr,Rising)
    attachPCINT(digitalPinTOPCINT(interrupt.Reset),reset_isr,Rising)
};

void loop(){
    switch(modeSel.active){
        case handwash_state:
            C_state.present=handwash_state;
            break;
        case bucket_state:
            C_state.present=bucket_state;
            break;
        case timer_state:
            C_state.present=timer_state;
            break;
        case static_state:
            C_state.present=static_state;
            break;
        case reset_state:
            C_state.present=reset_state;
            break;        
    }
    runStateMachine()
};