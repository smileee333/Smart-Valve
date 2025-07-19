#include <Servo.h>
// Assumptions:
//    -> water flow is considered to be laminar in nature so flow_rate remains constant even if it changes
//    -> In bucket mode : Bucket's base distance from tap doesn't change that 
//    -> Components used should be control logic, hardware timer, distance_measuring sensor(may be IR/Ultrasonic), Actuaror(to close/open valve)
//
#define flow_rate 10
#define crosssection_area 2 
#define time_units 60000 // 1 min
#define decremental_cal_delay 1000 // 1000 is taken as arbitirary value

//------------------------------------------------------------------------------------------------------------------
//arduino micro specific definitions

//pin numbers
typedef struct{
    const int Bucket = 15;//PCINT13 -> PCIE1[14:8] -> PCMSK1
    const int Timer = 10;//PCINT2 -> PCIE0[7:0] -> PCMSK0
    const int Static = 4;//PCINT20 -> PCIE2[23:16] ->PCMSK2
    const int Reset = 2;//External Interrupt -> INT0
}ir;
volatile ir interrupt;

typedef struct{
    const int Bucket = 9;
    const int Timer = 8;
    const int Static = 7;
    int present=0;//to check the LED which is currently turned on
}indicators;
volatile indicators LED;

const int valve_pin = 11;
const int timer_input_pin =14;
typedef struct{
    int max_hand_proximity_macro = 20.00;
    int max_proximity = 300.00;
    int trigPin = 5;
    int echoPin = 6;
}HCSR04;
volatile HCSR04 proximity;


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
    state present;
    state past;
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
volatile capability_reg cap;

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

    float duration = pulseIn(proximity.echoPin, HIGH);
    float distance = (duration*.0343)/2;
    return distance;
};
//---------------------------------------------------------------Valve_Actuator_Function----------------------------------------------------------------------
//a boolen variable save the current state of valve ,  false means closed and true means open
Servo valve;


bool valve_state=false;
void valve_action(bool valve_state){
    int open_angle=90; //measured in degree
    int closing_angle=0; //measured in degree
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
        int d1=d_measure();
        delay(decremental_cal_delay);
        int d2=d_measure();
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
};
//-----------------------------------static_capability-------------------------------------------

bool isStaticDisable(){
    if(!cap.static_mode.static_status_bit){
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
};


//-----------------------Reset Capablity------------------------------------------------
void reset_handler(){
    //reset handler should reset everything to handwash state
    cap.cap_reg.control_bit=0;
    cap.cap_reg.status_bit=0;
    cap.cap_reg.data=proximity.max_hand_proximity_macro;
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
    modeSel.active=handwash_state;
    digitalWrite(LED.present,LOW);
    LED.present=0;
};


//toggle control bit of capability register
void toggleControlBit(){
    cap.cap_reg.control_bit=!cap.cap_reg.control_bit;
};
void toggleValveState(){
    valve_state!=valve_state;
};
//function to reinitialize Capability Register when a state change occurs
void reinit(int present){
    switch(present){
        case handwash_state:
            cap.handwash.handwash_status_bit=0;
            cap.handwash.proximity_bit=0;
            cap.handwash.max_hand_proximity=proximity.max_hand_proximity_macro;
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
            cap.timer.timer_status_bit=0;
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

volatile bool rising_button_state=false; // this bit lets us execute ISRs only in the rising edge of interrupt signal

ISR(PCINT1_vect){
    //code that runs when static button hits
    bucket_isr();
};
void bucket_isr(){
    rising_button_state=!rising_button_state;
    if(rising_button_state==true){
        modeSel.capture=bucket_state;
        if(modeSel.active==modeSel.capture){
            modeSel.active=handwash_state;
            digitalWrite(LED.present,LOW);
            LED.present=0;
        }
        else{
            modeSel.active=bucket_state;
            if(LED.present!=0){
                digitalWrite(LED.present,LOW);
            }
            LED.present=LED.Bucket;
            digitalWrite(LED.present,HIGH);
        }
    }
};

ISR(PCINT0_vect){
    //code that runs when static button hits
    timer_isr();
}

void timer_isr(){
    rising_button_state=!rising_button_state;
    if(rising_button_state==true){
        modeSel.capture=timer_state;
        if(modeSel.active==modeSel.capture){
            modeSel.active=handwash_state;
            digitalWrite(LED.present,LOW);
            LED.present=0;
            cap.timer.timer_status_bit=0;
        }
        else{
            modeSel.active=timer_state;
            //capture the current timer value from potentiometer and store it in cap.timer.timer_input 
            cap.timer.timer_input=analogRead(timer_input_pin);
            if(LED.present!=0){
                digitalWrite(LED.present,LOW);
            }
            LED.present=LED.Timer;
            digitalWrite(LED.present,HIGH);
            cap.timer.timer_status_bit=1;
        }
    }
};


ISR(PCINT2_vect){
    //code that runs when static button hits
    static_isr();
}
void static_isr(){
    rising_button_state=!rising_button_state;
    if(rising_button_state==true){
        modeSel.capture=static_state;
        if(modeSel.active==modeSel.capture){
            modeSel.active=handwash_state;       
            digitalWrite(LED.present,LOW);
            LED.present=0;
            cap.static_mode.static_status_bit=0;
        }
        else{
            modeSel.active=static_state;
            if(LED.present!=0){
                digitalWrite(LED.present,LOW);
            }
            LED.present=LED.Static;
            digitalWrite(LED.present,HIGH);
            cap.static_mode.static_status_bit=1;
        }
    }
};
ISR(INT0_vect){
    //code that runs when reset button hits 
    reset_isr();
}
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
    //pinmodes configuration
    pinMode(LED.Bucket, OUTPUT);
    pinMode(LED.Timer, OUTPUT);
    pinMode(LED.Static, OUTPUT);

    pinMode(proximity.trigPin, OUTPUT);
    pinMode(proximity.echoPin, INPUT);

    pinMode(valve_pin,OUTPUT);
    pinMode(timer_input_pin,INPUT);//for potentiometer-> timer input

    pinMode(interrupt.Bucket, INPUT_PULLUP);
    pinMode(interrupt.Timer, INPUT_PULLUP);
    pinMode(interrupt.Static, INPUT_PULLUP);
    pinMode(interrupt.Reset, INPUT_PULLUP);

    //attaching servo motor:
    valve.attach(valve_pin);
    //attaching interrupts----->
    //Pin change interupts :
    PCICR=0b00000111;//enable PCIE0,PCIE1,PCIE2 for bucket, timer,static
    PCMSK0=0b00000100;//PCINT2
    PCMSK1=0b0000010;//PCINT9
    PCMSK2=0b00010000;//PCINT20
    //External Interrupts"
    EICRA=0b00000011;
    EIMSK=0b00000001;//enabling int0

    sei();//enabling interrupts gloabally

    //state-machine initialization ------>
    C_state.past=handwash_state;//initializing to handwash state
    reset_handler();
    modeSel.active=handwash_state;

    //Turning off all the LEDs initally
    digitalWrite(LED.Bucket,LOW);
    digitalWrite(LED.Timer,LOW);
    digitalWrite(LED.Static,LOW);
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
    runStateMachine();
};