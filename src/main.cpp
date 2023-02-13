#include <math.h>
#include <mbed.h>

Timer t, t1;
Ticker Tracker;
int ii, jj, kk = 0;
int aa, bb, cc = 0;
int xx[20], ch1[20], chh[8];
int Counter = 0;
int Stop = 0;
int autoThread;
int manuThread;
int rfThread;
float prevTimeRF;

double robotArrived = 0;

float AngleNormalized = 0;
float SpeedValueWifi = 0;
float AngleValueWifi = 0;
// specifing arrays and variables to store values
void read_me(), read_rc();
Timer t2, t3;

static UnbufferedSerial ESP32(p28, p27);
int countt = 0;
int bufflen, DataRx;
char buf[300];
char static_buf[300];
char Status[300];
char snd[300];
char a[2], b[2], c[2], d[2], e[2], f[2], g[2], h[2], i[2], j[2], k[2], l[2], m[2], n[2], o[2];
char ssid[32] = "ESP8266"; // WiFi router ssid inside the quotes
char pwd[32] = "123456789"; // WiFi router password inside the quotes
char ch[1];
int parameterOrder;
float parameterValue = 0.0f;

EventQueue queue(32 * EVENTS_EVENT_SIZE);
EventQueue queue1(32 * EVENTS_EVENT_SIZE);
EventQueue queue2(32 * EVENTS_EVENT_SIZE);

char buffer[10];
int readingIndex = 0;
float* f_buf = (float*)buffer;
float Speed = 0.0f;
float Angle = 0.0f;
float RecieverValueCh1 = 0.0f;
float RecieverValueCh2 = 0.0f;
float Vr = 0.0f;
float Vl = 0.0f;
float Dc = 0.0f;
float Dl = 0.0f;
float Dr = 0.0f;
float Xpos = 0.0f;
float Ypos = 0.0f;
float Phi = 0.0f;
float DLL = 0;
float DRR = 0;

float xGoal = 0.0f;
float yGoal = 0.0f;

float xObstacle = 0.0f;
float yObstacle = 0.0f;

float xObstacle1 = 0.0f;
float yObstacle1 = 0.0f;

float minDistanceObs = 0.0f;
float vObstacleMin[2];

float xObstacle2 = 0.0f;
float yObstacle2 = 0.0f;

float kObstacle = 0.2f;
float kGoal = 0.5f;

float Obstacles[3];
float vObstacles;

float dGoal = 0.0f;
float dGoalMax = 0.2f;
float dObstacle = 0.0f;
float dObstacle1 = 0.0f;
float dObstacleMax = 1.0f;

float vObstacle[2];
float vObstacle1[2];
float vObstacle2[2];
float vGoal[2];

float kAngle = 0.35f;

float directionAngle = 0.0f;

float SpeedAngle = 0.0f;

float vMax = 0.4f;
float vOMax = 0.7f;

#define R 0.127f
#define L 0.385f

static DigitalOut led4(LED4);
static DigitalOut led2(LED2);
static DigitalOut led3(LED3);
static DigitalOut led(LED1);

DigitalOut dir(p22);
DigitalOut dir1(p24);

DigitalIn HSU(p15);
DigitalIn HSV(p16);
DigitalIn HSW(p17);

InterruptIn RC(p30);

DigitalIn HSU1(p7);
DigitalIn HSV1(p6);
DigitalIn HSW1(p5);

InterruptIn HallSensor_U(p15);
InterruptIn HallSensor_V(p16);
InterruptIn HallSensor_W(p17);

InterruptIn HallSensor_U1(p7);
InterruptIn HallSensor_V1(p6);
InterruptIn HallSensor_W1(p5);

PwmOut pwm(p21);
PwmOut pwm1(p23);

typedef struct {

    /* Controller gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low-pass filter time constant */
    float tau;

    /* Output limits */
    float limMin;
    float limMax;

    /* Integrator limits */
    float limMinInt;
    float limMaxInt;

    /* Sample time (in seconds) */
    float T;

    /* Controller "memory" */
    float integrator;
    float prevError; /* Required for integrator */
    float differentiator;
    float prevMeasurement; /* Required for differentiator */

    /* Controller output */
    float out;

} PIDController;

void ESPconfig(), SendCMD(), clearBuffer(), clear_static_Buffer(), ClearSnd(), resetVariables(), readWifiData(),
    sendData(), updateRobot(), manualMode(), autoMode(), RFmode(), resetOdometryValues();

float constrainAngle(float AngleNonNormalized)
{
    AngleNonNormalized = fmod(AngleNonNormalized + 3.14, 2 * 3.14);
    if (AngleNonNormalized < 0)
        AngleNonNormalized += 2 * 3.14;
    return AngleNonNormalized - 3.14;
}

float PIDController_Update(PIDController* pid, float setpoint, float measurement)
{

    /*
     * Error signal
     */
    float error = setpoint - measurement;

    /*
     * Proportional
     */
    float proportional = pid->Kp * error;

    /*
     * Integral
     */
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    /* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {
        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;
    }

    /*
     * Derivative (band-limited differentiator)
     */

    pid->differentiator
        = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) /* Note: derivative on measurement, therefore minus
                                                                     sign in front of equation! */
              + (2.0f * pid->tau - pid->T) * pid->differentiator)
        / (2.0f * pid->tau + pid->T);

    /*
     * Compute output and apply limits
     */
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }

    /* Store error and measurement for later use */
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    /* Return controller output */
    return pid->out;
}

void PIDController_Init(PIDController* pid)
{
    /* Clear controller variables */
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;

    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;
}

void read_me()
{
    // this code reads value from RC reciever from PPM pin
    // this code gives channel values from 0-1000 values
    aa = t1.read_us(); // store time value a when pin value falling
    cc = aa - bb; // calculating time inbetween two peaks
    bb = aa; // Kp
    xx[ii] = cc; // storing 15 value in array
    ii = ii + 1;
    if (ii == 18) {
        for (int jj = 0; jj <= 18; jj++) {
            ch1[jj] = xx[jj];
        }
        ii = 0;
    }
    prevTimeRF = t.read_ms();

} // copy store all values from temporary array another array after 15 reading

void on_rx_interrupt()
{
    if (countt == 99) {
        countt = 0;
    }
    led = 1;
    char c;
    if (ESP32.read(&c, 1)) {
        ch[0] = c;
        if (strstr(ch, "{") != NULL && countt == 0) {
            countt++;
        } else if (countt > 0 && strstr(ch, "{") == NULL && strstr(ch, "}") == NULL) {
            buf[countt - 1] = c;
            countt++;
        }
        if (strstr(ch, "}") != NULL) {
            countt = 0;
            clear_static_Buffer();
            strcpy(static_buf, buf);
            clearBuffer();
            if (strlen(static_buf) != 0) {
                queue.call(&readWifiData);
            }
        }
    }
    led = 0;
}

#define CW 1 // Assign a value to represent clock wise rotation
#define CCW -1 // Assaign a value to represent counter-clock wise rotation

// variable for the first motor
float setpoint = 0.0f;
int direct = 1; // Integer variable to store BLDC rotation direction
int pulseCount; // Integer variable to store the pulse count

float startTime; // Float variable to store the start time of the current interrupt
float prevTime; // Float variable to store the start time of the previous interrupt
float pulseTimeW; // Float variable to store the elapsed time between interrupts for hall sensor W
float pulseTimeU; // Float variable to store the elapsed time between interrupts for hall sensor U
float pulseTimeV; // Float variable to store the elapsed time between interrupts for hall sensor V
float AvPulseTime; // Float variable to store the average elapsed time between all interrupts

float PPM; // Float variable to store calculated pulses per minute
float RPM; // Float variable to store calculated revolutions per minute
float HSUV;
float HSVV;
float HSWV;

float HSU_Val = HSU.read();
float HSV_Val = HSV.read(); // Set the V sensor value as boolean and read initial state
float HSW_Val = HSW.read(); // Set the W sensor value as boolean and read initial state

// varaibles for the second motor////////////////////////
float setpoint1 = 0.0f;
int direct1 = 1; // Integer variable to store BLDC rotation direction
int pulseCount1; // Integer variable to store the pulse count

float startTime1; // Float variable to store the start time of the current interrupt
float prevTime1; // Float variable to store the start time of the previous interrupt
float pulseTimeW1; // Float variable to store the elapsed time between interrupts for hall sensor W
float pulseTimeU1; // Float variable to store the elapsed time between interrupts for hall sensor U
float pulseTimeV1; // Float variable to store the elapsed time between interrupts for hall sensor V
float AvPulseTime1; // Float variable to store the average elapsed time between all interrupts

float PPM1; // Float variable to store calculated pulses per minute
float RPM1; // Float variable to store calculated revolutions per minute
float HSUV1;
float HSVV1;
float HSWV1;

float HSU_Val1 = HSU1.read();
float HSV_Val1 = HSV1.read(); // Set the V sensor value as boolean and read initial state
float HSW_Val1 = HSW1.read(); // Set the W sensor value as boolean and read initial state

float Rwheeld = 0.0f;
float Lwheeld = 0.0f;

double normalizedAngle = 0;

/////////////////////

void HallSensorU()
{
    startTime = t.read_ms();
    HSU_Val = HSU.read();
    HSW_Val = HSW.read(); // Read the current W (or V) hall sensor value
    direct = (HSU_Val == HSW_Val) ? CW : CCW;
    if (direct == CW) {
        // increase
        Lwheeld += 1;
    } else {
        // decrease
        Lwheeld -= 1;
    }
    pulseCount = pulseCount + (1 * direct);
    pulseTimeU = startTime - prevTime;
    AvPulseTime = ((pulseTimeW + pulseTimeU + pulseTimeV) / 3);
    PPM = (1000 / AvPulseTime) * 60;
    RPM = PPM / 90;
    prevTime = startTime;
}

void HallSensorV()
{
    startTime = t.read_ms();
    HSV_Val = HSV.read();
    HSU_Val = HSU.read(); // Read the current U (or W) hall sensor value
    direct = (HSV_Val == HSU_Val) ? CW : CCW;
    if (direct == CW) {
        // increase
        Lwheeld += 1;
    } else {
        // decrease
        Lwheeld -= 1;
    }
    pulseCount = pulseCount + (1 * direct);
    pulseTimeV = startTime - prevTime;
    AvPulseTime = ((pulseTimeW + pulseTimeU + pulseTimeV) / 3);
    PPM = (1000 / AvPulseTime) * 60;
    RPM = PPM / 90;
    prevTime = startTime;
}

void HallSensorW()
{
    startTime = t.read_ms(); // Set startTime to current microcontroller elapsed time value
    HSW_Val = HSW.read(); // Read the current W hall sensor value
    HSV_Val = HSV.read(); // Read the current V (or U) hall sensor value
    direct = (HSW_Val == HSV_Val) ? CW : CCW; // Determine rotation direction (ternary if statement)
    if (direct == CW) {
        // increase
        Lwheeld += 1;
    } else {
        // decrease
        Lwheeld -= 1;
    }
    pulseCount = pulseCount + (1 * direct); // Add 1 to the pulse count
    pulseTimeW = startTime - prevTime; // Calculate the current time between pulses
    AvPulseTime = ((pulseTimeW + pulseTimeU + pulseTimeV) / 3); // Calculate the average time time between pulses
    PPM = (1000 / AvPulseTime) * 60; // Calculate the pulses per min (1000 millis in 1 second)
    RPM = PPM / 90; // Calculate revs per minute based on 90 pulses per rev
    prevTime = startTime; // Remember the start time for the next interrupt
}

////////////////////

void HallSensorU1()
{
    startTime1 = t1.read_ms();
    HSU_Val1 = HSU1.read();
    HSW_Val1 = HSW1.read(); // Read the current W (or V) hall sensor value
    direct1 = (HSU_Val1 == HSW_Val1) ? CW : CCW;
    if (direct1 == CW) {
        // increase
        Rwheeld += 1;
    } else {
        // decrease
        Rwheeld -= 1;
    }
    pulseCount1 = pulseCount1 + (1 * direct1);
    pulseTimeU1 = startTime1 - prevTime1;
    AvPulseTime1 = ((pulseTimeW1 + pulseTimeU1 + pulseTimeV1) / 3);
    PPM1 = (1000 / AvPulseTime1) * 60;
    RPM1 = PPM1 / 90;
    prevTime1 = startTime1;
}

void HallSensorV1()
{
    startTime1 = t1.read_ms();
    HSV_Val1 = HSV1.read();
    HSU_Val1 = HSU1.read(); // Read the current W (or V) hall sensor value
    direct1 = (HSV_Val1 == HSU_Val1) ? CW : CCW;
    if (direct1 == CW) {
        // increase
        Rwheeld += 1;
    } else {
        // decrease
        Rwheeld -= 1;
    }
    pulseCount1 = pulseCount1 + (1 * direct1);
    pulseTimeV1 = startTime1 - prevTime1;
    AvPulseTime1 = ((pulseTimeW1 + pulseTimeU1 + pulseTimeV1) / 3);
    PPM1 = (1000 / AvPulseTime1) * 60;
    RPM1 = PPM1 / 90;
    prevTime1 = startTime1;
}

void HallSensorW1()
{

    startTime1 = t1.read_ms(); // Set startTime to current microcontroller elapsed time value
    HSW_Val1 = HSW1.read(); // Read the current W hall sensor value
    HSV_Val1 = HSV1.read(); // Read the current V (or U) hall sensor value
    direct1 = (HSW_Val1 == HSV_Val1) ? CW : CCW; // Determine rotation direction (ternary if statement)
    if (direct1 == CW) {
        // increase
        Rwheeld += 1;
    } else {
        // decrease
        Rwheeld -= 1;
    }
    pulseCount1 = pulseCount1 + (1 * direct1); // Add 1 to the pulse count
    pulseTimeW1 = startTime1 - prevTime1; // Calculate the current time between pulses
    AvPulseTime1 = ((pulseTimeW1 + pulseTimeU1 + pulseTimeV1) / 3); // Calculate the average time time between pulses
    PPM1 = (1000 / AvPulseTime1) * 60; // Calculate the pulses per min (1000 millis in 1 second)
    RPM1 = PPM1 / 90; // Calculate revs per minute based on 90 pulses per rev
    prevTime1 = startTime1; // Remember the start time for the next interrupt
}

#define PID_KP 0.0003f
#define PID_KI 300.0f
#define PID_KD 0.0f

#define PID_TAU 0.0f

#define SAMPLE_TIME_S 0.000001f

#define PID_LIM_MIN 0.0f
#define PID_LIM_MAX 0.08f

#define PID_LIM_MIN_INT 0.0f
#define PID_LIM_MAX_INT 0.2f

//=========================================

#define PID_KP1 0.0005f
#define PID_KI1 300.0f
#define PID_KD1 0.0f

#define PID_TAU1 0.0f

#define SAMPLE_TIME_S1 0.000001f

#define PID_LIM_MIN1 -0.08f
#define PID_LIM_MAX1 0.08f

#define PID_LIM_MIN_INT1 -1.0f
#define PID_LIM_MAX_INT1 1.0f

PIDController pid
    = { PID_KP, PID_KI, PID_KD, PID_TAU, PID_LIM_MIN, PID_LIM_MAX, PID_LIM_MIN_INT, PID_LIM_MAX_INT, SAMPLE_TIME_S };

PIDController pid1 = { PID_KP1, PID_KI1, PID_KD1, PID_TAU1, PID_LIM_MIN1, PID_LIM_MAX1, PID_LIM_MIN_INT1,
    PID_LIM_MAX_INT1, SAMPLE_TIME_S1 };

int main()
{

    // put your setup code here, to run once:

    t2.start();
    HSU.mode(PullDown);
    HSV.mode(PullDown);
    HSW.mode(PullDown);

    HSU1.mode(PullDown);
    HSV1.mode(PullDown);
    HSW1.mode(PullDown);

    RC.rise(&read_me);

    PIDController_Init(&pid);
    PIDController_Init(&pid1);

    HallSensor_U1.rise(&HallSensorU1);
    HallSensor_U1.fall(&HallSensorU1);
    HallSensor_V1.rise(&HallSensorV1);
    HallSensor_V1.fall(&HallSensorV1);
    HallSensor_W1.rise(&HallSensorW1);
    HallSensor_W1.fall(&HallSensorW1);

    HallSensor_U.rise(&HallSensorU);
    HallSensor_U.fall(&HallSensorU);
    HallSensor_V.rise(&HallSensorV);
    HallSensor_V.fall(&HallSensorV);
    HallSensor_W.rise(&HallSensorW);
    HallSensor_W.fall(&HallSensorW);
    led4 = 0;

    t.start();
    t1.start();
    t3.start();

    pwm.period(0.0009f); // 4 second period
    pwm.write(0.0f);

    pwm1.period(0.0009f); // 4 second period
    pwm1.write(0.0f);

    Thread eventThread(osPriorityAboveNormal1, 1500);
    Thread eventThread1(osPriorityAboveNormal1, 1500);

    ESP32.baud(115200);
    ESP32.format(
        /* bits */ 8,
        /* parity */ SerialBase::None,
        /* stop bit */ 1);
    ESP32.attach(&on_rx_interrupt, SerialBase::RxIrq);
    ESPconfig();
    eventThread1.start(callback(&queue1, &EventQueue::dispatch_forever));
    queue1.call_every(200ms, &sendData);
    eventThread.start(callback(&queue, &EventQueue::dispatch_forever));
    queue.call_every(100ms, &updateRobot);
    clear_static_Buffer();
    wait_us(osWaitForever);
}

void updateRobot()
{
    // put your main code here, to run repeatedly:
    led2 = 1;
    if ((t.read_ms() - prevTime) > 700) {
        RPM = 0;
    }
    if ((t1.read_ms() - prevTime1) > 700) {

        RPM1 = 0;
    }

    if (RPM > 500) {
        RPM = 0;
    }
    if (RPM1 > 500) {
        RPM1 = 0;
    }

    Dl = (2 * 3.14159 * R) * (Lwheeld / 90);
    Dr = (2 * 3.14159 * R) * (Rwheeld / 90);
    DLL = DLL + Lwheeld;
    DRR = DRR + Rwheeld;
    Rwheeld = 0.0f;
    Lwheeld = 0.0f;
    Dc = (Dl + Dr) / 2;
    Phi = Phi + (Dr - Dl) / L;
    Xpos = Xpos + Dc * cos(Phi);
    Ypos = Ypos + Dl * sin(Phi);
    float average = 0.0f;
    for (int i = 0; i < 10; i++) {
        average = average + RPM;
    }
    average = average / 10.0f;

    float average1 = 0.0f;
    for (int i = 0; i < 10; i++) {
        average1 = average1 + RPM1;
    }
    average1 = average1 / 10.0f;

    if (Vl > 0.0f) {
        PIDController_Update(&pid, Vl, RPM);
        pwm1.write(pid.out);
        dir = 0;

    } else if (Vl < 0.0f) {
        PIDController_Update(&pid, -Vl, RPM);
        pwm1.write(pid.out);
        dir = 1;
    } else {

        pwm1.write(0.0f);
        PIDController_Init(&pid);
    }
    if (Vr > 0.0f) {
        PIDController_Update(&pid1, Vr, RPM1);
        pwm.write(pid1.out);
        dir1 = 1;

    } else if (Vr < 0.0f) {
        PIDController_Update(&pid1, -Vr, RPM1);
        pwm.write(pid1.out);
        dir1 = 0;
    } else {

        pwm.write(0.0f);
        PIDController_Init(&pid1);
    }
    normalizedAngle = constrainAngle(Phi);

    led2 = 0;
}

void sendData()
{
    sprintf(Status,
        "{\"Xpos\":%f,\"Ypos\":%f,\"Phi\":%f,\"Speed input\":%f,\"Angle\":%f ,\"robotArrived\":%lf, \"Xgoal\":%f, "
        "\"Ygoal\":%f, \"dGoal\":%f,\"dObstacle\":%f,\"dObstacle1\":%f}\r\n",
        Xpos, Ypos, normalizedAngle, Speed, Angle, robotArrived, xGoal, yGoal, dGoal, dObstacle, dObstacle1);
    ESP32.write(Status, strlen(Status));
    resetVariables();
    led4 = !led4;
}

void autoMode()
{
    robotArrived = 0;

    vGoal[0] = xGoal - Xpos;
    vGoal[1] = yGoal - Ypos;
    vObstacle[0] = xObstacle - Xpos;
    vObstacle[1] = yObstacle - Ypos;

    vObstacle1[0] = xObstacle1 - Xpos;
    vObstacle1[1] = yObstacle1 - Ypos;

    dGoal = sqrt(pow(vGoal[0], 2) + pow(vGoal[1], 2));
    dObstacle = sqrt(pow(vObstacle[0], 2) + pow(vObstacle[1], 2));
    dObstacle1 = sqrt(pow(vObstacle1[0], 2) + pow(vObstacle1[1], 2));

    if (dObstacle < dObstacleMax) {
        Speed = kGoal * sqrt(pow(vGoal[0] - kObstacle * vObstacle[0], 2) + pow(vGoal[1] - kObstacle * vObstacle[1], 2));
        directionAngle = atan2((vGoal[1] - kObstacle * vObstacle[1]), (vGoal[1] - kObstacle * vObstacle[1]));
    } else if (dObstacle1 < dObstacleMax) {
        Speed
            = kGoal * sqrt(pow(vGoal[0] - kObstacle * vObstacle1[0], 2) + pow(vGoal[1] - kObstacle * vObstacle1[1], 2));
        directionAngle = atan2((vGoal[1] - kObstacle * vObstacle1[1]), (vGoal[1] - kObstacle * vObstacle1[1]));
    } else {
        Speed = kGoal * sqrt(pow(vGoal[0], 2) + pow(vGoal[1], 2));
        directionAngle = atan2((vGoal[1]), (vGoal[0]));
    }
    directionAngle = directionAngle + 3.14;
    normalizedAngle = normalizedAngle + 3.14;

    SpeedAngle = constrainAngle(normalizedAngle - directionAngle);
    Angle = SpeedAngle * kAngle;

    if (Speed > vMax) {
        Speed = vMax;
    }
    Vl = ((2 * Speed + Angle * L) / (2.0 * R)) * 60.0 / (2.0 * 3.14);
    Vr = ((2 * Speed - Angle * L) / (2.0 * R)) * 60.0 / (2.0 * 3.14);
    if (Vr * Vl < 0) {
        Vl = -((2 * Speed + Angle * L) / (2 * R)) * 60 / (2 * 3.14);
        Vr = -((2 * Speed - Angle * L) / (2 * R)) * 60 / (2 * 3.14);
    }
    if (dGoal < dGoalMax) {
        robotArrived = 1;
        Vl = 0.0f;
        Vr = 0.0f;
        queue1.cancel(autoThread);
    }
}

void RFmode()
{
    read_rc();
    if (chh[1] > 1600) {
        RecieverValueCh2 = (float(chh[1]) - 1500.0f) / 500.0f;

    } else if (chh[1] < 1400) {
        RecieverValueCh2 = -(1 - (float(chh[1]) - 1000.0f) / 500.0f);

    } else {
        RecieverValueCh2 = 0.0f;
    }

    if (chh[0] > 1500) {
        RecieverValueCh1 = ((float(chh[0]) - 900.0f) / 1000.0f - 0.5) * 2.0 - 0.1;

    } else if (chh[0] < 1400) {
        RecieverValueCh1 = -(0.6 - (float(chh[0]) - 900.0f) / 1000.0f) * 2.0;

    } else {
        RecieverValueCh1 = 0.0f;
    }

    Speed = -RecieverValueCh2 * vMax;
    Angle = RecieverValueCh1 * vOMax;
    Vl = ((2 * Speed + Angle * L) / (2 * R)) * 60 / (2 * 3.14);
    Vr = ((2 * Speed - Angle * L) / (2 * R)) * 60 / (2 * 3.14);
    if (Vr * Vl < 0) {
        Vl = -((2 * Speed + Angle * L) / (2 * R)) * 60 / (2 * 3.14);
        Vr = -((2 * Speed - Angle * L) / (2 * R)) * 60 / (2 * 3.14);
    }
}

void readWifiData()
{
    int len = strlen(static_buf);
    led3 = 1;

    if (sscanf(static_buf, "mode:manual,m:%f,a:%f", &SpeedValueWifi, &AngleValueWifi) && len != 0) {

        queue1.cancel(autoThread);
        queue1.cancel(rfThread);
        Speed = ((SpeedValueWifi - 50.0f) / 50.0f) * vMax;
        Angle = ((AngleValueWifi - 50.0f) / 50.0f) * vOMax;
        Vl = ((2 * Speed + Angle * L) / (2.0 * R)) * 60.0 / (2.0 * 3.14);
        Vr = ((2 * Speed - Angle * L) / (2.0 * R)) * 60.0 / (2.0 * 3.14);
        if (Vr * Vl < 0) {
            Vl = -((2 * Speed + Angle * L) / (2 * R)) * 60 / (2 * 3.14);
            Vr = -((2 * Speed - Angle * L) / (2 * R)) * 60 / (2 * 3.14);
        }
    } else if (sscanf(static_buf, "mode:auto,X:\"%f\",Y:\"%f\",Xobs:\"%f\",Yobs:\"%f\",Xobs2:\"%f\",Ybos2:\"%f\"",
                   &xGoal, &yGoal, &xObstacle, &yObstacle, &xObstacle1, &yObstacle1)
        && len != 0) {
        queue1.cancel(rfThread);
        autoThread = queue1.call_every(100ms, &autoMode);
    } else if (sscanf(static_buf, "update,\"%d\":\"%f\"", &parameterOrder, &parameterValue) && len != 0) {
        switch (parameterOrder) {
        case 0:
            kGoal = parameterValue;
            printf("\n Kgoal Updated \n");
            break;
        case 1:
            kObstacle = parameterValue;
            printf("\n kObstacle Updated \n");
            break;
        case 2:
            kAngle = parameterValue;
            printf("\n kAngle Updated \n");
            break;
        case 3:
            dGoalMax = parameterValue;
            printf("\n dGoal Updated \n");
            break;
        case 4:
            dObstacleMax = parameterValue;
            printf("\n dObs Updated \n");
            break;
        case 5:
            vMax = parameterValue;
            printf("\n vMax Updated \n");
            break;
        case 6:
            vOMax = parameterValue;
            printf("\n vOMax Updated \n");
            break;
        }
    } else if (strcmp(static_buf, "resetPos") == 0) {
        resetOdometryValues();

    } else if (strcmp(static_buf, "rfMode") == 0) {
        queue1.cancel(autoThread);
        rfThread = queue1.call_every(100ms, &RFmode);

    } else if (strcmp(static_buf, "urgentStop") == 0) {
        queue1.cancel(rfThread);
        queue1.cancel(autoThread);
        Vl = 0;
        Vr = 0;
    }
    clear_static_Buffer();
    led3 = 0;
}
void ESPconfig()
{
    strcpy(snd, "+++");
    clearBuffer();
    SendCMD();
    wait_us(2000000);
    ClearSnd();
    strcpy(snd, "AT\r\n");
    clearBuffer();
    SendCMD();

    wait_us(2000000);
    ClearSnd();
    strcpy(snd, "AT+RST\r\n");
    clearBuffer();
    SendCMD();

    ClearSnd();
    wait_us(2000000);

    strcpy(snd, "AT+GMR\r\n");
    clearBuffer();
    SendCMD();
    ClearSnd();
    wait_us(2000000);
    strcpy(snd, "AT+CWMODE=2\r\n");
    clearBuffer();
    SendCMD();
    ClearSnd();
    wait_us(2000000);

    strcpy(snd, "AT+CWSAP=\"");
    strcat(snd, ssid);
    strcat(snd, "\",\"");
    strcat(snd, pwd);
    strcat(snd, "\",5,3\r\n\"");
    clearBuffer();
    SendCMD();
    ClearSnd();
    wait_us(2000000);

    strcpy(snd, "AT+CIPSTART=\"UDP\",\"192.168.4.2\",4444,5555\r\n");
    clearBuffer();
    SendCMD();
    ClearSnd();
    wait_us(2000000);

    strcpy(snd, "AT+CIPMODE=1\r\n");
    clearBuffer();
    SendCMD();
    ClearSnd();
    wait_us(2000000);

    strcpy(snd, "AT+CIPSEND\r\n");
    clearBuffer();
    SendCMD();
    ClearSnd();
    wait_us(2000000);

    clearBuffer();
    clear_static_Buffer();
}

void SendCMD() { ESP32.write(snd, strlen(snd)); }
void clearBuffer()
{
    memset(buf, '\0', sizeof(buf));
    countt = 0;
}
void clear_static_Buffer() { memset(static_buf, '\0', sizeof(static_buf)); }
void ClearSnd() { memset(snd, '\0', sizeof(snd)); }
void resetVariables()
{
    memset(a, '\0', sizeof(a));
    memset(b, '\0', sizeof(b));
    memset(c, '\0', sizeof(c));
    memset(d, '\0', sizeof(d));
    memset(e, '\0', sizeof(e));
    memset(f, '\0', sizeof(f));
    memset(g, '\0', sizeof(g));
    memset(h, '\0', sizeof(h));
    memset(i, '\0', sizeof(i));
    memset(j, '\0', sizeof(j));
    memset(k, '\0', sizeof(k));
    memset(l, '\0', sizeof(l));
    memset(m, '\0', sizeof(m));
    memset(n, '\0', sizeof(n));
    memset(o, '\0', sizeof(o));
    memset(Status, '\0', sizeof(Status));
}
void resetOdometryValues()
{
    robotArrived = 0;
    Angle = 0.0f;
    Speed = 0.0f;
    Xpos = 0.0f;
    Ypos = 0.0f;
    Phi = 0.0f;
    xGoal = 0.0f;
    yGoal = 0.0f;
    dGoal = 0.0f;
    dObstacle1 = 0.0f;
    dObstacle = 0.0f;
    normalizedAngle = 0.0f;
}
void read_rc()
{
    ii = 0;
    jj = 0;
    kk = 0;
    for (kk = 0; kk <= 18; kk++) {
        if (ch1[kk] > 8000) {
            jj = kk + 1;
            break;
        }
    } // detecting separation space 10000us in that another array
    for (ii = 0; ii <= 7; ii++) {
        chh[ii] = (ch1[ii + jj]);
    }
}
