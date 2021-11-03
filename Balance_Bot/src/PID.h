#define PID_MAX 30
volatile double elapsedTime, time1, timePrev;



volatile typedef struct
{
    double KP_XY = 9;
    double KI_XY = 400; //350
    double KD_XY = 0.4;

    float KP_Z = 2;
    float KI_Z = 0;
    float KD_Z = .1;
} PID_const;
PID_const pid;

class PID_NOT
{
private:
    float PID_Output, error, previous_error;
    float pid_p = 0;
    float pid_i = 0;
    float pid_d = 0;
    float KP = 0;
    float KI = 0;
    float KD = 0;

public:
    void Set_Const(float kp, float ki, float kd);
    int Calc(float angle_value, float Target);
    PID_NOT(float kp,float ki,float kd);
    ~PID_NOT();
};

PID_NOT::PID_NOT(float kp,float ki,float kd)
{
    KP = kp;
    KI = ki;
    KD = kd;
}

PID_NOT::~PID_NOT()
{
}

void PID_NOT::Set_Const(float kp, float ki, float kd)
{
    KP = kp;
    KI = ki;
    KD = kd;
}

int PID_NOT::Calc(float angle_value, float Target)
{

    error = angle_value - Target;
    
    pid_p = KP * error;

    if (-3 < error && error < 3)
        pid_i = pid_i + (KI * error);
    else
        pid_i = 0;

    pid_d = KD * ((error - previous_error) / elapsedTime);

    PID_Output = pid_p + pid_i + pid_d;
    
    PID_Output = constrain(PID_Output, -PID_MAX, PID_MAX);
    previous_error = error;
    
    return (int)PID_Output;
}
