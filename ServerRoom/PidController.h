class PidController {
private:
    float Kp, Ki, Kd;
    float outMin, outMax;
    float setpoint;
    float currentTemp;
    float dt;
    float integralSum;
    float lastError;

    float getError();
    void antiWindup(float& integralSum);

public:
    PidController(float Kp, float Ki, float Kd, float outMin, float outMax);

    float getProportional();
    float getIntegral();
    float getDerivative();

    float compute(float setpoint, float currentTemp, float dt);
};