
class PidController {
private:
	float Kp, Ki, Kd; // PID coefficients
	float outMin, outMax; // PWM Output limits (0...255)
	float setpoint; // Target temperature 
	float currentTemp; // Current temperature
	float dt; // Time difference between updates
	float integralSum; // Integral sum for the integral term
	float lastError; // Last error for the derivative term

public:
	PidController(float Kp, float Ki, float Kd, float outMin, float outMax, float setpoint, float currentTemp, float dt, float integralSum, float lastError) {
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
		this->outMin = outMin;
		this->outMax = outMax;
		this->setpoint = setpoint;
		this->currentTemp = currentTemp;
		this->dt = dt; // Default time difference
		this->integralSum = integralSum;
		this->lastError = lastError;
	};

private:
	float getError() {
		return currentTemp - setpoint;
	};

public: 
	float getProportional() {
		return Kp * getError();
	};

	float getIntegral() {
		integralSum += getError() * dt;
		antiWindup(integralSum);
		return Ki * integralSum;
	};

	float getDerivative() {
		if (dt <= 0.0f) return 0.0f; // Prevent division by zero)
		return Kd * (getError() - lastError) / dt;
	};

	float compute(float setpoint, float currentTemp, float dt) {
		this->setpoint = setpoint;
		this->currentTemp = currentTemp;
		this->dt = dt;
		float error = getError();
		float pTerm = getProportional();
		float iTerm = getIntegral();
		float dTerm = getDerivative();

		float output = pTerm + iTerm + dTerm;

		// Clamp output to PWM limits
		if (output < outMin) output = outMin;
		if (output > outMax) output = outMax;

		lastError = error;

		return output;
	}
private:
	// Anti-windup: Clamp integral sum to output limits
	void antiWindup(float& integralSum) {
		if (Ki > 0.0f) {
			float maxIntegral = outMax / Ki;
			float minIntegral = outMin / Ki;
			if (integralSum < minIntegral) integralSum = minIntegral;
			else if (integralSum > maxIntegral) integralSum = maxIntegral;
		}
	}
};
 