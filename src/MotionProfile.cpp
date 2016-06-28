#include "MotionProfile.h"


MotionProfile::MotionProfile(float aVelocityMax, float aAccelerationMax, short aMethod, int aSampleTime, int aPwmVelocityPin)
{
	this->maxVelocity = aVelocityMax;
	this->maxAcceleration = aAccelerationMax;
	this->method = aMethod;
	this->sampleTime = aSampleTime;
	this->pwmVelocityPin = aPwmVelocityPin;
	init();
}

void MotionProfile::init() {
	// Check the method
	if (0 > this->method || this->method > this->nrMethods - 1) {
		// Set trapezoidal as default
		this->method = 1;
	}
	
	//Set the pwm pin to output
	pinMode(this->pwmVelocityPin, OUTPUT);

	// Time variables
	this->lastTime = 0;
	this->delta = 0;
	
	// State variables
	reset();
	
	// Comparison precision
	this->compFactor = 6;
	
	// Misc
	this->isFinished = false;
}

float MotionProfile::update(float setpoint) {
	unsigned long now = millis();
	unsigned long timeChange = (now - lastTime);
	
	if(timeChange >= sampleTime) {	
		// Calculate elapsed time
		bool valid = timeCalculation();	

		// Check if finished
		isFinished = static_cast<long>(position * (10^compFactor)) == static_cast<long>(setpoint * (10^compFactor));
		//Serial.println(isFinished);
			
		if (!isFinished) {
			// Shift state variables
			oldPosition = position;
			oldVelocity = velocity;
		
			if (valid) {
				// Check what type of motion profile to use
				switch(method) {
					case 0:
						// Constant velocity profile
						calculateConstantVelocityProfile(setpoint);
						break;
					case 1:
						// Trapezoidal velocity profile
						calculateTrapezoidalProfile(setpoint);
						break;
				}
			}
		
			// Update state
			stateCalculation();
		}		
	}
	return position;
}

bool MotionProfile::timeCalculation() {
	// Get current time in microseconds
	long currentTime = micros();
	if (lastTime == 0) {
		lastTime = currentTime;
		return false;
	}
	
	// Delta is measured in seconds, thus divide by 1e6 because we're using microseconds.
	delta = static_cast<float>(currentTime - lastTime) / 1000000;
	lastTime = currentTime;
	return true;
}

void MotionProfile::stateCalculation() {
	// Contains NaN checks, see: https://stackoverflow.com/questions/570669/checking-if-a-double-or-float-is-nan-in-c
	// Calculate velocity
	velocity = (position - oldPosition) / delta;
	if (velocity != velocity) {
		velocity = 0;
	}
	
	// Calculate acceleration
	acceleration = (velocity - oldVelocity) / delta;
	if (acceleration != acceleration) {
		acceleration = 0;
	}
}

void MotionProfile::calculateConstantVelocityProfile(float setpoint) {
	float suggestedVelocity = (setpoint - position) / delta;
	
	if (suggestedVelocity > maxVelocity) {
		position += maxVelocity * delta;
	}
	else if (suggestedVelocity < -maxVelocity) {
		position += -maxVelocity * delta;
	}
	else {
		position += suggestedVelocity * delta;
	}
}

void MotionProfile::calculateTrapezoidalProfile(float setpoint) {
	// Check if we need to de-accelerate
	if (((velocity * velocity) / maxAcceleration) / 2 >= abs(setpoint - position)) {
		if (velocity < 0) {
			position += (velocity + maxAcceleration * delta) * delta;
		}
		else if (velocity > 0) {
			position += (velocity - maxAcceleration * delta) * delta;
		}
	}
	else {
		// We're not too close yet, so no need to de-accelerate. Check if we need to accelerate or maintain velocity.
		if (abs(velocity) < maxVelocity || (setpoint < position && velocity > 0) || (setpoint > position && velocity < 0)) {
			// We need to accelerate, do so but check the maximum acceleration.
			// Keep velocity constant at the maximum
			float suggestedVelocity = 0.0;
			if (setpoint > position) {
				suggestedVelocity = velocity + maxAcceleration * delta;
				if (suggestedVelocity > maxVelocity) {
					suggestedVelocity = maxVelocity;
				}
			}
			else {
				suggestedVelocity = velocity - maxAcceleration * delta;
				if (abs(suggestedVelocity) > maxVelocity) {
					suggestedVelocity = -maxVelocity;
				}				
			}
			position += suggestedVelocity * delta;
		}
		else {
			// Keep velocity constant at the maximum
			if (setpoint > position) {
				position += maxVelocity * delta;
			}
			else {
				position += -maxVelocity * delta;
			}
		}
	}
} 

void MotionProfile::pause() {
	lastTime = 0;
}

// Getters and setters
bool MotionProfile::getFinished() {
	return isFinished;
}

void MotionProfile::setMaxVelocity(float aMaxVelocity) {
	maxVelocity = aMaxVelocity;
}

void MotionProfile::setMaxAcceleration(float aMaxAcceleration) {
	maxAcceleration = aMaxAcceleration;
}

void MotionProfile::setCompFactor(int aFactor) {
	compFactor = aFactor;
}

void MotionProfile::reset() {
	// Reset all state variables
	position = 0;
	oldPosition = 0;
	velocity = 0;
	oldVelocity = 0;
	acceleration = 0;
}
