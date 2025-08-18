#ifndef LINEAR_ACTUATOR_H
#define LINEAR_ACTUATOR_H

#include <Arduino.h>

class LinearActuator {
public:
    LinearActuator(uint8_t extendPin, uint8_t retractPin, uint8_t feedbackPin);
    void begin();
    void extend();
    void retract();
    void stop();
    //bool isExtended();
    //bool isRetracted();
    bool isReady();
    int setMin(int pos); //set calibrated min position
    int setMax(int pos); //set calibrated max position
    void setLength(int length);
    void setTimeOut(int timeout); //set timeout
    void setPosition(int position); //goto position
    void update(); //actuator update
    int readPosition(); // New method to read the position
    int getTarget();
    void setTolerance(int tolerance);
    int readRaw();

private:
    uint8_t _extendPin;
    uint8_t _retractPin;
    uint8_t _feedbackPin; // New member for the feedback pin
    //bool _isExtended;
    //bool _isRetracted;
    bool _isReady;
    int _minPosition;
	int _maxPosition;
	int _timeout;
	int _lastTime;
	int _minRead;
	int _maxRead;
	int _target;
	int _tolerance;
};

#endif // LINEAR_ACTUATOR_H
