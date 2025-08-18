#include "LinearActuator.h"

LinearActuator::LinearActuator(uint8_t extendPin, uint8_t retractPin, uint8_t feedbackPin)
    : _extendPin(extendPin), _retractPin(retractPin), _feedbackPin(feedbackPin),
      _isReady(false), _minPosition(0), _maxPosition(300), _timeout(60000), _lastTime(0), _minRead(0), _maxRead(4096),
      _target(0), _tolerance(5) {}

void LinearActuator::begin() {
    pinMode(_extendPin, OUTPUT);
    pinMode(_retractPin, OUTPUT);
    stop(); // Ensure the actuator is stopped initially
}

void LinearActuator::extend() {
    digitalWrite(_extendPin, HIGH);
    digitalWrite(_retractPin, LOW);
    _isReady = false;
    _lastTime = millis();
}

void LinearActuator::retract() {
    digitalWrite(_extendPin, LOW);
    digitalWrite(_retractPin, HIGH);
    _isReady = false;
    _lastTime = millis();
}

void LinearActuator::stop() {
    digitalWrite(_extendPin, LOW);
    digitalWrite(_retractPin, LOW);
    delay(10);
    _target = readPosition();
    _isReady = true;
}

int LinearActuator::getTarget(){
	return _target;
}

bool LinearActuator::isReady() {
    return _isReady;
}

int LinearActuator::setMin(int pos){ //set calibrated min position
	if(pos < 0){
		_minRead = analogRead(_feedbackPin);
	}else {
		_minRead = pos;
	}
	return _minRead;
}

int LinearActuator::setMax(int pos){ //set calibrated max position
	if(pos < 0){
		_maxRead = analogRead(_feedbackPin);
	}else {
		_maxRead = pos;
	}
	return _maxRead;
}

void LinearActuator::setLength(int length){
	_maxPosition = length;
}

void LinearActuator::setTimeOut(int timeout){ //set timeout
	_timeout = timeout;
}

void LinearActuator::setPosition(int position){ //goto position
	if(isReady()){
		_target = position;
		//_lastTime = millis();
		if(_target - readPosition() > 0){
			extend();
		}else {
			retract();
		}
	}
}

void LinearActuator::update(){ //update actuator status
	if(millis() - _lastTime > _timeout){
		stop();
	}else {
		if (abs(_target - readPosition()) < _tolerance){
			stop();
		}
	}
	delay(10);
}

void LinearActuator::setTolerance(int tolerance){
	_tolerance = tolerance;
}

int LinearActuator::readPosition() {
    return map(analogRead(_feedbackPin), _minRead, _maxRead	, _minPosition, _maxPosition); // Read the position from the feedback pin
}

int LinearActuator::readRaw(){
	return analogRead(_feedbackPin);
}
