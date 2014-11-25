#include<R2WD_ROB5.h>

R2WD_ROB5::R2WD_ROB5(MotorWheel* wheelLeft,MotorWheel* wheelRight,unsigned int wheelspanMM):
			_wheelLeft(wheelLeft),_wheelRight(wheelRight),_wheelspanMM(wheelspanMM) {
	setSwitchMotorsStat(MOTORS_FB);
}

unsigned int R2WD_ROB5::getWheelspanMM() const {
	return _wheelspanMM;
}
unsigned int R2WD_ROB5::setWheelspanMM(unsigned int wheelspanMM) {
	_wheelspanMM=wheelspanMM;
	return getWheelspanMM();
}

unsigned char R2WD_ROB5::getSwitchMotorsStat() const {
	return _switchMotorsStat;
}
unsigned char R2WD_ROB5::setSwitchMotorsStat(unsigned char switchMotorsStat) {
	if(MOTORS_FB<=switchMotorsStat && switchMotorsStat<=MOTORS_BF)
		_switchMotorsStat=switchMotorsStat;
	return getSwitchMotorsStat();
}
unsigned char R2WD_ROB5::switchMotors() {
	if(getSwitchMotorsStat()==MOTORS_FB) setSwitchMotorsStat(MOTORS_BF);
	else setSwitchMotorsStat(MOTORS_FB);
	MotorWheel* temp=_wheelRight;
	_wheelRight=_wheelLeft;
	_wheelLeft=temp;
	return getSwitchMotorsStat();
}
unsigned char R2WD_ROB5::switchMotorsReset() {
	if(getSwitchMotorsStat()==MOTORS_BF) switchMotors();
	return getSwitchMotorsStat();
}

unsigned char R2WD_ROB5::getCarStat() const {
	return _carStat;
}
unsigned char R2WD_ROB5::setCarStat(unsigned char carStat) {
    if(STAT_UNKNOWN<=carStat && carStat<=STAT_UPPERRIGHT)
        return _carStat=carStat;
    else
        return STAT_UNKNOWN;
}

unsigned int R2WD_ROB5::getRadiusMM() const {
	switch(getCarStat()) {
		case STAT_ADVANCE:
		case STAT_BACKOFF:
			return 0; break;
		case STAT_ROTATELEFT:
		case STAT_ROTATERIGHT:
			return getWheelspanMM()>>1; break;
	}
	return _radiusMM;
}
unsigned int R2WD_ROB5::setRadiusMM(unsigned int radiusMM) {
	_radiusMM=radiusMM;
	return getRadiusMM();
}

unsigned int R2WD_ROB5::wheelRightSetSpeedMMPS(unsigned int speedMMPS,bool dir) {
	return _wheelRight->setSpeedMMPS(speedMMPS,dir);
}
unsigned int R2WD_ROB5::wheelRightGetSpeedMMPS() const {
	return _wheelRight->getSpeedMMPS();
}
unsigned int R2WD_ROB5::wheelLeftSetSpeedMMPS(unsigned int speedMMPS,bool dir) {
	return _wheelLeft->setSpeedMMPS(speedMMPS,dir);
}
unsigned int R2WD_ROB5::wheelLeftGetSpeedMMPS() const {
	return _wheelLeft->getSpeedMMPS();
}


unsigned int R2WD_ROB5::setMotorAll(unsigned int speedMMPS,bool dir) {
	wheelLeftSetSpeedMMPS(speedMMPS,dir);
	wheelRightSetSpeedMMPS(speedMMPS,dir);
	return wheelRightGetSpeedMMPS();
}
unsigned int R2WD_ROB5::setMotorAllStop() {
	return setMotorAll(0,DIR_ADVANCE);
}
unsigned int R2WD_ROB5::setMotorAllAdvance(unsigned int speedMMPS) {
	return setMotorAll(speedMMPS,DIR_ADVANCE);
}
unsigned int R2WD_ROB5::setMotorAllBackoff(unsigned int speedMMPS) {
	return setMotorAll(speedMMPS,DIR_BACKOFF);
}
unsigned int R2WD_ROB5::setCarStop() {
	setCarStat(STAT_STOP);
	return setMotorAll(0,DIR_ADVANCE);
}

unsigned int R2WD_ROB5::setCarAdvanceBase(unsigned int speedMMPSL,unsigned int speedMMPSR) {
	wheelLeftSetSpeedMMPS(speedMMPSL,DIR_ADVANCE);
	wheelRightSetSpeedMMPS(speedMMPSR,DIR_BACKOFF);
	return getCarSpeedMMPS();
}
unsigned int R2WD_ROB5::setCarBackoffBase(unsigned int speedMMPSL,unsigned int speedMMPSR) {
	wheelLeftSetSpeedMMPS(speedMMPSL,DIR_BACKOFF);
	wheelRightSetSpeedMMPS(speedMMPSR,DIR_ADVANCE);
	return getCarSpeedMMPS();
}

unsigned int R2WD_ROB5::setCarAdvance(unsigned int speedMMPS) {
	setCarStat(STAT_ADVANCE);
	return setCarAdvanceBase(speedMMPS,speedMMPS);
}
unsigned int R2WD_ROB5::setCarBackoff(unsigned int speedMMPS) {
	setCarStat(STAT_BACKOFF);
	return setCarBackoffBase(speedMMPS,speedMMPS);
}

unsigned int R2WD_ROB5::setCarRotateLeft(unsigned int speedMMPS) {
	setCarStat(STAT_ROTATELEFT);
	return setMotorAllBackoff(speedMMPS);
}
unsigned int R2WD_ROB5::setCarRotateRight(unsigned int speedMMPS) {
	setCarStat(STAT_ROTATERIGHT);
	return setMotorAllAdvance(speedMMPS);
}

unsigned int R2WD_ROB5::setCarArcBase(unsigned int speedMMPS,unsigned int radiusMM) {
	unsigned int delta=(int)((float)getWheelspanMM()/(radiusMM<<1)*speedMMPS);
	unsigned int V1=speedMMPS-delta;
	unsigned int V2=speedMMPS+delta;

	setRadiusMM(radiusMM);
	switch(getCarStat()) {
		case STAT_UPPERLEFT:
			setCarAdvanceBase(V1,V2); break;
		case STAT_LOWERLEFT:
			setCarBackoffBase(V1,V2); break;
		case STAT_UPPERRIGHT:
			setCarAdvanceBase(V2,V1); break;
		case STAT_LOWERRIGHT:
			setCarBackoffBase(V2,V1); break;
	}
	return getCarSpeedMMPS();
}


unsigned int R2WD_ROB5::setCarUpperLeft(unsigned int speedMMPS,unsigned int radiusMM) {
	setCarStat(STAT_UPPERLEFT);
	return setCarArcBase(speedMMPS,radiusMM);
}
unsigned int R2WD_ROB5::setCarLowerLeft(unsigned int speedMMPS,unsigned int radiusMM) {
	setCarStat(STAT_LOWERLEFT);
	return setCarArcBase(speedMMPS,radiusMM);
}
unsigned int R2WD_ROB5::setCarLowerRight(unsigned int speedMMPS,unsigned int radiusMM) {
	setCarStat(STAT_LOWERRIGHT);
	return setCarArcBase(speedMMPS,radiusMM);
}
unsigned int R2WD_ROB5::setCarUpperRight(unsigned int speedMMPS,unsigned int radiusMM) {
	setCarStat(STAT_UPPERRIGHT);
	return setCarArcBase(speedMMPS,radiusMM);
}


unsigned int R2WD_ROB5::getCarSpeedMMPS() const {
	unsigned int speedMMPSL=wheelLeftGetSpeedMMPS();
	unsigned int speedMMPSR=wheelRightGetSpeedMMPS();
	return (speedMMPSL+speedMMPSR)>>1;
}
unsigned int R2WD_ROB5::setCarSpeedMMPS(unsigned int speedMMPS,unsigned int ms) {
	unsigned int carStat=getCarStat();
	unsigned int currSpeed=getCarSpeedMMPS();

	unsigned int (R2WD_ROB5::*carAction)(unsigned int speedMMPS);
	switch(carStat) {
		case STAT_ADVANCE:
			carAction=&R2WD_ROB5::setCarAdvance; break;
		case STAT_BACKOFF:
			carAction=&R2WD_ROB5::setCarBackoff; break;
		case STAT_ROTATELEFT:
			carAction=&R2WD_ROB5::setCarRotateLeft; break;
		case STAT_ROTATERIGHT:
			carAction=&R2WD_ROB5::setCarRotateRight; break;
		case STAT_UPPERLEFT:
		case STAT_LOWERLEFT:
		case STAT_LOWERRIGHT:
		case STAT_UPPERRIGHT:
			return setCarSpeedMMPSArc(speedMMPS,getRadiusMM(),ms);break;
		default:
			return currSpeed; break;
	}

	if(ms<100 || abs(speedMMPS-currSpeed)<10) {
		return (this->*carAction)(speedMMPS);
	}

	for(int time=0,speed=currSpeed;time<=ms;time+=50) {
		speed=map(time,0,ms,currSpeed,speedMMPS);
		(this->*carAction)(speed);
		delayMS(50);
	}

	(this->*carAction)(speedMMPS);
	return getCarSpeedMMPS();
}

unsigned int R2WD_ROB5::setCarSpeedMMPSArc(unsigned int speedMMPS,unsigned int radiusMM,unsigned int ms) {
	unsigned int carStat=getCarStat();
	unsigned int currSpeed=getCarSpeedMMPS();

	unsigned int (R2WD_ROB5::*carAction)(unsigned int speedMMPS,unsigned int radiusMM);
	switch(carStat) {
		case STAT_ADVANCE:
		case STAT_BACKOFF:
		case STAT_ROTATELEFT:
		case STAT_ROTATERIGHT:
			return setCarSpeedMMPS(speedMMPS,ms); break;
		case STAT_UPPERLEFT:
			carAction=&R2WD_ROB5::setCarUpperLeft; break;
		case STAT_LOWERLEFT:
			carAction=&R2WD_ROB5::setCarLowerLeft; break;
		case STAT_LOWERRIGHT:
			carAction=&R2WD_ROB5::setCarLowerRight; break;
		case STAT_UPPERRIGHT:
			carAction=&R2WD_ROB5::setCarUpperRight; break;
		default:
			return currSpeed; break;
	}

	if(ms<100 || abs(speedMMPS-currSpeed)<10) {
		return (this->*carAction)(speedMMPS,radiusMM);
	}

	for(int time=0,speed=currSpeed;time<=ms;time+=50) {
		speed=map(time,0,ms,currSpeed,speedMMPS);
		(this->*carAction)(speed,radiusMM);
		delayMS(50);
	}

	(this->*carAction)(speedMMPS,radiusMM);
	return getCarSpeedMMPS();
}


unsigned int R2WD_ROB5::setCarSlow2Stop(unsigned int ms) {
	unsigned char carStat=getCarStat();
	//if(STAT_UNKNOWN<=STAT_UPPERLEFT || carStat<=STAT_LOWERRIGHT) {
	if(STAT_UPPERLEFT<=carStat && carStat<=STAT_UPPERRIGHT) {
		return setCarSpeedMMPSArc(0,getRadiusMM(),ms);
	}
	return setCarSpeedMMPS(0,ms);
}
 

bool R2WD_ROB5::PIDEnable(float kc,float taui,float taud,unsigned int interval) {
	return _wheelLeft->PIDEnable(kc,taui,taud,interval) && 
			_wheelRight->PIDEnable(kc,taui,taud,interval);
}
bool R2WD_ROB5::PIDRegulate() {
	return _wheelLeft->PIDRegulate() && _wheelRight->PIDRegulate();
}
/*
void R2WD_ROB5::delayMS(unsigned long ms,bool debug) {
	for(int i=0;i<ms;i+=SAMPLETIME) {
		PIDRegulate();
		if(debug && (i%500==0)) debugger();
		delay(SAMPLETIME);
	}
}
 */
void R2WD_ROB5::delayMS(unsigned int ms,bool debug) {
	//for(unsigned long endTime=millis()+ms;millis()<endTime;) {
        if(millis()% SAMPLETIME == 0)
            PIDRegulate();
        //}
		//if(endTime-millis()>=SAMPLETIME) delay(SAMPLETIME);
		//else delay(endTime-millis());
	//}
}


void R2WD_ROB5::RUN_regulate() {
    if(millis()% SAMPLETIME == 0)
        PIDRegulate();
    }

void R2WD_ROB5::demoActions(unsigned int speedMMPS,unsigned int duration,unsigned int uptime,bool debug) {
	unsigned int (R2WD_ROB5::*carAction[])(unsigned int speedMMPS)={
		&R2WD_ROB5::setCarAdvance,
		&R2WD_ROB5::setCarBackoff,
		&R2WD_ROB5::setCarRotateLeft,
		&R2WD_ROB5::setCarRotateRight,
	};
	unsigned int (R2WD_ROB5::*carAction2[])(unsigned int speedMMPS,unsigned int radiusMM)={
		&R2WD_ROB5::setCarUpperLeft,
		&R2WD_ROB5::setCarLowerLeft,
		&R2WD_ROB5::setCarUpperRight,
		&R2WD_ROB5::setCarLowerRight,
	};

	for(int i=0;i<8;++i) {
		if(i<4) {
		Serial.println(i,DEC);
			(this->*carAction[i])(0);
			setCarSpeedMMPS(speedMMPS,uptime);
		} else {
			(this->*carAction2[i-4])(0,500);
			setCarSpeedMMPSArc(speedMMPS,getRadiusMM(),uptime);
		}
		delayMS(duration,debug);
		setCarSlow2Stop(uptime);
	}
    //setCarStop();
    //delayMS(duration,debug);
    //switchMotors();
}


unsigned int R2WD_ROB5::setCarArcTime(unsigned int speedMMPS,unsigned int radiusMM,
			unsigned long duration,unsigned int uptime) {
	setCarSpeedMMPSArc(speedMMPS,radiusMM,uptime);
	delayMS(duration);
	setCarSlow2Stop(uptime);
	return uptime<<1+duration;
}
unsigned int R2WD_ROB5::setCarUpperLeftTime(unsigned int speedMMPS,unsigned int radiusMM,
			unsigned long duration,unsigned int uptime) {
	setCarUpperLeft(0,radiusMM);
	return setCarArcTime(speedMMPS,getRadiusMM(),duration,uptime);
}
unsigned int R2WD_ROB5::setCarLowerLeftTime(unsigned int speedMMPS,unsigned int radiusMM,
			unsigned long duration,unsigned int uptime) {
	setCarLowerLeft(0,radiusMM);
	return setCarArcTime(speedMMPS,getRadiusMM(),duration,uptime);
}
unsigned int R2WD_ROB5::setCarUpperRightTime(unsigned int speedMMPS,unsigned int radiusMM,
			unsigned long duration,unsigned int uptime) {
	setCarUpperRight(0,radiusMM);
	return setCarArcTime(speedMMPS,getRadiusMM(),duration,uptime);
}
unsigned int R2WD_ROB5::setCarLowerRightTime(unsigned int speedMMPS,unsigned int radiusMM,
			unsigned long duration,unsigned int uptime) {
	setCarLowerRight(0,radiusMM);
	return setCarArcTime(speedMMPS,getRadiusMM(),duration,uptime);
}
 
/*
unsigned int R2WD_ROB5::setCarArcAngle(unsigned int speedMMPS,unsigned int radiusMM,float radian,unsigned int uptime) {
	if(radian<0) return 0;
	unsigned int (R2WD_ROB5::*carAction)(unsigned int speedMMPS,unsigned int radiusMM);
	unsigned long timeMS=(long)(radiusMM*radian/speedMMPS*1000);
	switch(getCarStat()) {
		case STAT_UPPERLEFT:
			carAction=&R2WD_ROB5::setCarUpperLeft; break;
		case STAT_LOWERLEFT:
			carAction=&R2WD_ROB5::setCarLowerLeft; break;
		case STAT_UPPERRIGHT:
			carAction=&R2WD_ROB5::setCarUpperRight; break;
		case STAT_LOWERRIGHT:
			carAction=&R2WD_ROB5::setCarLowerRight; break;
		default:
			return getCarSpeedMMPS(); break;
	}
	setCarSpeedMMPSArc(speedMMPS,radiusMM,uptime);
	//(this->*carAction)(speedMMPS,radiusMM);
	delayMS(timeMS-uptime);
	setCarSlow2Stop(uptime);
	return timeMS;
} 
 */
unsigned int R2WD_ROB5::setCarArcAngle(unsigned int speedMMPS,unsigned int radiusMM,float radian,unsigned int uptime) {
	if(radian<0) return 0;
	unsigned long timeMS=(long)(radiusMM*radian/speedMMPS*1000);
	setCarSpeedMMPSArc(speedMMPS,radiusMM,uptime);
	delayMS(timeMS-uptime);
	setCarSlow2Stop(uptime);
	return timeMS;
}
unsigned int R2WD_ROB5::setCarUpperLeftAngle(unsigned int speedMMPS,unsigned int radiusMM,float radian,unsigned int uptime) {
	setCarUpperLeft(0,radiusMM);
	return setCarArcAngle(speedMMPS,getRadiusMM(),radian,uptime);
}
unsigned int R2WD_ROB5::setCarLowerLeftAngle(unsigned int speedMMPS,unsigned int radiusMM,float radian,unsigned int uptime) {
	setCarLowerLeft(0,radiusMM);
	return setCarArcAngle(speedMMPS,getRadiusMM(),radian,uptime);
}
unsigned int R2WD_ROB5::setCarUpperRightAngle(unsigned int speedMMPS,unsigned int radiusMM,float radian,unsigned int uptime) {
	setCarUpperRight(0,radiusMM);
	return setCarArcAngle(speedMMPS,getRadiusMM(),radian);
}
unsigned int R2WD_ROB5::setCarLowerRightAngle(unsigned int speedMMPS,unsigned int radiusMM,float radian,unsigned int uptime) {
	setCarLowerRight(0,radiusMM);
	return setCarArcAngle(speedMMPS,getRadiusMM(),radian,uptime);
}


unsigned int R2WD_ROB5::setCarRotateAngle(unsigned int speedMMPS,float radian) {
	if(radian<0) return 0;
	unsigned long timeMS=(long)((getWheelspanMM()>>1)*radian/speedMMPS*1000);
	delayMS(timeMS);
	return timeMS;
}
unsigned int R2WD_ROB5::setCarRotateLeftAngle(unsigned int speedMMPS,float radian) {
	setCarRotateLeft(speedMMPS);
	return setCarRotateAngle(speedMMPS,radian);
}
unsigned int R2WD_ROB5::setCarRotateRightAngle(unsigned int speedMMPS,float radian) {
	setCarRotateRight(speedMMPS);
	return setCarRotateAngle(speedMMPS,radian);
}

unsigned int R2WD_ROB5::setCarStraightDistance(unsigned int speedMMPS,unsigned long distance) {
	unsigned long timeMS=distance*1000/speedMMPS;
	delayMS(timeMS);
	return timeMS;
}
unsigned int R2WD_ROB5::setCarAdvanceDistance(unsigned int speedMMPS,unsigned long distance) {
	setCarAdvance(speedMMPS);
	return setCarStraightDistance(speedMMPS,distance);
}
unsigned int R2WD_ROB5::setCarBackoffDistance(unsigned int speedMMPS,unsigned long distance) {
	setCarBackoff(speedMMPS);
	return setCarStraightDistance(speedMMPS,distance);
}

void R2WD_ROB5::debugger(bool wheelLeftDebug,bool wheelRightDebug) const {
	if(wheelLeftDebug) _wheelLeft->debugger();
	if(wheelRightDebug) _wheelRight->debugger();
}


unsigned long R2WD_ROB5::getRotationTime(unsigned int speedMMPS,float radian){
    unsigned long timeMS = (long)((getWheelspanMM()>>1)*radian/speedMMPS*1000);
    return timeMS;
}
unsigned long R2WD_ROB5::getStraightTime(unsigned int speedMMPS, unsigned long distance){
    unsigned long timeMS = distance*1000/speedMMPS;
    return timeMS;
}

