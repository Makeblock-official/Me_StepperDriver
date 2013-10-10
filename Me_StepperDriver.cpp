/******************************************************************************
* File Name          : Me_StepperDriver.cpp
* Author             : Evan
* Updated            : Evan
* Version            : V0.1.2
* Date               : 10/08/2013
* Description        : Class for Makeblock Electronic modules of Me-Stepper
                       Driver. The module can only be connected to the PORT_1, 
                       PORT_2 of Me-Base Shield.
* License            : CC-BY-SA 3.0, GPL 
* Copyright (C) 2011 Hulu Robot Technology Co., Ltd. All right reserved.
*******************************************************************************/

#include "Me_StepperDriver.h"

Me_StepperDriver::Me_StepperDriver()
{
	s1Pin = mePort[PORT_1].innersidePin;
	s2Pin = mePort[PORT_1].outsidePin;
}

Me_StepperDriver::Me_StepperDriver(int portNum)
{
	if(portNum>0 && portNum < 3 )
	{
		s1Pin = mePort[portNum].innersidePin;
		s2Pin = mePort[portNum].outsidePin;
	}
}

void Me_StepperDriver::begin()
{
	Me_BaseShield::begin();
//	pinMode(s1Pin,OUTPUT);
//	pinMode(s2Pin,OUTPUT);
//	digitalWrite(s1Pin,HIGH);
//	digitalWrite(s2Pin,LOW);
	delay(1500); //wait Me_StepperDriver initialize /limit:1400
	Wire.begin(); // join i2c bus (address optional for master)
	slaveAddress = 0x04;
	setCurrentPosition(0);
}

void Me_StepperDriver::setMicroStep(byte microStep)
{
  twi_write(slaveAddress, STP_MS_CTRL, microStep);
}

void Me_StepperDriver::reset()
{
  twi_write(slaveAddress, STP_RUN_CTRL, STP_RESET_CTRL);
}

void Me_StepperDriver::moveTo(long stepperMoveTo)
{
  twi_write(slaveAddress, STP_MOVE_TO_L1, *((char *)(&stepperMoveTo)));
  twi_write(slaveAddress, STP_MOVE_TO_L2, *((char *)(&stepperMoveTo) + 1));
  twi_write(slaveAddress, STP_MOVE_TO_H1, *((char *)(&stepperMoveTo) + 2));
  twi_write(slaveAddress, STP_MOVE_TO_H2, *((char *)(&stepperMoveTo) + 3));
}

void Me_StepperDriver::move(long stepperMove)
{
  twi_write(slaveAddress, STP_MOVE_L1, *((char *)(&stepperMove)));
  twi_write(slaveAddress, STP_MOVE_L2, *((char *)(&stepperMove) + 1));
  twi_write(slaveAddress, STP_MOVE_H1, *((char *)(&stepperMove) + 2));
  twi_write(slaveAddress, STP_MOVE_H2, *((char *)(&stepperMove) + 3));
}

void Me_StepperDriver::runSpeed()
{
  twi_write(slaveAddress, STP_RUN_CTRL, STP_RUN_SPEED);
}

void Me_StepperDriver::setMaxSpeed(long stepperMaxSpeed)
{
  twi_write(slaveAddress, STP_MAX_SPEED_L1, *((char *)(&stepperMaxSpeed)));
  twi_write(slaveAddress, STP_MAX_SPEED_L2, *((char *)(&stepperMaxSpeed) + 1));
  twi_write(slaveAddress, STP_MAX_SPEED_H1, *((char *)(&stepperMaxSpeed) + 2));
  twi_write(slaveAddress, STP_MAX_SPEED_H2, *((char *)(&stepperMaxSpeed) + 3));
}

void Me_StepperDriver::setAcceleration(long stepperAcceleration)
{
  twi_write(slaveAddress, STP_ACC_L1, *((char *)(&stepperAcceleration)));
  twi_write(slaveAddress, STP_ACC_L2, *((char *)(&stepperAcceleration) + 1));
  twi_write(slaveAddress, STP_ACC_H1, *((char *)(&stepperAcceleration) + 2));
  twi_write(slaveAddress, STP_ACC_H2, *((char *)(&stepperAcceleration) + 3));
}

void Me_StepperDriver::setSpeed(long stepperSpeed)
{
  twi_write(slaveAddress, STP_SPEED_L1, *((char *)(&stepperSpeed)));
  twi_write(slaveAddress, STP_SPEED_L2, *((char *)(&stepperSpeed) + 1));
  twi_write(slaveAddress, STP_SPEED_H1, *((char *)(&stepperSpeed) + 2));
  twi_write(slaveAddress, STP_SPEED_H2, *((char *)(&stepperSpeed) + 3));
}

long Me_StepperDriver::speed()
{
  *((char *)(&stepperSpeedRead))   = twi_read(slaveAddress, STP_SPEED_RL1);
  *((char *)(&stepperSpeedRead)+1) = twi_read(slaveAddress, STP_SPEED_RL2);
  *((char *)(&stepperSpeedRead)+2) = twi_read(slaveAddress, STP_SPEED_RH1);
  *((char *)(&stepperSpeedRead)+3) = twi_read(slaveAddress, STP_SPEED_RH2);
  return stepperSpeedRead;
}

long Me_StepperDriver::distanceToGo()
{
  *((char *)(&stepperDistanceToGoRead))   = twi_read(slaveAddress, STP_DIS_TOGO_RL1);
  *((char *)(&stepperDistanceToGoRead)+1) = twi_read(slaveAddress, STP_DIS_TOGO_RL2);
  *((char *)(&stepperDistanceToGoRead)+2) = twi_read(slaveAddress, STP_DIS_TOGO_RH1);
  *((char *)(&stepperDistanceToGoRead)+3) = twi_read(slaveAddress, STP_DIS_TOGO_RH2);
  return stepperDistanceToGoRead;
}

long Me_StepperDriver::targetPosition()
{
  *((char *)(&stepperTargetPositionRead))   = twi_read(slaveAddress, STP_TARGET_POS_RL1);
  *((char *)(&stepperTargetPositionRead)+1) = twi_read(slaveAddress, STP_TARGET_POS_RL2);
  *((char *)(&stepperTargetPositionRead)+2) = twi_read(slaveAddress, STP_TARGET_POS_RH1);
  *((char *)(&stepperTargetPositionRead)+3) = twi_read(slaveAddress, STP_TARGET_POS_RH2);
  return stepperTargetPositionRead;
}

long Me_StepperDriver::currentPosition()
{
  *((char *)(&stepperCurrentPositionRead))   = twi_read(slaveAddress, STP_CURRENT_POS_RL1);
  *((char *)(&stepperCurrentPositionRead)+1) = twi_read(slaveAddress, STP_CURRENT_POS_RL2);
  *((char *)(&stepperCurrentPositionRead)+2) = twi_read(slaveAddress, STP_CURRENT_POS_RH1);
  *((char *)(&stepperCurrentPositionRead)+3) = twi_read(slaveAddress, STP_CURRENT_POS_RH2);
  return stepperCurrentPositionRead;
}

void Me_StepperDriver::setCurrentPosition(long stepperCuttentPos)
{
  twi_write(slaveAddress, STP_CURRENT_POS_L1, *((char *)(&stepperCuttentPos)));
  twi_write(slaveAddress, STP_CURRENT_POS_L2, *((char *)(&stepperCuttentPos) + 1));
  twi_write(slaveAddress, STP_CURRENT_POS_H1, *((char *)(&stepperCuttentPos) + 2));
  twi_write(slaveAddress, STP_CURRENT_POS_H2, *((char *)(&stepperCuttentPos) + 3));
}

void Me_StepperDriver::enable()
{
  twi_write(slaveAddress, STP_EN_CTRL, STP_ENABLE);
}

void Me_StepperDriver::disable()
{
  twi_write(slaveAddress, STP_EN_CTRL, STP_DISABLE);
}

void Me_StepperDriver::run()
{
  twi_write(slaveAddress, STP_RUN_CTRL, STP_RUN);
}

void Me_StepperDriver::stop()
{
  twi_write(slaveAddress, STP_RUN_CTRL, STP_STOP);
}

void Me_StepperDriver::wait()
{
  twi_write(slaveAddress, STP_RUN_CTRL, STP_WAIT);
}

boolean Me_StepperDriver::readState()
{
  if(twi_read(slaveAddress, STP_RUN_STATE))
    return true;
  else
    return false;
}

// must write low first
byte Me_StepperDriver::twi_read(int slaveAddress, byte dataAddress)
{
  byte rxByte;
  Wire.beginTransmission(slaveAddress); // transmit to device
  Wire.write(dataAddress);  						// sends one byte
  Wire.endTransmission();    						// stop transmitting
  delayMicroseconds(1);
  Wire.requestFrom(slaveAddress, 1);    // request 1 bytes from slave device
  while(Wire.available())    						// slave may send less than requested
    return rxByte = Wire.read(); 				// receive a byte
  return 0;
}

void Me_StepperDriver::twi_write(int slaveAddress, byte dataAddress, byte data)
{
  Wire.beginTransmission(slaveAddress); // transmit to slave device
  Wire.write(dataAddress);  						// sends data address
  Wire.endTransmission();    						// stop transmitting

  Wire.beginTransmission(slaveAddress); // transmit to slave device
  Wire.write(data);  										// sends one byte data
  Wire.endTransmission();    						// stop transmitting
}