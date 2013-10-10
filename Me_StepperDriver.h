/******************************************************************************
* File Name          : Me_StepperDriver.h
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

#include <Arduino.h>
#include <Wire.h>
#include <Me_BaseShield.h>

#ifndef Me_StepperDriver_h
#define Me_StepperDriver_h

//address table
#define STP_RUN_STATE 0x91
#define STP_SPEED_RL1 0x92
#define STP_SPEED_RL2 0x93
#define STP_SPEED_RH1 0x94
#define STP_SPEED_RH2 0x95
#define STP_DIS_TOGO_RL1 0x96
#define STP_DIS_TOGO_RL2 0x97
#define STP_DIS_TOGO_RH1 0x98
#define STP_DIS_TOGO_RH2 0x99
#define STP_TARGET_POS_RL1 0x9A
#define STP_TARGET_POS_RL2 0x9B
#define STP_TARGET_POS_RH1 0x9C
#define STP_TARGET_POS_RH2 0x9D
#define STP_CURRENT_POS_RL1 0x9E
#define STP_CURRENT_POS_RL2 0x9F
#define STP_CURRENT_POS_RH1 0xA0
#define STP_CURRENT_POS_RH2 0xA1

#define STP_ACC_L1 0x01 //This is an expensive call since it requires a square root to be calculated. Don't call more ofthen than needed.
#define STP_ACC_L2 0x02
#define STP_ACC_H1 0x03
#define STP_ACC_H2 0x04
#define STP_MAX_SPEED_L1 0x05
#define STP_MAX_SPEED_L2 0x06
#define STP_MAX_SPEED_H1 0x07
#define STP_MAX_SPEED_H2 0x08
#define STP_SPEED_L1 0x09
#define STP_SPEED_L2 0x0A
#define STP_SPEED_H1 0x0B
#define STP_SPEED_H2 0x0C
#define STP_MOVE_TO_L1 0x0D
#define STP_MOVE_TO_L2 0x0E
#define STP_MOVE_TO_H1 0x0F
#define STP_MOVE_TO_H2 0x10
#define STP_MOVE_L1 0x11
#define STP_MOVE_L2 0x12
#define STP_MOVE_H1 0x13
#define STP_MOVE_H2 0x14
#define STP_CURRENT_POS_L1 0x15
#define STP_CURRENT_POS_L2 0x16
#define STP_CURRENT_POS_H1 0x17
#define STP_CURRENT_POS_H2 0x18

#define STP_RUN_CTRL 0x1C
#define STP_EN_CTRL 0x1D
#define STP_SLEEP_CTRL 0x1F
#define STP_MS_CTRL 0x20

//data table
#define STP_RUN 0x01
#define STP_STOP 0x02
#define STP_WAIT 0x03
#define STP_RESET_CTRL 0x04
#define STP_RUN_SPEED 0x05
#define STP_TRUE 0x01
#define STP_FALSE 0x00
#define STP_ENABLE 0x01
#define STP_DISABLE 0x02
#define STP_FULL 0x01
#define STP_HALF 0x02
#define STP_QUARTER 0x04
#define STP_EIGHTH 0x08
#define STP_SIXTEENTH 0x16

class Me_StepperDriver
{
public:
	Me_StepperDriver();
	//portNum can ONLY be PORT_1 or PORT_2
	Me_StepperDriver(int portNum);
	
	// initialize stepper driver.
	void begin();
	
	// set micro step. Mode: STP_FULL, STP_HALF, STP_QUARTER, STP_EIGHTH, STP_SIXTEENTH
	void setMicroStep(byte microStep);

	// stop stepper and reset current position to zero.
	void reset();
    
  /// Set the target position. The run() function will try to move the motor
  /// from the current position to the target position set by the most
  /// recent call to this function. Caution: moveTo() also recalculates the speed for the next step. 
  /// If you are trying to use constant speed movements, you should call setSpeed() after calling moveTo().
  /// \param[in] absolute The desired absolute position. Negative is anticlockwise from the 0 position.
  void moveTo(long stepperMoveTo);

  /// Set the target position relative to the current position
  /// \param[in] relative The desired position relative to the current position.
  /// Negative is anticlockwise from the current position.
  void move(long stepperMove);

  /// Poll the motor and step it if a step is due, implmenting a constant
  /// speed as set by the most recent call to setSpeed(). You must call this as
  /// frequently as possible, but at least once per step interval,
  /// \return true if the motor was stepped.
  void runSpeed();

  /// Sets the maximum permitted speed. the run() function will accelerate
  /// up to the speed set by this function.
  /// \param[in] speed The desired maximum speed in steps per second. Must
  /// be > 0. Caution: Speeds that exceed the maximum speed supported by the processor may
  /// Result in non-linear accelerations and decelerations.
  void setMaxSpeed(long stepperMaxSpeed);

  /// Sets the acceleration and deceleration parameter.
  /// \param[in] acceleration The desired acceleration in steps per second
  /// per second. Must be > 0.0. This is an expensive call since it requires a square 
  /// root to be calculated. Dont call more ofthen than needed
  void setAcceleration(long stepperAcceleration);

  /// Sets the desired constant speed for use with runSpeed().
  /// \param[in] speed The desired constant speed in steps per
  /// second. Positive is clockwise. Speeds of more than 1000 steps per
  /// second are unreliable. Very slow speeds may be set (eg 0.00027777 for
  /// once per hour, approximately. Speed accuracy depends on the Arduino
  /// crystal. Jitter depends on how frequently you call the runSpeed() function.
  void setSpeed(long stepperSpeed);

  /// The most recently set speed
  /// \return the most recent speed in steps per second
  long speed();

  /// The distance from the current position to the target position.
  /// \return the distance from the current position to the target position in steps.
  /// Positive is clockwise from the current position.
  long distanceToGo();

  /// The most recently set target position.
  /// \return the target position in steps. Positive is clockwise from the 0 position.
  long targetPosition();

  /// The currently motor position.
  /// \return the current motor position
  /// in steps. Positive is clockwise from the 0 position.
  long currentPosition();  

  /// Resets the current position of the motor, so that wherever the motor
  /// happens to be right now is considered to be the new 0 position. Useful
  /// for setting a zero position on a stepper after an initial hardware
  /// positioning move.
  /// Has the side effect of setting the current motor speed to 0.
  /// \param[in] position The position in steps of wherever the motor happens to be right now.
  void setCurrentPosition(long stepperCuttentPos);

	// enable stepper driver, Keep the micro step current position.
	void enable(); 
	
	// disable stepper driver, release the stepper.
	void disable();

	// output pulse
	void run(); 

	/// Sets a new target position that causes the stepper
  /// to stop as quickly as possible, using to the current speed and acceleration parameters.
	void stop();

	// stop all dispose, keep the user setting data.
	void wait();

	boolean readState();

	byte twi_read(int slaveAddress, byte dataAddress);
	
	void twi_write(int slaveAddress, byte dataAddress, byte data);
	
private:
	byte s1Pin;
	byte s2Pin;
	byte slaveAddress;
	long stepperSpeedRead;
	long stepperDistanceToGoRead;
	long stepperTargetPositionRead;
	long stepperCurrentPositionRead;
};
#endif

