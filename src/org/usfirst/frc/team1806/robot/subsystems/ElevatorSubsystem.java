package org.usfirst.frc.team1806.robot.subsystems;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;

import edu.wpi.first.wpilibj.CANSpeedController.ControlMode;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ElevatorSubsystem extends Subsystem {

	/*
	 * RARE MEMES!
	 */

	CANTalon elevatorSRX;
	DigitalInput bottomLimit;
	DigitalInput topLimit;
	
	int elevatorSetpoint;

	public ElevatorSubsystem() {

		bottomLimit = new DigitalInput(RobotMap.bottomElevatorLimit);
		// topLimit = new DigitalInput(RobotMap.topElevatorLimit);

		elevatorSRX = new CANTalon(0);
		//elevatorSRX.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Absolute);
		elevatorSRX.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		// starts in pid mode
		elevatorSRX.changeControlMode(TalonControlMode.Position);
		//elevatorSRX.setPID(0, 0, 0);
		elevatorSRX.setPID(Constants.elevatorPIDp, Constants.elevatorPIDi, Constants.elevatorPIDd);
		elevatorSRX.reverseOutput(false);
		elevatorSRX.reverseSensor(true);
		elevatorSRX.setAllowableClosedLoopErr(0);
		//elevatorSRX.configPeakOutputVoltage(14, -14);
		//elevatorSRX.setCloseLoopRampRate(1);
		//elevatorSRX.
		
		elevatorSRX.setForwardSoftLimit(106000);
		elevatorSRX.enableForwardSoftLimit(true);
		elevatorSRX.setReverseSoftLimit(-500);
		elevatorSRX.enableReverseSoftLimit(true);
		
		elevatorSRX.setIZone(2000);
		
		elevatorSRX.enable();

	}

	public void elevatorEnable() {
		elevatorSRX.enable();
		elevatorSRX.enableControl();
	}

	public void elevatorDisable() {
		elevatorSRX.disable();
	}
	
	public double elevatorGetPowerOutput(){
		return elevatorSRX.getOutputVoltage();
	}

	public boolean isMovingDown(){
		return elevatorGetPowerOutput() < 0;
	}
	
	public boolean isMovingUp(){
		return elevatorGetPowerOutput() > 0;
	}
	
	public void elevatorSetPIDValues(double p, double i, double d){
		elevatorSRX.setP(p);
		elevatorSRX.setI(i);
		elevatorSRX.setD(d);
	}
	
	public void elevatorSetControlMode(TalonControlMode cm) {
		elevatorSRX.changeControlMode(cm);
	}

	public void elevatorSetSetpoint(double setpoint) {
		System.out.println(setpoint);
		elevatorSRX.set(setpoint);
		elevatorSetpoint = (int) setpoint;
	}

	public void elevatorMoveAtSpeed(double speed) {
		// for manual mode - directly set speed
		
		if (isBottomLimitHit() && speed < 0) {
			speed = 0;
			Robot.states.shooterArmPositionTracker = ShooterArmPosition.DOWN;
		}
		
		elevatorSetControlMode(TalonControlMode.PercentVbus);
		elevatorSRX.set(speed);

	}

	public void elevatorStopMovement() {
		//elevatorSRX.disableControl();
		if (elevatorSRX.getControlMode() == TalonControlMode.Position) {
			elevatorSRX.disable();
		} else {
			elevatorSRX.set(0);
		}
	}

	public void elevatorResetEncoder() {
		elevatorSRX.setEncPosition(0);
	}
	
	public void elevatorSetPosition(int pos){
		elevatorSRX.setEncPosition(pos);
	}

	public boolean isElevatorPIDEnabled() {
		return elevatorSRX.getControlMode() == TalonControlMode.Position;
	}

	public boolean isElevatorPIDOnTarget() {
		System.out.println(Math.abs(elevatorSRX.getPosition() - elevatorSRX.getSetpoint()));
		return Math.abs(elevatorSRX.getPosition() - elevatorSRX.getSetpoint()) < Constants.elevatorAbsoluteTolerance;
	}

	public double getElevatorPosition() {
		return elevatorSRX.getPosition();
	}
	
	public TalonControlMode getElevatorControlMode(){
		return elevatorSRX.getControlMode();
	}

	public int getElevatorSetpoint() {
		return elevatorSetpoint;
	}

	public boolean isBottomLimitHit() {
		return !bottomLimit.get();
	}

	// for some reason you gotta call this a lot to use the SRX PID idk why.

	public void resetSrxPID() {
		elevatorSRX.reset();
		elevatorSRX.disable();
		elevatorSRX.enable();
		elevatorSRX.enableControl();
	}

	/*
	 * public boolean isTopLimitHit() { //return topLimit.get(); }
	 */

	public void resetElevatorEncoder() {
		elevatorSRX.setEncPosition(0);

	}

	public void initDefaultCommand() {

	}
}
