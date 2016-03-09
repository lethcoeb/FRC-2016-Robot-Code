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

		// TODO delete me
		elevatorSRX = new CANTalon(0);
		elevatorSRX.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		// starts in pid mode
		elevatorSRX.changeControlMode(TalonControlMode.Position);
		elevatorSRX.setPID(.05, 0.000005, 0.01);
		elevatorSRX.reverseOutput(false);
		elevatorSRX.reverseSensor(true);
		elevatorSRX.configPeakOutputVoltage(14, -14);
		elevatorSRX.setCloseLoopRampRate(1);
		//elevatorSRX.
		elevatorSRX.enable();

	}

	public void elevatorEnable() {
		elevatorSRX.enable();
	}

	public void elevatorDisable() {
		elevatorSRX.disable();
	}

	public void elevatorSetControlMode(TalonControlMode cm) {
		elevatorSRX.changeControlMode(cm);
	}

	public void elevatorSetSetpoint(int setpoint) {
		elevatorSRX.setSetpoint(setpoint);
		elevatorSetpoint = setpoint;
	}

	public void elevatorMoveAtSpeed(double speed) {
		// for manual mode - directly set speed
		if(speed == 0){
			elevatorSRX.set(0);
		}
		else if (isBottomLimitHit() && speed < 0) {
			
				elevatorSRX.set(0);
			
			Robot.states.shooterArmPositionTracker = ShooterArmPosition.DOWN;
		} else {
			//hella set it to whatever
			if (elevatorSRX.getControlMode() != TalonControlMode.PercentVbus) {
				elevatorSRX.changeControlMode(TalonControlMode.PercentVbus);
			}
			elevatorSRX.set(speed);
		}

	}

	public void elevatorStopMovement() {
		if (elevatorSRX.getControlMode() == TalonControlMode.Position) {
			elevatorSRX.disable();
		} else {
			elevatorSRX.set(0);
		}
	}

	public void elevatorResetEncoder() {
		elevatorSRX.setEncPosition(0);
	}

	public boolean isElevatorPIDEnabled() {
		return elevatorSRX.getControlMode() == TalonControlMode.Position;
	}

	public boolean isElevatorPIDOnTarget() {
		return Math.abs(elevatorSRX.getPosition() - elevatorSRX.getSetpoint()) < Constants.elevatorAbsoluteTolerance;
	}

	public double getElevatorPosition() {
		return elevatorSRX.getPosition();
	}

	public double getElevatorSetpoint() {
		return elevatorSetpoint;
	}

	public boolean isBottomLimitHit() {
		return !bottomLimit.get();
	}

	// for some reason you gotta call this a lot to use the SRX PID idk why.
	// fuck srx's
	// jk i love them but still
	public void resetSrxPID() {
		elevatorSRX.reset();
		elevatorSRX.disable();
		elevatorSRX.enable();
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
