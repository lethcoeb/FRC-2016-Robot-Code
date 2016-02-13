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

	public ElevatorSubsystem() {

		bottomLimit = new DigitalInput(RobotMap.bottomElevatorLimit);
		topLimit = new DigitalInput(RobotMap.topElevatorLimit);

		// TODO delete me
		elevatorSRX = new CANTalon(0);
		elevatorSRX.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		// starts in pid mode
		elevatorSRX.changeControlMode(TalonControlMode.Position);
		elevatorSRX.setPID(.05, 0, 0);
		elevatorSRX.reverseOutput(true);
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

	public void elevatorSetSetpoint(double setpoint) {
		elevatorSRX.setSetpoint(setpoint);
	}

	public void elevatorMoveAtSpeed(double speed) {
		// for manual mode - directly set speed
		if (topLimit.get()) {
			// too high/low, stop dat motor
			if (speed > 0) {
				elevatorSRX.set(0);
			}
			Robot.states.shooterArmPositionTracker = ShooterArmPosition.UP;
		} else if (bottomLimit.get()) {
			if(speed < 0){
				elevatorSRX.set(0);
			}
			Robot.states.shooterArmPositionTracker = ShooterArmPosition.DOWN;
		} else {
			Robot.states.shooterArmPositionTracker = ShooterArmPosition.MID;
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
		return elevatorSRX.getSetpoint();
	}

	public boolean isBottomLimitHit() {
		return bottomLimit.get();
	}

	public boolean isTopLimitHit() {
		return topLimit.get();
	}

	public void resetElevatorEncoder() {
		elevatorSRX.setEncPosition(0);
	}

	public void initDefaultCommand() {

	}
}
