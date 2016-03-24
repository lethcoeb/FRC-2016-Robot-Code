package org.usfirst.frc.team1806.robot;

public class RobotStates {
	
	public RobotStates(){
		reset();
	}
	
	public enum Mode{
		AUTONOMOUS, TELEOP
	}
	public enum IntakePosition{
		DEPLOYED, RETRACTED, MOVING
	}
	
	public enum ShooterArmPosition{
		DOWN, UP, HOLDING, OTHER
	}
	
	public enum DriveControlMode{
		DRIVER, AUTO
	}
	
	public enum DrivetrainGear{
		HIGH, LOW
	}
	
	public enum ShooterCocked{
		COCKED, NOTCOCKED
	}
	
	public enum IntakeRollerState{
		INTAKING, OUTTAKING, STOPPED
	}
	
	public enum ElevatorOperatorControlMode{
		AUTO, MANUAL, RESET
	}
	
	public enum VisionTrackingState{
		JETSON, ROBORIO, NONE
	}
	
	public enum IntakeControlMode{
		DRIVER, AUTOMATIC
	}
	
	public boolean collectingBalling;
	
	public boolean autoLiningUp;
	public boolean hasBall;
	
	public int overshoot;
	
	public IntakePosition intakePositionTracker;
	public ShooterArmPosition shooterArmPositionTracker;
	public DriveControlMode driveControlModeTracker;
	public DrivetrainGear drivetrainGearTracker;
	public ShooterCocked shooterCockedTracker;
	public IntakeRollerState intakeRollerStateTracker;
	public ElevatorOperatorControlMode elevatorOperatorControlModeTracker;
	public VisionTrackingState visionTrackingStateTracker;
	public IntakeControlMode intakeControlModeTracker;
	public Mode mode;
	
	
	public void reset(){
		mode = Mode.TELEOP;
		intakePositionTracker = IntakePosition.RETRACTED;
		shooterArmPositionTracker = ShooterArmPosition.UP;
		driveControlModeTracker = DriveControlMode.DRIVER;
		drivetrainGearTracker = DrivetrainGear.LOW;
		shooterCockedTracker = ShooterCocked.COCKED;
		intakeRollerStateTracker = IntakeRollerState.STOPPED;
		elevatorOperatorControlModeTracker = ElevatorOperatorControlMode.AUTO;
		visionTrackingStateTracker = VisionTrackingState.JETSON;
		intakeControlModeTracker = IntakeControlMode.DRIVER;
		hasBall = true;
		autoLiningUp = false;
		collectingBalling = false;
		overshoot = 0;
	}
	
}
