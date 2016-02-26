package org.usfirst.frc.team1806.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardUpdater {
	
	public void push(){
		
		if(pushStates){
			SmartDashboard.putString("Intake Position", Robot.states.intakePositionTracker.toString());
			SmartDashboard.putString("Intake Roller Status", Robot.states.intakeRollerStateTracker.toString());
			SmartDashboard.putString("Shooter Position", Robot.states.shooterArmPositionTracker.toString());
			SmartDashboard.putString("Shooter Cocked State", Robot.states.shooterCockedTracker.toString());
			SmartDashboard.putString("Drivetrain Gear", Robot.states.drivetrainGearTracker.toString());
			SmartDashboard.putBoolean("HasBall?", Robot.states.hasBall);
			SmartDashboard.putNumber("RobotTilt", Robot.drivetrainSS.getTilt());
			SmartDashboard.putString("VisionState", Robot.states.visionTrackingStateTracker.toString());
			
			//States
			SmartDashboard.putString("DriveControlMode", Robot.states.driveControlModeTracker.toString());
			SmartDashboard.putString("Gear", Robot.states.drivetrainGearTracker.toString());
			SmartDashboard.putString("ElevatorMode", Robot.states.elevatorOperatorControlModeTracker.toString());
			SmartDashboard.putBoolean("hasBall", Robot.states.hasBall);
			SmartDashboard.putString("intakePosition", Robot.states.intakePositionTracker.toString());
			SmartDashboard.putString("intakeRollerState", Robot.states.intakeRollerStateTracker.toString());
			SmartDashboard.putString("shooterArmPos", Robot.states.shooterArmPositionTracker.toString());
			SmartDashboard.putString("shooter cocked?", Robot.states.shooterCockedTracker.toString());
			SmartDashboard.putString("visionTrackingState", Robot.states.visionTrackingStateTracker.toString());
			
			SmartDashboard.putBoolean("transduction sensor", Robot.shooterSS.shooterIsCocked());
			SmartDashboard.putBoolean("bottom limit", Robot.elevatorSS.isBottomLimitHit());
			
			
			SmartDashboard.putNumber("elevator pos", Robot.elevatorSS.getElevatorPosition());
			SmartDashboard.putNumber("elevator setpoint", Robot.elevatorSS.getElevatorSetpoint());
			SmartDashboard.putBoolean("optical", Robot.shooterSS.hasBallSensor());
			
			SmartDashboard.putNumber("true angle", Robot.drivetrainSS.getTrueAngle());
			SmartDashboard.putNumber("Winch current", Robot.pdp.getCurrent(13));

		}
		
	}
	
	//what information do you want to send
	public static boolean pushStates = true;
	
}
