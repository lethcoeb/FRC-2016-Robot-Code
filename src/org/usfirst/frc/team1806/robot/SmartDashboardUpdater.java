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
		}
		
	}
	
	//what information do you want to send
	public static boolean pushStates = true;
	
}
