package org.usfirst.frc.team1806.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardUpdater {

	NetworkTable towerTracker;
	
	public SmartDashboardUpdater(){
		towerTracker = NetworkTable.getTable("TowerTracker");
	}
	
	public void push() {

		if (pushStates) {
			SmartDashboard.putString("Intake Position", Robot.states.intakePositionTracker.toString());
			SmartDashboard.putString("Intake Roller Status", Robot.states.intakeRollerStateTracker.toString());
			SmartDashboard.putString("Shooter Cocked State", Robot.states.shooterCockedTracker.toString());
			SmartDashboard.putString("Drivetrain Gear", Robot.states.drivetrainGearTracker.toString());
			SmartDashboard.putBoolean("HasBall?", Robot.states.hasBall);
			SmartDashboard.putNumber("RobotTilt", Robot.drivetrainSS.getTilt());
			SmartDashboard.putString("VisionState", Robot.states.visionTrackingStateTracker.toString());

			// States
			SmartDashboard.putString("DriveControlMode", Robot.states.driveControlModeTracker.toString());
			SmartDashboard.putString("Gear", Robot.states.drivetrainGearTracker.toString());
			SmartDashboard.putString("ElevatorMode", Robot.states.elevatorOperatorControlModeTracker.toString());
			SmartDashboard.putBoolean("hasBall", Robot.states.hasBall);
			SmartDashboard.putString("intakePosition", Robot.states.intakePositionTracker.toString());
			SmartDashboard.putString("intakeRollerState", Robot.states.intakeRollerStateTracker.toString());
			SmartDashboard.putString("shooterArmPos", Robot.states.shooterArmPositionTracker.toString());
			SmartDashboard.putString("shooter cocked?", Robot.states.shooterCockedTracker.toString());
			SmartDashboard.putString("visionTrackingState", Robot.states.visionTrackingStateTracker.toString());

			SmartDashboard.putBoolean("transduction sensor", Robot.shooterSS.isShooterCocked());
			SmartDashboard.putBoolean("bottom limit", Robot.elevatorSS.isBottomLimitHit());

			SmartDashboard.putNumber("elevator pos", Robot.elevatorSS.getElevatorPosition());
			SmartDashboard.putNumber("elevator setpoint", Robot.elevatorSS.getElevatorSetpoint());
			SmartDashboard.putBoolean("optical", Robot.shooterSS.hasBallSensor());

			SmartDashboard.putNumber("True Angle", Robot.drivetrainSS.getTrueAngle());
		//	SmartDashboard.putNumber("Winch current", Robot.pdp.getCurrent(13));

			//SmartDashboard.putNumber("total current", Robot.pdp.getTotalCurrent());

			SmartDashboard.putString("intake control mode", Robot.states.intakeControlModeTracker.toString());
			
			
			SmartDashboard.putNumber("LoopsUntilDone", Robot.states.loopsUntilDone);
			SmartDashboard.putNumber("Initial Target", Robot.states.initialTarget);
			//SmartDashboard.putNumber("Winch Resistence", Robot.getPDPResistance(RobotMap.PDPcockingWinchSlot));

			/*if (!Robot.jr.isGoalFound()) {
				SmartDashboard.putString("Angle to goal", "N/A");
				SmartDashboard.putString("LeftRight", "NotFound");
			}else{
				SmartDashboard.putString("Angle to goal", String.valueOf(Robot.jr.getAngleToGoal()));
				if(Robot.jr.getAngleToGoal() > 0){
					SmartDashboard.putString("LeftRight", "Right");
				}else{
					SmartDashboard.putString("LeftRight", "Left");
				}
			}*/
			
			
			
			SmartDashboard.putNumber("Navx Yaw", Robot.drivetrainSS.getYaw());
			
			SmartDashboard.putNumber("LsY", Robot.oi.dc.getLeftJoyY());
			
			SmartDashboard.putNumber("TurnPC error", Robot.drivetrainSS.getTurnPCError());

			SmartDashboard.putNumber("angular velocity", Robot.drivetrainSS.getRotationalSpeed());
			
			SmartDashboard.putNumber("RightEncValue", Robot.drivetrainSS.getRightEncoderDistance());
			SmartDashboard.putNumber("LeftEncValue", Robot.drivetrainSS.getLeftEncoderDistance());
			
			SmartDashboard.putNumber("NavxPitch", Robot.drivetrainSS.getPitch());
			SmartDashboard.putNumber("NavxRoll", Robot.drivetrainSS.getRoll());
			
			if(Robot.states.pulsePower != null){
				SmartDashboard.putNumber("PulsePower Global", Robot.states.pulsePower);
			}
			
			//SmartDashboard.putBoolean("GoalFound", towerTracker.getBoolean("GoalFound", false));
			//SmartDashboard.putNumber("AngleToGoal", towerTracker.getNumber("AngleToGoal", 0));
			
			
		}

	}

	// what information do you want to send
	public static boolean pushStates = true;

}
