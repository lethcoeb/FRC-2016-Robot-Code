package org.usfirst.frc.team1806.robot.commands.autonomous;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.DriveControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToPosition extends Command {
	
	double kTargetPos;
	double maxPIDSpeed;
	
    public DriveToPosition(double inches, double maxSpeed) {
        requires(Robot.drivetrainSS);
        kTargetPos = inches;
        maxPIDSpeed = maxSpeed;
        //this.setTimeout((maxSpeed/inches) * 72);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrainSS.drivetrainDrivePIDSetMaxSpeed(maxPIDSpeed);
    	Robot.drivetrainSS.resetYaw();
    	Robot.drivetrainSS.resetEncoders();
    	Robot.states.driveControlModeTracker = DriveControlMode.AUTO;
    	Robot.drivetrainSS.drivetrainDrivePIDEnable();
    	Robot.drivetrainSS.drivetrainDrivePIDSetSetpoint(kTargetPos);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        //return Math.abs(Robot.drivetrainSS.getRightEncoderDistance()) > Math.abs(kTargetPos) || Math.abs(Robot.drivetrainSS.getLeftEncoderDistance()) > Math.abs(kTargetPos);
    	return Robot.drivetrainSS.drivetrainDrivePIDisOnTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("DriveToPosition finished");
    	Robot.drivetrainSS.arcadeDrive(0, 0);
    	Robot.drivetrainSS.drivetrainDrivePIDResetMaxSpeed();
    	Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
    	Robot.drivetrainSS.drivetrainDrivePIDDisable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("DriveToPosition interrupted");
    	Robot.drivetrainSS.arcadeDrive(0, 0);
    	Robot.drivetrainSS.drivetrainDrivePIDResetMaxSpeed();
    	Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
    	Robot.drivetrainSS.drivetrainDrivePIDDisable();
    }
}
