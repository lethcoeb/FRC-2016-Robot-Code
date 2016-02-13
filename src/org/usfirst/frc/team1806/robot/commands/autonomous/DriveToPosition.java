package org.usfirst.frc.team1806.robot.commands.autonomous;

import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToPosition extends Command {
	
	double kTargetPos;
	
    public DriveToPosition(double inches) {
        requires(Robot.drivetrainSS);
        kTargetPos = inches;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrainSS.drivetrainDrivePIDEnable();
    	Robot.drivetrainSS.drivetrainDrivePIDSetSetpoint(kTargetPos);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.drivetrainSS.drivetrainDrivePIDisOnTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrainSS.drivetrainDrivePIDDisable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drivetrainSS.drivetrainDrivePIDDisable();
    }
}
