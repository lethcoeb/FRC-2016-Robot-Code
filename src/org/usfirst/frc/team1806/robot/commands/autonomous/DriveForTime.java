package org.usfirst.frc.team1806.robot.commands.autonomous;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.DriveControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveForTime extends Command {

	double timee, speedd;
	Timer t;
	
    public DriveForTime(double time, double speed) {
        requires(Robot.drivetrainSS);
        timee = time;
        speedd = speed;
        t = new Timer();
        t.reset();
        t.start();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.states.driveControlModeTracker = DriveControlMode.AUTO;
    	Robot.drivetrainSS.arcadeDrive(speedd, Robot.drivetrainSS.getYaw() * .05);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return t.get() >= timee;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrainSS.arcadeDrive(0, 0);
    	Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drivetrainSS.arcadeDrive(0, 0);
    	Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
    }
}
