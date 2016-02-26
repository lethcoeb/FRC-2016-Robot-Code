package org.usfirst.frc.team1806.robot.commands.shooter;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ReleaseBall extends Command {
	
	Timer t;
	double kTimeToRelease = Constants.timeToUnpinch;
	
    public ReleaseBall() {
        requires(Robot.shooterSS);
        t = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.states.hasBall = false;
    	t.reset();
    	t.start();
    	Robot.shooterSS.releaseBall();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return t.get() > kTimeToRelease;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.states.hasBall = false;

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.states.hasBall = false;

    }
}
