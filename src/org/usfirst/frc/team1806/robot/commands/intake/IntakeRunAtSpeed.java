package org.usfirst.frc.team1806.robot.commands.intake;

import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeRunAtSpeed extends Command {

	
	double speed;
    public IntakeRunAtSpeed(double mspeed) {
        requires(Robot.intakeSS);
        speed = mspeed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intakeSS.runAtSpeed(speed);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
