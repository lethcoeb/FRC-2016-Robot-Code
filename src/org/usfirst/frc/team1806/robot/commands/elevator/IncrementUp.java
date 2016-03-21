package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IncrementUp extends Command {

	double target;
	
    public IncrementUp(double inc) {
        requires(Robot.elevatorSS);
        target = Robot.elevatorSS.getElevatorPosition() + inc;
        Robot.elevatorSS.resetSrxPID();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevatorSS.elevatorMoveAtSpeed(.15);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.elevatorSS.getElevatorPosition() >= target;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevatorSS.elevatorMoveAtSpeed(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	//Robot.elevatorSS.elevatorMoveAtSpeed(0);
    }
}
