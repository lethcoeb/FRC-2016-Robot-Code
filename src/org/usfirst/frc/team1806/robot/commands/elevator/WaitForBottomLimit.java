package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WaitForBottomLimit extends Command {

	boolean limitSwitchFound;
	
    public WaitForBottomLimit() {
        requires(Robot.elevatorSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	limitSwitchFound = Robot.elevatorSS.isBottomLimitHit();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	limitSwitchFound = Robot.elevatorSS.isBottomLimitHit();
    	
    	if(!limitSwitchFound && Robot.elevatorSS.getElevatorPosition() >= 0){
    		Robot.elevatorSS.elevatorMoveAtSpeed(-.2);
    	}else if(!limitSwitchFound && Robot.elevatorSS.getElevatorPosition() < 0){
    		Robot.elevatorSS.elevatorMoveAtSpeed(.2);
    	}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return limitSwitchFound;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevatorSS.elevatorStopMovement();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevatorSS.elevatorStopMovement();
    }
}
