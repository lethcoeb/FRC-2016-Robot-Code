package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LowBarTeleopKill extends Command {

    public LowBarTeleopKill() {
    	requires(Robot.elevatorSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.oi.lowBarLatch.update(Robot.oi.dA) && Robot.states.lowBarring;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.states.lowBarring = false;
    	System.out.println("Ending lowbar teleop.");
    	try{
    		super.cancel();
    	}catch(Exception e){
    		e.printStackTrace();
    	}
    	new MoveToHoldingFromLow(Robot.states.hasBall);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.states.lowBarring = false;
    	System.out.println("Interrupting lowbar teleop.");
    	Robot.intakeSS.stopIntaking();
    	try{
    		super.cancel();
    	}catch(Exception e){
    		e.printStackTrace();
    	}
    	super.cancel();
    }
}
