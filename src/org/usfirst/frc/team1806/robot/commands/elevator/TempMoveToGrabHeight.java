package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TempMoveToGrabHeight extends Command {

	
	boolean outtaking = true;
	boolean finished = false;
	
    public TempMoveToGrabHeight() {
        requires(Robot.elevatorSS);
        requires(Robot.intakeSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(Robot.states.hasBall){
    		Robot.intakeSS.runAtSpeed(-Constants.intakeSpeedToMatchArm + .15);
    	}
    	Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
    	Robot.elevatorSS.elevatorSetSetpoint(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	if(Robot.elevatorSS.getElevatorPosition() < 5000 && outtaking){
    		Robot.intakeSS.stopIntaking();
    		outtaking = false;
    	}
    	
    	if(!Robot.oi.oPOVDown){
    		finished = true;
    	}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	new MoveToHoldingFromLow(Robot.states.hasBall).start();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	
    }
}
