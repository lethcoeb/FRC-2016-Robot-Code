package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TempMoveToGrabHeight extends Command {

    public TempMoveToGrabHeight() {
        requires(Robot.elevatorSS);
        requires(Robot.intakeSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(Robot.states.hasBall){
    		Robot.intakeSS.runAtSpeed(-Constants.intakeSpeedToMatchArm);
    	}
    	Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
    	Robot.elevatorSS.elevatorSetSetpoint(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return !Robot.oi.oc.getPOVDown();
    }

    // Called once after isFinished returns true
    protected void end() {
    	new MoveToHoldingPID();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	
    }
}
