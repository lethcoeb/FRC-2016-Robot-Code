package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TempMoveToChevalDeFunHeight extends Command {

	//bruh will this work it needs to listen for a button
	
	boolean atHeight = false;
	
    public TempMoveToChevalDeFunHeight() {
        requires(Robot.elevatorSS);
        requires(Robot.intakeSS);
        //FIXME this
        //this.setTimeout(5);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevatorSS.resetSrxPID();
    	Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.PercentVbus);
    	Robot.elevatorSS.elevatorMoveAtSpeed(.7);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.elevatorSS.getElevatorPosition() > Constants.elevatorChevaldeFunHeight && !atHeight){
    		atHeight = true;
    		Robot.elevatorSS.elevatorMoveAtSpeed(0);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return !Robot.oi.oc.getPOVUp();
    }

    // Called once after isFinished returns true
    protected void end() {
    	new MoveToHoldingPID().start();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	
    }
}
