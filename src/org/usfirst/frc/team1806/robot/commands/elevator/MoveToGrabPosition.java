package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MoveToGrabPosition extends Command {

    public MoveToGrabPosition() {
        requires(Robot.elevatorSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("MoveToGrabPosition() Started");
    	Robot.elevatorSS.resetSrxPID();
    	if(!Robot.elevatorSS.isElevatorPIDEnabled()){
        	Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
    	}
    	
    	if(Robot.elevatorSS.getElevatorPosition() < Constants.elevatorIntakeEngagedHeight && Robot.states.hasBall){
    		//need to run intake because ball is engaged w/ intake still
    	}
    	
    	Robot.elevatorSS.elevatorSetSetpoint(0);
    	if(!Robot.elevatorSS.isBottomLimitHit()){
    		Robot.states.shooterArmPositionTracker = ShooterArmPosition.OTHER;
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.elevatorSS.isElevatorPIDOnTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevatorSS.elevatorStopMovement();
    	Robot.elevatorSS.resetElevatorEncoder();
		Robot.states.shooterArmPositionTracker = ShooterArmPosition.DOWN;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevatorSS.elevatorStopMovement();
    	Robot.elevatorSS.resetSrxPID();

    }
}
