package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MoveToLocationPID extends Command {

	double target;
	double startPosition;
	
	boolean PIDEngaged;
	boolean up;
	
    public MoveToLocationPID(int location) {
        requires(Robot.elevatorSS);
        target = location;
        startPosition = Robot.elevatorSS.getElevatorPosition();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	Robot.states.shooterArmPositionTracker = ShooterArmPosition.OTHER;
    	
    	//decide whether PID needs to engage at the start or not
    	if(Math.abs(target - startPosition) > Constants.elevatorPIDEngageDistance){ 
    		PIDEngaged = false;
    	}else{
    		PIDEngaged = true;
    	}
    	
    	
    	Robot.elevatorSS.resetSrxPID();
    	//Move elevator accordingly
    	if(PIDEngaged){
    		System.out.println("Engaging PID at init");
  
    		//Here we switch the PID Constants on the SRX based on whether or not we need to travel up or down
    		if(target - startPosition > 0){
    			//Elevator needs to go up; use 'up' pid constants
    			Robot.elevatorSS.elevatorSetPIDValues(Constants.elevatorPIDp, Constants.elevatorPIDi, Constants.elevatorPIDd);
    		}else{
    			//ELevator needs to go down; use 'down' pid constants
    			Robot.elevatorSS.elevatorSetPIDValues(Constants.elevatorDownPIDp, Constants.elevatorDownPIDi, Constants.elevatorDownPIDd);
    		}
    		
    		//Set SRX's control mode & setpoint to make it move
    		Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
    		Robot.elevatorSS.elevatorSetSetpoint((int) target);

    	}else{
    		//Move the elevator at a constant "full" speed because you're not close enough to the target to warrant using PID control
    		System.out.println("Moving at full speed");
    		Robot.elevatorSS.elevatorMoveAtSpeed(1 * Math.signum(target - startPosition));
    	}
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	//Only check if you need to engage PID control if you haven't already
    	if(!PIDEngaged){
    		
    		//If you're close enough to warrant usage of PID control... use it
    		if(Math.abs(Robot.elevatorSS.getElevatorPosition() - target) < Constants.elevatorPIDEngageDistance){
    			//Set PIDEngaged to true so you don't try to reenable PID control after you already enabled it
    			PIDEngaged = true;
        		System.out.println("Engaging PID after moving at full speed.");
        		if(target - startPosition > 0){
        			//Elevator needs to go up; use 'up' pid constants
        			Robot.elevatorSS.elevatorSetPIDValues(Constants.elevatorPIDp, Constants.elevatorPIDi, Constants.elevatorPIDd);
        		}else{
        			//Elevator needs to go down; use 'down' pid constants
        			Robot.elevatorSS.elevatorSetPIDValues(Constants.elevatorDownPIDp, Constants.elevatorDownPIDi, Constants.elevatorDownPIDd);
        		}
        		//Enable PID control and set setpoint to target position
        		Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
        		Robot.elevatorSS.elevatorSetSetpoint(target);

    		}
    	}    	
    }

    protected boolean isFinished() {
    	//Return true to end the command if the elevator is within range of its setpoint.
        return Math.abs(target - Robot.elevatorSS.getElevatorPosition()) < Constants.elevatorAbsoluteTolerance;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("MoveToLocationPID finished, elevator is on target.");
    	Robot.elevatorSS.elevatorStopMovement();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("MoveToLocationPID interrupted.");
    	Robot.elevatorSS.elevatorStopMovement();
    }
}
