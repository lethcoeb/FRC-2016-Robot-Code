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
  
    		
    		if(target - startPosition > 0){
    			//Elevator needs to go up; use 'up' pid constants
    			Robot.elevatorSS.elevatorSetPIDValues(Constants.elevatorPIDp, Constants.elevatorPIDi, Constants.elevatorPIDd);
    		}else{
    			//ELevator needs to go down; use 'down' pid constants
    			Robot.elevatorSS.elevatorSetPIDValues(Constants.elevatorDownPIDp, Constants.elevatorDownPIDi, Constants.elevatorDownPIDd);
    		}
    		
    		Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
    		Robot.elevatorSS.elevatorSetSetpoint((int) target);

    	}else{
    		//MOve the elevator at a constant "full" speed because you're not close enough to the target to warrant a pid engagement
    		System.out.println("Moving at full speed");
    		Robot.elevatorSS.elevatorMoveAtSpeed(1 * Math.signum(target - startPosition));
    	}
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(!PIDEngaged){
    		
    		if(Math.abs(Robot.elevatorSS.getElevatorPosition() - target) < Constants.elevatorPIDEngageDistance){
    			PIDEngaged = true;
        		System.out.println("Engaging PID after moving at full speed");
        		//Robot.elevatorSS.resetSrxPID();
    			//Robot.elevatorSS.elevatorDisable();
        		if(target - startPosition > 0){
        			//Elevator needs to go up; use 'up' pid constants
        			Robot.elevatorSS.elevatorSetPIDValues(Constants.elevatorPIDp, Constants.elevatorPIDi, Constants.elevatorPIDd);
        		}else{
        			//ELevator needs to go down; use 'down' pid constants
        			Robot.elevatorSS.elevatorSetPIDValues(Constants.elevatorDownPIDp, Constants.elevatorDownPIDi, Constants.elevatorDownPIDd);
        		}
        		Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
        		Robot.elevatorSS.elevatorSetSetpoint(target);
        		
        		System.out.println(Robot.elevatorSS.elevatorGetPowerOutput());
        		
        		//Robot.elevatorSS.elevatorEnable();
    		}
    	}
    	
    	//System.out.print(Robot.elevatorSS.elevatorGetPowerOutput() + "         ");
    	//System.out.println("Setpoint - position: " + (target - Robot.elevatorSS.getElevatorPosition()));
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(target - Robot.elevatorSS.getElevatorPosition()) < Constants.elevatorAbsoluteTolerance;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("MoveToLocationPID finished");
    	Robot.elevatorSS.elevatorStopMovement();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("MoveToLocationPID interrupted");
    	Robot.elevatorSS.elevatorStopMovement();
    }
}
