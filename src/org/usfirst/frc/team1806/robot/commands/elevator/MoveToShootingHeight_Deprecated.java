package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.IntakeControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MoveToShootingHeight_Deprecated extends Command {

	
	boolean pidEnabled = false;
	boolean intakingStopped = false;
	
    public MoveToShootingHeight_Deprecated() {
        requires(Robot.elevatorSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.states.shooterArmPositionTracker = ShooterArmPosition.OTHER;
    	System.out.println("MoveToShootingHeight() Started");
    	Robot.elevatorSS.resetSrxPID();
    	Robot.elevatorSS.elevatorSetSetpoint(Constants.elevatorShootingHeight);

    	
        	Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.PercentVbus);
        	Robot.elevatorSS.elevatorMoveAtSpeed(1);
        	System.out.println("Moving full speed to target");
    	
    	
    	Robot.states.shooterArmPositionTracker = ShooterArmPosition.OTHER;
    	
    	
    	if(Robot.elevatorSS.getElevatorPosition() < Constants.elevatorIntakeEngagedHeight && Robot.states.hasBall){
    		//you need to help the elevator raise with the intake since the ball is engaged
    		Robot.states.intakeControlModeTracker = IntakeControlMode.AUTOMATIC;
    		Robot.intakeSS.runAtSpeed(.4);
    	}
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	if(Robot.elevatorSS.getElevatorPosition() >= Constants.elevatorShootingHeight - 10000 && !pidEnabled){
    		
    		pidEnabled = true;
    		Robot.elevatorSS.resetSrxPID();
    		Robot.elevatorSS.elevatorDisable();
    		Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
    		Robot.elevatorSS.elevatorSetPIDValues(Constants.elevatorPIDp, Constants.elevatorPIDi, Constants.elevatorPIDd);
        	Robot.elevatorSS.elevatorSetSetpoint(Constants.elevatorShootingHeight);
        	Robot.elevatorSS.elevatorEnable();
        	System.out.println("Engaging PID at height: " + Robot.elevatorSS.getElevatorPosition());
        	
    	}
    	
    	if(Robot.elevatorSS.getElevatorPosition() >= 40000 && !intakingStopped){
    		intakingStopped = true;
    		//can stop helping
    		//FIXME make this a variable
    		Robot.states.intakeControlModeTracker = IntakeControlMode.DRIVER;
    		Robot.intakeSS.stopIntaking();
    	}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return Math.abs(Robot.elevatorSS.getElevatorPosition() - Constants.elevatorShootingHeight) < 1000;
    	//return Robot.elevatorSS.isElevatorPIDOnTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	//Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
    	Robot.elevatorSS.elevatorStopMovement();
		Robot.states.shooterArmPositionTracker = ShooterArmPosition.UP;
		Robot.states.intakeControlModeTracker = IntakeControlMode.DRIVER;
		Robot.intakeSS.stopIntaking();

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevatorSS.elevatorStopMovement();
    	Robot.states.intakeControlModeTracker = IntakeControlMode.DRIVER;
    	Robot.intakeSS.runAtSpeed(0);
    	Robot.elevatorSS.resetSrxPID();
    }
}
