package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MoveToHoldingPID extends Command {

	public MoveToHoldingPID() {
        requires(Robot.elevatorSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("MoveToHoldingPID() Started");
    	Robot.elevatorSS.resetSrxPID();
    	if(!Robot.elevatorSS.isElevatorPIDEnabled()){
        	Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
    	}
    	
    	Robot.elevatorSS.elevatorSetSetpoint(Constants.elevatorHoldingHeight);
    	Robot.states.shooterArmPositionTracker = ShooterArmPosition.OTHER;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.elevatorSS.isElevatorPIDOnTarget();
    }

    // Called once after isFinished returns true
    protected void end(){
    	Robot.elevatorSS.elevatorStopMovement();
		Robot.states.shooterArmPositionTracker = ShooterArmPosition.HOLDING;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevatorSS.elevatorStopMovement();
    	Robot.elevatorSS.resetSrxPID();

    }
}
