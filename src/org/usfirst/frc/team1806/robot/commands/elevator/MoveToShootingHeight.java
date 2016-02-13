package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MoveToShootingHeight extends Command {

    public MoveToShootingHeight() {
        requires(Robot.elevatorSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(!Robot.elevatorSS.isElevatorPIDEnabled()){
        	Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
    	}
    	Robot.elevatorSS.elevatorSetSetpoint(Constants.elevatorShootingHeight);
    	if(!Robot.elevatorSS.isElevatorPIDOnTarget()){
    		Robot.states.shooterArmPositionTracker = ShooterArmPosition.MID;
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.elevatorSS.isElevatorPIDOnTarget() && !Robot.elevatorSS.isTopLimitHit()){
    		Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.PercentVbus);
    		Robot.elevatorSS.elevatorMoveAtSpeed(.2);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.elevatorSS.isTopLimitHit() ;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
    	Robot.elevatorSS.elevatorStopMovement();
		Robot.states.shooterArmPositionTracker = ShooterArmPosition.UP;

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevatorSS.elevatorStopMovement();
    }
}
