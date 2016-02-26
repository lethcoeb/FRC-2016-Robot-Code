package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates;
import org.usfirst.frc.team1806.robot.RobotStates.ElevatorOperatorControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ResetElevator extends Command {

	boolean finished = false;
	
    public ResetElevator() {
        requires(Robot.elevatorSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//FIXME uncomment this ish
    	//if(Robot.states.intakePositionTracker != RobotStates.IntakePosition.RETRACTED){
    	Robot.elevatorSS.resetSrxPID();
    	Robot.states.shooterArmPositionTracker = ShooterArmPosition.OTHER;
    	Robot.elevatorSS.elevatorSetSetpoint(0);
    	
    		Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.PercentVbus);
    		Robot.elevatorSS.elevatorMoveAtSpeed(-.35);
    	//}else{
    		//finished = true;
    //	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.elevatorSS.isBottomLimitHit()){
    		finished = true;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Finished resetting");
    	Robot.elevatorSS.elevatorMoveAtSpeed(0);
    	Robot.states.shooterArmPositionTracker = ShooterArmPosition.DOWN;
    	Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
    	Robot.elevatorSS.elevatorResetEncoder();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevatorSS.elevatorMoveAtSpeed(0);
    }
}
