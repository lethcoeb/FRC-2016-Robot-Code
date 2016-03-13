package org.usfirst.frc.team1806.robot.commands.intake;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.IntakeControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.IntakeRollerState;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingFromLow;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SpitOutBall extends Command {

	boolean hasBegunOuttaking = false;
	
    public SpitOutBall() {
    	//HELLA REQUIREMENTS
    	Robot.states.intakeRollerStateTracker = IntakeRollerState.OUTTAKING;
        requires(Robot.elevatorSS);
        requires(Robot.intakeSS);
        requires(Robot.shooterSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		Robot.states.intakeControlModeTracker = IntakeControlMode.AUTOMATIC;
		System.out.println("SpitOutBall() started");
    	Robot.elevatorSS.resetSrxPID();
    	Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
    	Robot.elevatorSS.elevatorSetSetpoint(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(!hasBegunOuttaking && Robot.elevatorSS.getElevatorPosition() < Constants.elevatorHoldingHeight - 3000){
    		System.out.println("outtaking");
        	Robot.intakeSS.outtakeBall();
        	Robot.shooterSS.releaseBall();
        	Robot.states.hasBall = false;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !Robot.oi.dc.getButtonRB();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.states.intakeRollerStateTracker = IntakeRollerState.STOPPED;
    	Robot.intakeSS.stopIntaking();
    	new MoveToHoldingFromLow().start();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.states.intakeRollerStateTracker = IntakeRollerState.STOPPED;
    	Robot.intakeSS.stopIntaking();
    }
}
