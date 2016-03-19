package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.RobotStates;
import org.usfirst.frc.team1806.robot.RobotStates.ElevatorOperatorControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.IntakePosition;
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
		System.out.println("ResetElevator() started");
		Robot.states.elevatorOperatorControlModeTracker = ElevatorOperatorControlMode.RESET;

		if (Robot.states.intakePositionTracker == IntakePosition.DEPLOYED) {
			Robot.elevatorSS.resetSrxPID();
			Robot.states.shooterArmPositionTracker = ShooterArmPosition.OTHER;

			Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.PercentVbus);
			Robot.elevatorSS.elevatorMoveAtSpeed(Constants.resetSpeed);

			Robot.intakeSS.runAtSpeed(-Constants.intakeSpeedToMatchArm + .3);
		} else {
			finished = true;
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.elevatorSS.isBottomLimitHit()) {
			stopElevator();
		} else if (Robot.pdp.getCurrent(RobotMap.PDPelevatorSlot) >= 100) {
			// stalling, limit switch is broken
			stopElevator();
		}
	}
	
	private void stopElevator(){
		System.out.println("Finished resetting");
		Robot.elevatorSS.elevatorMoveAtSpeed(0);
		Robot.states.shooterArmPositionTracker = ShooterArmPosition.DOWN;
		Robot.elevatorSS.elevatorResetEncoder();
		Robot.intakeSS.stopIntaking();
		finished = true;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// return finished;
		return (finished) && !Robot.oi.dBack;
	}

	protected void end() {
		Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
		Robot.states.elevatorOperatorControlModeTracker = ElevatorOperatorControlMode.AUTO;
		new MoveToHoldingFromLow(Robot.states.hasBall).start();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		System.out.println("reset arm interrupted");
		Robot.elevatorSS.elevatorMoveAtSpeed(0);
	}
}
