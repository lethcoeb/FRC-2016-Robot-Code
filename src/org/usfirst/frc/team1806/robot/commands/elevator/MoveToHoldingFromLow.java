package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.ElevatorOperatorControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.IntakeControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MoveToHoldingFromLow extends Command {

	boolean shouldSpin = true;

	public MoveToHoldingFromLow() {
		requires(Robot.elevatorSS);
	}

	public MoveToHoldingFromLow(boolean spin) {
		shouldSpin = spin;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.states.shooterArmPositionTracker = ShooterArmPosition.OTHER;
		System.out.println("MoveToHolding() Started");
		Robot.elevatorSS.resetSrxPID();
		Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.PercentVbus);
		Robot.elevatorSS.elevatorMoveAtSpeed(1);
		Robot.states.intakeControlModeTracker = IntakeControlMode.AUTOMATIC;
		if (shouldSpin) {
			Robot.intakeSS.runAtSpeed(Constants.intakeSpeedToMatchArm);
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.elevatorSS.getElevatorPosition() >= Constants.elevatorHoldingHeight;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.states.intakeControlModeTracker = IntakeControlMode.DRIVER;
		Robot.elevatorSS.elevatorStopMovement();
		Robot.states.shooterArmPositionTracker = ShooterArmPosition.HOLDING;
		Robot.intakeSS.stopIntaking();
		System.out.println("at holding");
		Robot.states.elevatorOperatorControlModeTracker = ElevatorOperatorControlMode.AUTO;
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.states.intakeControlModeTracker = IntakeControlMode.DRIVER;
		Robot.elevatorSS.elevatorStopMovement();
		Robot.states.shooterArmPositionTracker = ShooterArmPosition.OTHER;
		Robot.intakeSS.stopIntaking();
	}
}
