package org.usfirst.frc.team1806.robot.commands.intake;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates;
import org.usfirst.frc.team1806.robot.RobotStates.IntakePosition;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RaiseIntake extends Command {

	double kTimeToRaise = Constants.intakeTimeToRaise;

	Timer timer;
	boolean allowed = true;

	public RaiseIntake() {
		requires(Robot.intakeSS);
		timer = new Timer();

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (Robot.elevatorSS.getElevatorPosition() > 2000
				&& Robot.elevatorSS.getElevatorPosition() < Constants.elevatorChevaldeFunHeight) {
			allowed = false;
		}else{
			Robot.states.intakePositionTracker = IntakePosition.MOVING;
			Robot.intakeSS.retractIntake();
			timer.reset();
			timer.start();
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return timer.get() >= kTimeToRaise || !allowed;
	}

	// Called once after isFinished returns true
	protected void end() {
		if (allowed) {
			Robot.states.intakePositionTracker = IntakePosition.RETRACTED;
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		if (allowed) {
			Robot.states.intakePositionTracker = IntakePosition.RETRACTED;
		}
	}
}
