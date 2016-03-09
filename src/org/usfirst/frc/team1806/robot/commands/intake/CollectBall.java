package org.usfirst.frc.team1806.robot.commands.intake;

import java.sql.Blob;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.OperatorInterface;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.IntakeControlMode;
import org.usfirst.frc.team1806.robot.commands.RumbleController;
import org.usfirst.frc.team1806.robot.commands.RumbleControllerConstant;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingFromLow;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CollectBall extends Command {

	Timer t;
	final double kTimeToCenter = Constants.intakeTimeToCenterBall;
	final double kElevatorIntakeEngagedHeight = Constants.elevatorIntakeEngagedHeight;
	final double kElevatorSafeToCollect = Constants.elevatorSafeToCollect;
	boolean intaking = false;
	boolean ballSensed = false;
	boolean finished = false;
	boolean clamping = false;

	public CollectBall() {
		requires(Robot.intakeSS);
		requires(Robot.elevatorSS);
		requires(Robot.shooterSS);

		t = new Timer();
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.states.intakeControlModeTracker = IntakeControlMode.AUTOMATIC;
		System.out.println("CollectBall() started");
		Robot.elevatorSS.resetSrxPID();
		Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
		Robot.elevatorSS.elevatorSetSetpoint(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		// TODO variable this height?
		if (Robot.elevatorSS.getElevatorPosition() < kElevatorSafeToCollect && !intaking) {
			// you can now run the intake to collect the ball since the claw is
			// in position to grab the ball
			Robot.intakeSS.intakeBall();
			System.out.println("intake rolling");
			intaking = true;
		} else if (Robot.shooterSS.hasBallSensor() && intaking && !ballSensed) {
			// you successfully intaked a ball and now you need to wait a bit
			// for it to center.
			new RumbleControllerConstant(Robot.oi.dc).start();
			ballSensed = true;
			System.out.println("sensed ball");
		} else if (ballSensed && !Robot.shooterSS.hasBallSensor()) {
			// ball failed to be seen by sensor
			Robot.oi.stopRumbles();
			ballSensed = false;
		}
		
		if(intaking && Robot.oi.oc.getButtonStart()){
			clamping = true;
			Robot.shooterSS.pinchBall();
			t.reset();
			t.start();
		}

		if (!Robot.oi.dc.getButtonLB() && !clamping) {
			// new RumbleController(Robot.oi.dc).start();
			if (ballSensed) {
				clamping = true;
				Robot.shooterSS.pinchBall();
				t.reset();
				t.start();
			} else {
				finished = true;
			}
		}

		else if (clamping && t.get() > .25) {
			t.stop();
			finished = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return finished;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.oi.stopRumbles();
		Robot.states.hasBall = ballSensed;
		new MoveToHoldingFromLow(ballSensed).start();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.oi.stopRumbles();
		Robot.intakeSS.stopIntaking();
	}
}
