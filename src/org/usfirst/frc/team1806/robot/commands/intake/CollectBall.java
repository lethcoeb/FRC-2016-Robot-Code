package org.usfirst.frc.team1806.robot.commands.intake;

import java.sql.Blob;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.OperatorInterface;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.IntakeControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.IntakeRollerState;
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
	boolean pinched = false;
	boolean ok = false;
	
	
	public CollectBall() {
		
		Robot.states.collectingBalling = true;
		
		requires(Robot.intakeSS);
		requires(Robot.elevatorSS);
		requires(Robot.shooterSS);

		t = new Timer();
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		
		Robot.states.collectingBalling = true;
		Robot.states.intakeControlModeTracker = IntakeControlMode.AUTOMATIC;
		System.out.println("CollectBall() started");
		Robot.elevatorSS.resetSrxPID();
		Robot.elevatorSS.elevatorSetPIDValues(Constants.elevatorDownPIDp, Constants.elevatorDownPIDi, Constants.elevatorDownPIDd);
		Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
		Robot.elevatorSS.elevatorSetSetpoint(0);
		Robot.states.intakeRollerStateTracker = IntakeRollerState.INTAKING;
		
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.elevatorSS.getElevatorPosition() < kElevatorSafeToCollect && !intaking) {
			// you can now run the intake to collect the ball since the claw is
			// in position to grab the ball
			Robot.intakeSS.intakeBall();
			intaking = true;
			System.out.println("intake rolling");
		} else if ((Robot.shooterSS.hasBallSensor() || Robot.oi.oc.getLeftTrigger() > .5) && intaking && !ballSensed) {
			// you successfully intaked a ball and now you need to wait a bit
			// for it to center.
			new RumbleController(Robot.oi.dc).start();
			ballSensed = true;
			System.out.println("sensed ball");
		} else if (ballSensed && !Robot.shooterSS.hasBallSensor()) {
			// ball failed to be seen by sensor
			Robot.oi.stopRumbles();
			ballSensed = false;
		}
		
		if(intaking && ballSensed && !ok){
			//killer variable names
			//Pinch the ball if you haven't already
			ok = true;
			System.out.println("pinching");
			clamping = true;
			//Robot.shooterSS.pinchBall();
			pinched = true;
			t.reset();
			t.start();
		}

		if (!Robot.oi.dc.getButtonLB() && !clamping) {
			// new RumbleController(Robot.oi.dc).start();
			if (ballSensed) {
				clamping = true;
				
			} else {
				finished = true;
			}
		}

		else if (clamping && t.get() >= 0) {
			//If you've been clamped for more than .two seconds finish the command and bring the ball up to holding height
			System.out.println("finishing");
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
		Robot.states.intakeRollerStateTracker = IntakeRollerState.STOPPED;
		t.stop();
		Robot.oi.stopRumbles();
		Robot.states.hasBall = pinched;
		Robot.states.collectingBalling = false;
		new MoveToHoldingFromLow(pinched, pinched).start();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		
		Robot.states.intakeRollerStateTracker = IntakeRollerState.STOPPED;
		t.stop();
		Robot.states.hasBall = pinched;
		Robot.oi.stopRumbles();
		Robot.intakeSS.stopIntaking();
		Robot.states.collectingBalling = false;
	}
}
