package org.usfirst.frc.team1806.robot.commands.intake;

import java.sql.Blob;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.OperatorInterface;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.IntakeControlMode;
import org.usfirst.frc.team1806.robot.commands.RumbleController;
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
		}else if(Robot.shooterSS.hasBallSensor() && intaking && !ballSensed){
			//you successfully intaked a ball and now you need to wait a bit for it to center.
			t.start();
			ballSensed = true;
			System.out.println("sensed ball, centering");
		}else if(ballSensed && t.get() > kTimeToCenter){
			//ball was sensed and you've waited long enough for it to center. raise up the ball.
			Robot.shooterSS.pinchBall();
			Robot.states.hasBall = true;
			new RumbleController(Robot.oi.dc).start();
			System.out.println("ball centered, done");
			finished = true;
		}else if(!Robot.oi.dc.getButtonLB()){
			//new RumbleController(Robot.oi.dc).start();
			finished = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return finished;
	}

	// Called once after isFinished returns true
	protected void end() {
		new MoveToHoldingFromLow().start();
		Robot.intakeSS.stopIntaking();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.intakeSS.stopIntaking();
	}
}
