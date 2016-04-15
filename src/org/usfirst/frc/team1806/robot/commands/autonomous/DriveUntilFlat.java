package org.usfirst.frc.team1806.robot.commands.autonomous;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.DriveControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveUntilFlat extends Command {

	Double commandTimeout;
	double maxSpeed;
	//double kTimeUntilFlat = Constants.timeUntilFlat;
	double kTimeUntilFlat = 0;

	Double minDistanceToTravel;

	boolean finished = false;
	boolean flat;
	boolean hasHit = false;
	Timer timeoutTimer;
	Timer flatTimer;

	public DriveUntilFlat(double speed) {
		requires(Robot.drivetrainSS);
		maxSpeed = speed;
	}

	public DriveUntilFlat(double speed, double timeout) {
		requires(Robot.drivetrainSS);
		maxSpeed = speed;
		commandTimeout = timeout;
	}

	public DriveUntilFlat(double speed, double timeout, double minDistance) {
		requires(Robot.drivetrainSS);
		maxSpeed = speed;
		commandTimeout = timeout;
		minDistanceToTravel = Math.abs(minDistance);
	}
	
	public DriveUntilFlat(double speed, boolean doesntMatter, double minDistance){
		//hacky constructor to get a min distance without setting a timeout
		maxSpeed = speed;
		minDistanceToTravel = minDistance;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		timeoutTimer = new Timer();
		flatTimer = new Timer();
		timeoutTimer.reset();
		flatTimer.reset();
		timeoutTimer.start();
		Robot.states.driveControlModeTracker = DriveControlMode.AUTO;
		if(minDistanceToTravel != null){
			Robot.drivetrainSS.resetEncoders();
		}
		Robot.drivetrainSS.resetYaw();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.drivetrainSS.arcadeDrive(maxSpeed, Robot.drivetrainSS.getYaw() * .1);

		if (commandTimeout != null) {
			if (timeoutTimer.get() >= commandTimeout) {
				Robot.drivetrainSS.arcadeDrive(.1, 0);
				finished = true;
			}
		}

		if (!hasHit) {
			if (!Robot.drivetrainSS.isNavxFlat()) {
				hasHit = true;
			}
		}

		else {
			if (Robot.drivetrainSS.isNavxFlat()) {
				if (flat && flatTimer.get() > kTimeUntilFlat) {
					if (minDistanceToTravel != null) {
						if (Math.abs(Robot.drivetrainSS.getLeftEncoderDistance()) > minDistanceToTravel
								|| Math.abs(Robot.drivetrainSS.getRightEncoderDistance()) > minDistanceToTravel) {
							finished = true;
						} else {
							finished = false;
						}
					} else {
						finished = true;
					}
				} else if (!flat) {
					flatTimer.reset();
					flatTimer.start();
					flat = true;
				}
			} else if (flat) {
				// no longer flat
				flat = false;
				flatTimer.reset();
			}

		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return finished;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.drivetrainSS.arcadeDrive(0, 0);
		Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
		timeoutTimer.stop();
		flatTimer.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
