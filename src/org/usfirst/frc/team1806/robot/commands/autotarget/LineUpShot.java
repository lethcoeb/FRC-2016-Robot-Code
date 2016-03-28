package org.usfirst.frc.team1806.robot.commands.autotarget;

import javax.print.attribute.standard.Compression;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.OperatorInterface;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.DriveControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.Mode;
import org.usfirst.frc.team1806.robot.commands.DriverControlDrivetrain;
import org.usfirst.frc.team1806.robot.commands.RumbleController;
import org.usfirst.frc.team1806.robot.commands.shooter.ShootThenCock;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LineUpShot extends Command {

	boolean goalFound, withinRange, hasRumbled;

	double targetAngle;
	double voltage;
	int stage;

	Timer autoTimer;
	double pulsePower;
	double pulseWidth;

	boolean finished;

	boolean PIDEnabled = false;

	int loops = 0;
	int goalLoops;

	public LineUpShot() {
		requires(Robot.drivetrainSS);
		Robot.states.autoLiningUp = true;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		
		Robot.drivetrainSS.shiftLow();
		Robot.compressor.stop();

		// Determine angle from goal

		
		
		targetAngle = Robot.jr.getAngleToGoal();
		
		if(Math.abs(targetAngle) > 15){
			goalLoops = Math.abs((int) ((targetAngle / .1) * .6));
		}else if(Math.abs(targetAngle) > 8){
			goalLoops = Math.abs((int) ((targetAngle / .1) * .65));
		}else if(Math.abs(targetAngle) > 4){
			goalLoops = Math.abs((int) ((targetAngle / .1) * .8));
		}else{
			goalLoops = Math.abs((int) ((targetAngle / .1) * .5));
		}
		
		goalLoops = goalLoops - Robot.states.overshoot;
		if(goalLoops < 2){
			goalLoops = 2;
		}
		System.out.println(goalLoops);
		

		pulsePower = .20;
		pulseWidth = .01;

		Robot.drivetrainSS.resetYaw();
		
		autoTimer = new Timer();
		autoTimer.reset();
		autoTimer.start();

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		if (loops < goalLoops) {
			if (autoTimer.get() >= pulseWidth) {
				loops++;
				autoTimer.reset();
				autoTimer.start();
				Robot.drivetrainSS.arcadeDrive(0, pulsePower * -Math.signum(targetAngle));
			}
		}else{
			//done
			Robot.drivetrainSS.arcadeDrive(0, 0);
			finished = true;
		}
		
		if(Math.abs(Robot.drivetrainSS.getYaw() - targetAngle) < .5 || (Robot.states.mode == Mode.TELEOP && Robot.oi.dc.getRightTrigger() < .5)){
			finished = true;
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		/*
		 * return ((hasRumbled || Robot.oi.dc.getRightTrigger() < .5) &&
		 * Robot.states.mode == Mode.TELEOP) || (withinRange &&
		 * Robot.states.mode == Mode.AUTONOMOUS);
		 */
		return finished;
	}

	// Called once after isFinished returns true
	protected void end() {
		
		/*autoTimer.reset();
		autoTimer.start();
		
		while(autoTimer.get() < .1){
			
		}*/
		
		Robot.states.overshoot = Math.abs((int) ((Math.abs(Robot.drivetrainSS.getYaw() - targetAngle) / .1)));
		
		System.out.println("lineupshot finished");

		Robot.drivetrainSS.arcadeDrive(0, 0);
		Robot.states.autoLiningUp = false;

		if (Robot.states.mode == Mode.TELEOP && Robot.oi.dc.getRightTrigger() < .5) {

			Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
			new DriverControlDrivetrain().start();
			Robot.states.overshoot = 0;
			Robot.compressor.setClosedLoopControl(true);
			Robot.compressor.start();

		}else
		
		if(Robot.states.mode == Mode.AUTONOMOUS){
			autoTimer.reset();
			autoTimer.start();
			
			while(autoTimer.get() < .2){
				
			}
			
			if(Math.abs(Robot.drivetrainSS.getYaw() - targetAngle) >= .4){
				new LineUpShot().start();
			}else{
				new ShootThenCock().start();
				
				Robot.compressor.setClosedLoopControl(true);
				Robot.compressor.start();
			}
		}
		
		else if(Math.abs(Robot.drivetrainSS.getYaw() - targetAngle) >= .5){
			new LineUpShot().start();
		}
		
		else if (Robot.states.mode == Mode.AUTONOMOUS) {

			new ShootThenCock().start();
			
			Robot.compressor.setClosedLoopControl(true);
			Robot.compressor.start();

		}
		
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

		Robot.states.autoLiningUp = false;

		if (Robot.states.mode == Mode.TELEOP) {
			Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
			new DriverControlDrivetrain().start();
		}
		
		Robot.compressor.setClosedLoopControl(true);
		Robot.compressor.start();
	}
}