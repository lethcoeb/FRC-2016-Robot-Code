package org.usfirst.frc.team1806.robot.commands.autotarget;

import java.util.ArrayList;

import javax.print.attribute.standard.Compression;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.OperatorInterface;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates;
import org.usfirst.frc.team1806.robot.RobotStates.DriveControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.Mode;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;
import org.usfirst.frc.team1806.robot.commands.DriverControlDrivetrain;
import org.usfirst.frc.team1806.robot.commands.RumbleController;
import org.usfirst.frc.team1806.robot.commands.autonomous.RobotReset;
import org.usfirst.frc.team1806.robot.commands.shooter.ShootThenCock;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LineUpShot extends Command {

	public enum direction {
		LEFT, RIGHT
	}

	direction directionTracker;

	boolean goalFound, withinRange, hasRumbled;

	double targetAngle;
	double voltage;
	double voltageCompensation = 1;
	int stage;

	Timer autoTimer;
	double pulsePower;
	double pulseWidth;

	double overshoot;
	double currYaw;

	boolean finished;

	boolean PIDEnabled = false;

	int loops = 0;
	int goalLoops;
	int step;
	
	

	ArrayList<Double> yawTable = new ArrayList<Double>();

	public LineUpShot() {
		requires(Robot.drivetrainSS);
		Robot.states.autoLiningUp = true;
		Robot.drivetrainSS.resetYaw();
		if(Robot.states.pulsePower != null){
			step = 2;
			pulsePower = Robot.states.pulsePower;
			System.out.println("Using global pulsepower");
		}else{
			step = 1;
			pulsePower = .16;
		}
	}

	// Called just before this Command runs the first time
	protected void initialize() {

		Robot.drivetrainSS.shiftLow();
		Robot.compressor.stop();

		// Determine angle from goal

		goalFound = Robot.jr.isGoalFound();
		if(goalFound){
			targetAngle = Robot.jr.getAngleToGoal();
		}else{
			targetAngle = -10;
		}

		// TODO fix this
		if (Math.signum(targetAngle) > 0) {
			directionTracker = direction.LEFT;
		} else {
			directionTracker = direction.RIGHT;
		}

		voltage = Robot.pdp.getVoltage();

		goalLoops = Math.abs((int) ((targetAngle / .1)));
		// System.out.println("GoalLoops: " + goalLoops);

		overshoot = Robot.states.overshoot;
		// new algorithm to compensate for overshoot
		if (overshoot != 0) {
			goalLoops = (int) (goalLoops * overshoot);
		}

		// Old algorithm to compensate for overshoot
		// goalLoops = goalLoops - (int) (Robot.states.overshoot * 10);
		if (goalLoops < 1) {
			goalLoops = 1;
		}

		if (Math.abs(targetAngle) < .5 || Robot.states.autoalignmentShooting) {
			goalLoops = 0;
			
			if(Robot.oi.dX && Robot.states.hasBall && !Robot.states.autoalignmentShooting){
				Robot.states.autoalignmentShooting = true;
				new ShootThenCock().start();
			}
			
		} else {
			Robot.states.loopsUntilDone++;
		}

		if (voltage > 12.6) {
			voltageCompensation = 1;
		} else {
			voltageCompensation = ((12.6 - voltage) * .5) + 1;
			if (voltageCompensation >= 1.2) {
				voltageCompensation = 1.2;
			}
		}
		

		// pulsePower = .27 * voltageCompensation;
		

		if (targetAngle > 0) {
			// give more power if turning right because mechanical issues :-(
			// pulsePower = pulsePower * 1.2;
		}

		pulseWidth = .01;

		autoTimer = new Timer();
		autoTimer.reset();
		autoTimer.start();

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		/*
		 * int stepCount = 3;
		 * 
		 * for(int i = 0; i < goalLoops && !finished; i+= stepCount){
		 * 
		 * for(int j = 0; j < stepCount; j++){
		 * 
		 * pulse();
		 * 
		 * }
		 * 
		 * if(Math.abs(targetAngle - Robot.drivetrainSS.getYaw()) <= .5){
		 * finished = true; }
		 * 
		 * else if(Math.abs(targetAngle - Robot.drivetrainSS.getYaw()) <=
		 * stepCount * .5){ targetAngle = Robot.jr.getAngleToGoal(); pulse();
		 * finished = true; }
		 * 
		 * }
		 */

		

		double minYawChange = .1;

		switch (step) {

		// finding min pulsepower
		case 1: {
			
			Robot.drivetrainSS.arcadeDrive(0, -pulsePower);
			boolean moving = true;
			int loops = 0;
			//fill table w/ entries
			while (loops < 5) {
				
				Robot.drivetrainSS.arcadeDrive(0, -pulsePower);
				currYaw = Robot.drivetrainSS.getYaw();
				yawTable.add(currYaw);
				while (yawTable.size() > 5) {
					// keep table size at 5 entries
					yawTable.remove(0);
				}
				
				autoTimer.reset();
				autoTimer.start();
				while(autoTimer.get() < .02){
					
				}
				
				loops++;

			}
			
			for (int i = 1; i < yawTable.size(); i++) {
				if (Math.abs(yawTable.get(i)) - Math.abs(yawTable.get(i - 1)) < minYawChange) {
					moving = false;
				} else {
					
				}
			}
			
			//dont let it get too big
			if(pulsePower > .3){
				moving = true;
			}

			if (moving) {
				step = 2;
				System.out.println("Found pulsePower: " + pulsePower);
				Robot.drivetrainSS.arcadeDrive(0, 0);
				//set global power so robot remembers it
				Robot.states.pulsePower = pulsePower + .01;
			} else {
				
				/*autoTimer.reset();
				autoTimer.start();
				while(autoTimer.get() < .15){
					
				}*/
				
				pulsePower = pulsePower + .01;
				System.out.println("Changing pulsePower to " + pulsePower);
				
			}

		}

			// running goalloops cycle
		case 2: {
			
			currYaw = Robot.drivetrainSS.getYaw();
			yawTable.add(currYaw);
			while (yawTable.size() > 5) {
				// keep table size at 5 entries
				yawTable.remove(0);
			}
			
			if (loops < goalLoops) {
				if (autoTimer.get() >= pulseWidth) {
					loops++;
					autoTimer.reset();
					autoTimer.start();

					if (Math.abs(currYaw - targetAngle) <= 2.5) {
						Robot.drivetrainSS.arcadeRight(0, (pulsePower - .02) * -Math.signum(targetAngle));
					} else {
						Robot.drivetrainSS.arcadeDrive(0, pulsePower * -Math.signum(targetAngle));
					}

					// Robot.drivetrainSS.arcadeRight(0, pulsePower *
					// -Math.signum(targetAngle));
				}
			} else {
				// done
				Robot.drivetrainSS.arcadeDrive(0, 0);
				finished = true;
			}

			if (/*
				 * Math.abs(Robot.drivetrainSS.getYaw() - targetAngle) < .5 ||
				 */(Robot.states.mode == Mode.TELEOP && Robot.oi.dc.getRightTrigger() < .5)) {
				finished = true;
			}

			if(step == 2){
			boolean overshooting = true;
			if (yawTable.size() == 5) {
				for (int i = 1; i < yawTable.size(); i++) {
					if (Math.abs(yawTable.get(i) - targetAngle) > Math.abs(yawTable.get(i - 1) - targetAngle)) {
						// The difference between the target angle and current
						// angle is GROWING
						// keep overshooting var to true
					} else {
						// You're not overshooting so set variable to false,
						// it'll stay there
						overshooting = false;
					}
				}
				if (overshooting) {
					System.out.println("Robot is overshooting, killing command");
					finished = true;

					// New overshoot algorithm//
					// Returns overshoot/undershoot as percentage
					Robot.states.overshoot = 1 - ((currYaw - targetAngle) / currYaw);
					if (Robot.states.overshoot < 0) {
						Robot.states.overshoot = Math.abs(Robot.states.overshoot);
					}

					System.out.println("Overshoot: " + Robot.states.overshoot + ", currYaw: " + currYaw
							+ ", targetAngle: " + targetAngle);

					// Old overshoot algorithm
					// Robot.states.overshoot = (Math.abs(currYaw - targetAngle)
					// / .1);
					System.out.println("Overshoot: " + Robot.states.overshoot);
				}
			}
			}
		}

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

		currYaw = Robot.drivetrainSS.getYaw();

		// System.out.println("lineupshot finished");

		Robot.drivetrainSS.arcadeDrive(0, 0);

		
		/*if(Robot.states.mode == Mode.AUTONOMOUS){
		autoTimer.reset();
		autoTimer.start();

		
		while (autoTimer.get() < .1) {

		}
		}*/

		

		if (Robot.states.mode == Mode.TELEOP && Robot.oi.dc.getRightTrigger() < .5) {

			Robot.states.autoLiningUp = false;
			Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
			new DriverControlDrivetrain().start();
			Robot.states.overshoot = 0;
			Robot.compressor.setClosedLoopControl(true);
			Robot.compressor.start();

		} else

		if (Robot.states.mode == Mode.AUTONOMOUS) {
			/*
			 * autoTimer.reset(); autoTimer.start();
			 * 
			 * while (autoTimer.get() < .2) {
			 * 
			 * }
			 */

			// If your angle offset is too big or your arm isnt all the way up
			// repeat this command

			// Use currYaw since it was last calculated in the execute method,
			// like targetAngle
			if (Math.abs(Robot.jr.getAngleToGoal()) >= .4) {
				new LineUpShot().start();
			} else {
				
				autoTimer.reset();
				autoTimer.start();
				while(autoTimer.get() < .2){
					
				}if(Math.abs(Robot.jr.getAngleToGoal()) < .4){
				
				Robot.states.autoLiningUp = false;
				Robot.states.shooterArmPositionTracker = ShooterArmPosition.UP;
				new ShootThenCock().start();

				Robot.compressor.setClosedLoopControl(true);
				Robot.compressor.start();
				}else{
					new LineUpShot().start();
				}

			}
		}

		else if (Math.abs(currYaw - targetAngle) >= .5) {
			new LineUpShot().start();
		}

		else if (Robot.states.mode == Mode.AUTONOMOUS) {
			Robot.states.autoLiningUp = false;
			Robot.states.shooterArmPositionTracker = ShooterArmPosition.UP;
			new ShootThenCock().start();

			Robot.compressor.setClosedLoopControl(true);
			Robot.compressor.start();

		}

	}

	private void pulse() {
		autoTimer.reset();
		autoTimer.start();

		while (autoTimer.get() < pulseWidth) {
			Robot.drivetrainSS.arcadeDrive(0, pulsePower * -Math.signum(targetAngle));
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