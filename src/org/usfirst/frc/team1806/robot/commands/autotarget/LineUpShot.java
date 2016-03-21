package org.usfirst.frc.team1806.robot.commands.autotarget;

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

/**
 *
 */
public class LineUpShot extends Command {

	boolean goalFound, withinRange, hasRumbled;
	
	double targetAngle;
	double voltage;
	int stage;
	
	Timer autoTimer;

	public LineUpShot() {
		requires(Robot.drivetrainSS);
		Robot.states.autoLiningUp = true;
	}

	// Called just before this Command runs the first time
	protected void initialize() {

		Robot.drivetrainSS.resetYaw();
		goalFound = false;
		withinRange = false;
		hasRumbled = false;
		Robot.drivetrainSS.drivetrainTurnAbsolutePIDchangePID(Constants.drivetrainTurnP, Constants.drivetrainTurnI,
				Constants.drivetrainTurnD);
		
		autoTimer = new Timer();

		System.out.println("auto line up started");

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		if (!goalFound) {
			// allow driver to move the drivetrain to where the robot sees the
			// goal
			if (Robot.jr.isGoalFound()) {
				System.out.println("goal found!");
				goalFound = true;
				Robot.states.driveControlModeTracker = DriveControlMode.AUTO;
				System.out.println(Robot.jr.getAngleToGoal());
				targetAngle = Robot.jr.getAngleToGoal();
				Robot.drivetrainSS.shiftLow();
				evaluateSpeed();
				
				

			} else if (Robot.states.mode == Mode.TELEOP) {
				Robot.drivetrainSS.execute(Robot.drivetrainSS.zoneInput(Robot.oi.dc.getLeftJoyY()),
						Robot.drivetrainSS.zoneInput(-Robot.oi.dc.getRightJoyX()));
			} else if (Robot.states.mode == Mode.AUTONOMOUS) {
				// TODO make a 'autosearch' function
			}
		} else if (goalFound && !withinRange) {
			// found goal now line up
			if (Math.abs(targetAngle - Robot.drivetrainSS.getYaw()) < .5){

				System.out.println("on target");
				withinRange = true;
				Robot.drivetrainSS.arcadeDrive(0, 0);


			}else{
				evaluateSpeed();
				autoTimer.reset();
				autoTimer.start();
				while(autoTimer.get() <= .1){
					
				}
				
			}
		} else if (goalFound && withinRange && !hasRumbled && Robot.states.mode == Mode.TELEOP) {
			System.out.println("rumbled");
			new RumbleController(Robot.oi.dc).start();
			//This makes the command recursive
			//withinRange = false;
			hasRumbled = true;
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return ((hasRumbled || Robot.oi.dc.getRightTrigger() < .5) && Robot.states.mode == Mode.TELEOP)
				|| (withinRange && Robot.states.mode == Mode.AUTONOMOUS);
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("lineupshot finished");

		Robot.drivetrainSS.drivetrainTurnPIDDisable();
		Robot.drivetrainSS.arcadeDrive(0, 0);
		Robot.states.autoLiningUp = false;

		if (Robot.states.mode == Mode.TELEOP) {
			
			
			if(Robot.oi.dc.getRightTrigger() > .5){
				new LineUpShot().start();
			}else{
				Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
				new DriverControlDrivetrain().start();
			}
			
			
		}else if(Robot.states.mode == Mode.AUTONOMOUS){
			
			
			if(Math.abs(Robot.jr.getAngleToGoal()) > .5){
				new LineUpShot().start();
			}else{
				new ShootThenCock().start();
			}
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.drivetrainSS.drivetrainTurnPIDDisable();
		Robot.states.autoLiningUp = false;

		if (Robot.states.mode == Mode.TELEOP) {
			Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
			new DriverControlDrivetrain().start();
		}
	}
	
	private void evaluateSpeed(){
		voltage = Robot.pdp.getVoltage();
		if(Math.abs(targetAngle - Robot.drivetrainSS.getYaw()) > 10){
			//10 degree stage
			stage = 3;
			Robot.drivetrainSS.arcadeDrive(0, -.3 * Math.signum(targetAngle  - Robot.drivetrainSS.getYaw()));
			
			
			
		}
		
		else if(Math.abs(targetAngle  - Robot.drivetrainSS.getYaw()) > 5){
			Robot.drivetrainSS.arcadeDrive(0, -.25 * Math.signum(targetAngle  - Robot.drivetrainSS.getYaw()));
		}
		
		else if(Math.abs(targetAngle  - Robot.drivetrainSS.getYaw()) > 3.5){
			Robot.drivetrainSS.arcadeDrive(0, -.20 * Math.signum(targetAngle  - Robot.drivetrainSS.getYaw()));
		}
		
		else if(Math.abs(targetAngle  - Robot.drivetrainSS.getYaw()) > 2){
			stage = 2;
			Robot.drivetrainSS.arcadeDrive(0, -.195 * Math.signum(targetAngle  - Robot.drivetrainSS.getYaw()));
		}else if(Math.abs(targetAngle - Robot.drivetrainSS.getYaw()) >= .4){
			stage = 1;
			Robot.drivetrainSS.arcadeDrive(0, -.19 * Math.signum(targetAngle  - Robot.drivetrainSS.getYaw()));
		}else{
			Robot.drivetrainSS.arcadeDrive(0, 0);
		}
	}
}

/*
 * Broken stuff from 3/9/2013 8:07 PM. package
 * org.usfirst.frc.team1806.robot.commands.autotarget;
 * 
 * 
 * import org.usfirst.frc.team1806.robot.Constants; import
 * org.usfirst.frc.team1806.robot.OperatorInterface; import
 * org.usfirst.frc.team1806.robot.Robot; import
 * org.usfirst.frc.team1806.robot.RobotStates.DriveControlMode; import
 * org.usfirst.frc.team1806.robot.commands.RumbleController; import
 * org.usfirst.frc.team1806.robot.commands.RumbleControllerConstant;
 * 
 * import edu.wpi.first.wpilibj.command.Command;
 * 
 * /**
 *
 */
/*
 * public class LineUpShot extends Command {
 * 
 * boolean goalFound, withinRange, hasRumbled;
 * 
 * boolean finished; double targetAngle; int stage; double loops = 0; double
 * kLoopsUntilCheck = Constants.loopsToCheckSensorDisconnect;
 * 
 * public LineUpShot() { requires(Robot.drivetrainSS); Robot.states.autoLiningUp
 * = true;
 * 
 * }
 * 
 * // Called just before this Command runs the first time protected void
 * initialize() {
 * 
 * //Robot.drivetrainSS.resetYaw(); goalFound = false; withinRange = false;
 * hasRumbled = false;
 * 
 * if(Robot.jr.isGoalFound()){ targetAngle = Robot.jr.getAngleToGoal(); }
 * 
 * Robot.states.driveControlModeTracker = DriveControlMode.AUTO;
 * Robot.drivetrainSS.drivetrainTurnPIDReset(); Robot.drivetrainSS.resetYaw();
 * 
 * if(Math.abs(targetAngle) > Constants.drivetrainTurnPID1Tolerance){ //use big
 * ol' pid
 * Robot.drivetrainSS.drivetrainTurnPIDchangePID(Constants.drivetrainTurn1P,
 * Constants.drivetrainTurn1I, Constants.drivetrainTurn1D);
 * Robot.drivetrainSS.drivetrainTurnPIDSetTolerance(Constants.
 * drivetrainTurnPID1Tolerance);
 * Robot.drivetrainSS.drivetrainTurnPIDchangeMaxRotation(Constants.
 * drivetrainMaxRotationPIDStage3); stage = 3; }else if(Math.abs(targetAngle) >
 * Constants.drivetrainTurnPID2Tolerance){
 * Robot.drivetrainSS.drivetrainTurnPIDchangePID(Constants.drivetrainTurn2P,
 * Constants.drivetrainTurn2I, Constants.drivetrainTurn2D);
 * Robot.drivetrainSS.drivetrainTurnPIDSetTolerance(Constants.
 * drivetrainTurnPID2Tolerance);
 * Robot.drivetrainSS.drivetrainTurnPIDchangeMaxRotation(Constants.
 * drivetrainMaxRotationPIDStage2); stage = 2; }else{ //you hella close bruh use
 * dat small loop
 * Robot.drivetrainSS.drivetrainTurnPIDchangePID(Constants.drivetrainTurn3P,
 * Constants.drivetrainTurn3I, Constants.drivetrainTurn3D);
 * Robot.drivetrainSS.drivetrainTurnPIDSetTolerance(Constants.
 * drivetrainTurnPID3Tolerance);
 * Robot.drivetrainSS.drivetrainTurnPIDchangeMaxRotation(Constants.
 * drivetrainMaxRotationPIDStage1); stage = 1; }
 * 
 * Robot.drivetrainSS.drivetrainTurnPIDSetSetpoint(targetAngle);
 * Robot.drivetrainSS.drivetrainTurnPIDEnable(); System.out.println(
 * "auto line up started");
 * 
 * }
 * 
 * // Called repeatedly when this Command is scheduled to run protected void
 * execute() {
 * 
 * //System.out.println(Robot.jr.getAngleToGoal());
 * 
 * if(!goalFound){ //allow driver to move the drivetrain to where the bot sees
 * the goal if(Robot.jr.isGoalFound()){ System.out.println("goal found!");
 * goalFound = true; Robot.states.driveControlModeTracker =
 * DriveControlMode.AUTO; System.out.println(Robot.jr.getAngleToGoal());
 * //Robot.drivetrainSS.drivetrainTurnPIDSetSetpoint(Robot.jr.getAngleToGoal());
 * Robot.drivetrainSS.drivetrainTurnPIDEnable();
 * 
 * } }else if(goalFound && !withinRange){ //found goal now line up
 * 
 * loops++; if(loops >= kLoopsUntilCheck){ loops = 0;
 * if(!Robot.drivetrainSS.isNavxConnected()){ //navx is dead bruh kill it
 * Robot.drivetrainSS.drivetrainTurnPIDDisable(); finished = true;
 * System.out.println("Kill early because navx die"); } }
 * if(Robot.drivetrainSS.drivetrainTurnPIDisOnTarget()){ //System.out.println(
 * "PID loop on target");
 * 
 * if(stage == 3){ //step down to stage 2 stage = 2;
 * Robot.drivetrainSS.drivetrainTurnPIDDisable();
 * Robot.drivetrainSS.drivetrainTurnPIDReset();
 * Robot.drivetrainSS.drivetrainTurnPIDchangePID(Constants.drivetrainTurn2P,
 * Constants.drivetrainTurn2I, Constants.drivetrainTurn2D);
 * Robot.drivetrainSS.drivetrainTurnPIDSetTolerance(Constants.
 * drivetrainTurnPID2Tolerance);
 * Robot.drivetrainSS.drivetrainTurnPIDchangeMaxRotation(Constants.
 * drivetrainMaxRotationPIDStage2); /*if(Robot.jr.isGoalFound()){
 * Robot.drivetrainSS.drivetrainTurnAbsolutePIDSetSetpoint(Robot.jr.
 * getAngleToGoal()); }
 */
/*
 * Robot.drivetrainSS.drivetrainTurnPIDEnable();
 * 
 * 
 * System.out.println("moving to stage 2"); }else if(stage == 2 &&
 * Robot.jr.isAngleAcceptable()){ //step down to stage 1 stage = 1;
 * Robot.drivetrainSS.drivetrainTurnPIDDisable();
 * 
 * Robot.drivetrainSS.drivetrainTurnPIDchangePID(Constants.drivetrainTurn3P,
 * Constants.drivetrainTurn3I, Constants.drivetrainTurn3D);
 * Robot.drivetrainSS.drivetrainTurnPIDSetTolerance(Constants.
 * drivetrainTurnPID3Tolerance);
 * Robot.drivetrainSS.drivetrainTurnPIDchangeMaxRotation(Constants.
 * drivetrainMaxRotationPIDStage3); /*if(Robot.jr.isGoalFound()){
 * Robot.drivetrainSS.drivetrainTurnPIDSetSetpoint(Robot.jr.getAngleToGoal());
 * System.out.println(Robot.jr.getAngleToGoal()); }
 */
/*
 * Robot.drivetrainSS.drivetrainTurnPIDReset();
 * Robot.drivetrainSS.drivetrainTurnPIDEnable();
 * 
 * System.out.println("moving to stage 1");
 * 
 * Robot.drivetrainSS.drivetrainTurnPIDDisable();
 * Robot.drivetrainSS.drivetrainTurnPIDReset(); withinRange = true;
 * System.out.println("stage one done, ON TARGET");
 * 
 * }/*else if(stage == 1 && Robot.jr.isAngleAcceptable()){ //should be done.
 * 
 * Robot.drivetrainSS.drivetrainTurnPIDDisable();
 * Robot.drivetrainSS.drivetrainTurnPIDReset(); withinRange = true;
 * System.out.println("stage one done, ON TARGET"); }
 */
/*
 * } //else if(Robot.drivetrainSS.drivetrainTurnAbsolutePIDisOnTarget()){ /* If
 * for some reason our original angle was wrong, being that we aren't in an
 * acceptable range according to the Jetson, yet we got to the angle it
 * originally said the goal was at, try to line up again to the new angle.
 * 
 * the PID is disabled and re-enabled in order to reset the navX and clear
 * accumulated error.
 */
/*
 * Robot.drivetrainSS.drivetrainTurnPIDDisable();
 * Robot.drivetrainSS.drivetrainTurnPIDSetSetpoint(Robot.jr.getAngleToGoal());
 * Robot.drivetrainSS.drivetrainTurnPIDEnable(); }
 */
/*
 * }else if(goalFound && withinRange && !hasRumbled){
 * System.out.println("rumbled"); new RumbleController(Robot.oi.dc).start();
 * hasRumbled = true; }
 * 
 * }
 * 
 * // Make this return true when this Command no longer needs to run execute()
 * protected boolean isFinished() { return (Robot.oi.dc.getRightTrigger() < .4);
 * }
 * 
 * // Called once after isFinished returns true protected void end() {
 * 
 * System.out.println("lineupshot finished");
 * 
 * Robot.drivetrainSS.drivetrainTurnPIDDisable(); Robot.states.autoLiningUp =
 * false; Robot.states.driveControlModeTracker = DriveControlMode.DRIVER; }
 * 
 * // Called when another command which requires one or more of the same //
 * subsystems is scheduled to run protected void interrupted() {
 * Robot.drivetrainSS.drivetrainTurnPIDDisable(); Robot.states.autoLiningUp =
 * false; Robot.states.driveControlModeTracker = DriveControlMode.DRIVER; } }
 */