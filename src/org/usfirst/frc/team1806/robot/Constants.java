package org.usfirst.frc.team1806.robot;

public class Constants {

	public static final double intakeRollerSpeed = .8;
	public static final double outtakeRollerSpeed = -.8;

	// FIXME this
	public static final double encoderCountsPerRevolution = .01;

	// DRIVETRAIN THINGS
	public static final double drivetrainDriveP = .5;
	public static final double drivetrainDriveI = 0;
	public static final double drivetrainDriveD = 0;
	public static final double drivetrainDrivePIDTolerance = 1; // inches

	//Stage 1 is for big turns (30+ deg), stage 2 is for 30 to 5 deg, stage 3 is for <5 degree turns
	//90 degree values
	public static final double drivetrainTurn1P = .03;
	public static final double drivetrainTurn1I = 0;
	public static final double drivetrainTurn1D = 0.1375;
	//5 degree values
		//.5deg
	public static final double drivetrainTurn2P = .1;
	public static final double drivetrainTurn2I = 0;
	public static final double drivetrainTurn2D = 0.0025;
	
	//jones wants to do only i
	/*public static final double drivetrainTurnP = .07;
	public static final double drivetrainTurnI = 0.002;
	public static final double drivetrainTurnD = 0.1;*/
	
	public static final double drivetrainTurn3P = .007;
	public static final double drivetrainTurn3I = 0.004;
	public static final double drivetrainTurn3D = 0.25;
	
	
	//public static final double drivetrainTurnD = 0.0025;
	public static final double drivetrainTurnPID3Tolerance = 10; // degrees
	public static final double drivetrainTurnPID2Tolerance = 5; // degrees
	public static final double drivetrainTurnPID1Tolerance = .25; // degrees
	public static final double drivetrainTurnMinPowerToMove = .15;

	public static final double maxPowerDiffential = .05;
	public static final double maxTurnPowerDifferential = .2;

	public static final int loopsToCheckSensorDisconnect = 10;
	// ELEVATOR THINGS
	public static final double elevatorPIDp = .01;
	public static final double elevatorPIDi = 0;
	public static final double elevatorPIDd = 0;
	public static final double elevatorAbsoluteTolerance = 1500;

	//public static final double elevatorShootingHeight = 103560;
	public static final double elevatorShootingHeight = 98112;
	public static final double elevatorHoldingHeight = 30000;
	public static final double resetSpeed = -.35;

	public static final double elevatorIntakeEngagedHeight = 30000;
	// position where the ball is no longer engaged w/ the intake while held
	// with the claw. After the elevator surpasses this height,
	// it's safe to move the elevator around without using the collector to help
	// it

	public static final double elevatorSafeToCollect = 2000;
	// height at which it's safe to start running the intake. This is to avoid
	// sucking the ball in before the claw is ready

	// INTAKE STUFF
	public static final double intakeTimeToLower = .5;
	public static final double intakeTimeToRaise = .75;
	public static final double intakeTimeToCenterBall = .25;

	// SHOOTER STUFF
	public static final double timeToEngageDogGear = .25;
	public static final double gearEngageSpeed = .5;
	public static final double timeToPinch = .75;
	public static final double timeToUnpinch = .15;
	public static final double timeToSettle = 1; // after 'ungrabbing' the
													// ball with the claw at
													// shooting height this is
													// the time at which the
													// shooter will wait for the
													// ball to settle.
	public static final double timeToShoot = 2; // Time in seconds that it takes
												// for the puncher to travel all
												// the way through,
												// after this time elapses you
												// can begin recocking bc the
												// shooter is unmoving
	public static final double minTimeToCock = 1;

	// Networking stuff
	public static final double jetsonConnectionLostTimeout = 1; // timeout until
																// connection
																// failure for
																// Jetson, if
																// time between
																// received info
																// surpasses
																// this value
																// (seconds)
																// then
																// switch to
																// onboard
																// vision
																// processing
	// AUTONOMOUS CONSTANTS

	/*
	 * ONE BALL!!!!!!!!!!!!!
	 */

	// distances are in inches

	// these two distances get you to just past the defense
	public static double overDefense = 36;
	public static double overDefensePlusDelay = 72; // because we also have to
													// start further back

	// this is the drivestraight defense for shooting the angle from low bar
	public static double lowBarToAngledShot = 54;

	// these 2 distances are the distances you need to drive from pos. 2 and 5
	// to get in front of defense 4 (the dream shot)
	public static double defense2toDefense4 = 48;
	public static double defense5toDefense4 = 24;

	// this angle is for what you need to turn to to aim roughly at the goal
	// after crossing defense 3
	public static double defense3angleToGoal = 15;
	public static double lowBarToShotDelaySeconds = 5;
	public static double lowBarAngleToGoal = 30;

	// STEAL AUTO
	public static double timeToDeployOnBall = 1;

}
