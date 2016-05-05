package org.usfirst.frc.team1806.robot;

public class Constants {
	
	//lit new auto stuff
	public static final double[] autoForwardsNearGoalAngles = {62, 15, 6.5, -2.5, -10};
	public static final double[] autoBackwardsNearGoalAngles = {-150, -165, -175, 177, 170};
	public static final double distanceFromStartingPosToDefenses = 44;
	

	public static final double joystickDeadzone = .15;

	// FIXME this
	//public static final double encoderCountsPerRevolution = .01;
	public static final double encoderCountsPerRevolution = .0947368421;
	public static final double drivetrainMaxRotationPIDStage1 = 1; //degrees per second
	public static final double drivetrainMaxRotationPIDStage2 = .2; //degrees per second
	public static final double drivetrainMaxRotationPIDStage3 = .15; //degrees per second

	// DRIVETRAIN THINGS
	public static final double drivetrainDriveP = .025;
	public static final double drivetrainDriveI = 0.0001;
	public static final double drivetrainDriveD = 0.03;
	public static final double drivetrainDrivePIDTolerance = 2; // inches
	// AUTO SHIFTING STUFF
	public final static double drivetrainMaxLowGearSpeed = 3; //exceeding this speed will force an autoshift to high gear.
	public final static double drivetrainPowerDownshiftPowerThreshold = .85 ; //power must be over this
	public final static double drivetrainDownshiftPowerThreshold = .03; // power must be less than this
	public final static double drivetrainDownshiftSpeedThreshold = 7; //speed must be less than this
	public final static double drivetrainUpshiftPowerThreshold = .69; //power must be more than this
	public final static double drivetrainAccelerationThreshold = .2; // applied for determining actual accel for shifting
	public final static double drivetrainUpshiftSpeedThreshold = 6; //speed must be more than this
	
	//jones wants to do only i
	//public static final double drivetrainTurnP = .07;
	public static final double drivetrainTurnP = .15;
	public static final double drivetrainTurnI = 0.002;
	public static final double drivetrainTurnD = 0.1;

	//Stage 1 is for big turns (30+ deg), stage 2 is for 30 to 5 deg, stage 3 is for <5 degree turns
	//90 degree values
	public static final double drivetrainTurn1P = .035;
	public static final double drivetrainTurn1I = 0;
	public static final double drivetrainTurn1D = 0.2;
	//5 degree values
		//.5deg
	public static final double drivetrainTurn2P = .01;
	public static final double drivetrainTurn2I = 0.001;
	public static final double drivetrainTurn2D = 0.0025;
	

	
	//tiny lil values
	public static final double drivetrainTurn3P = .065;
	public static final double drivetrainTurn3I = 0;
	//public static final double drivetrainTurn3D = 0.15;
	public static final double drivetrainTurn3D = 0;

	
	
	//public static final double drivetrainTurnD = 0.0025;
	public static final double drivetrainTurnPID1Tolerance = 10; // degrees
	public static final double drivetrainTurnPID2Tolerance = 2; // degrees
	public static final double drivetrainTurnPID3Tolerance = .2; // degrees
	
	public static final double navxMinPitchToBeFlat = 4;
	public static final double timeUntilFlat = .75;
	public static final double timeUntilNotFlat = .5;

	public static final double maxPowerDiffential = .05;
	public static final double maxTurnPowerDifferential = .2;

	public static final int loopsToCheckSensorDisconnect = 10;
	// ELEVATOR THINGS

	public static final double elevatorPIDp = 0.15;
	public static final double elevatorPIDi = 0.0006;
	public static final double elevatorPIDd = 0;
	//public static final double elevatorAbsoluteTolerance = 25;
	public static final double elevatorAbsoluteTolerance = 500;
	public static final double elevatorDownPIDp = .12;
	public static final double elevatorDownPIDi = 0;
	public static final double elevatorDownPIDd = 0;

	public static final int elevatorBatterShotHeight = 103500;
	public static final int elevatorShootingHeight = 94500;
	//public static final int elevatorShootingHeight = 96000; //old arm
	//public static final int elevatorHoldingHeight = 20000;
	public static final int elevatorHoldingHeight = 26000;
	public static final int elevatorChevaldeFunHeight = 75000; //lowest height at which the intake can be raised and lowered
	public static final double resetSpeed = -.35;
	public static final double collectingIntakeStopOffset = -3000;
	
	public static final double elevatorIntakeEngagedHeight = 30000;
	
	public static final int elevatorPickupEncPosToEngage = 4000;
	
	public static final double elevatorPIDEngageDistance = 10000;
	// position where the ball is no longer engaged w/ the intake while held
	// with the claw. After the elevator surpasses this height,
	// it's safe to move the elevator around without using the collector to help
	// it

	public static final double elevatorSafeToCollect = 2000;
	// height at which it's safe to start running the intake. This is to avoid
	// sucking the ball in before the claw is ready

	// INTAKE STUFF
	public static final double intakeRollerSpeed = .5;
	public static final double outtakeRollerSpeed = -.8;
	
	public static final double intakeTimeToLower = .5;
	public static final double intakeTimeToRaise = .75;
	public static final double intakeTimeToCenterBall = 1;
	public static final double intakeSpeedToMatchArm = .625;

	// SHOOTER STUFF
	public static final double timeToEngageDogGear = .25;
	public static final double gearEngageSpeed = .5;
	public static final double timeToPinch = .5;
	public static final double timeToUnpinch = .15;
	public static final double timeToSettle = .7; // after 'ungrabbing' the
													// ball with the claw at
													// shooting height this is
													// the time at which the
													// shooter will wait for the
													// ball to settle.
	public static final double timeToShoot = 1.25; // Time in seconds that it takes
												// for the puncher to travel all
												// the way through,
												// after this time elapses you
												// can begin recocking bc the
												// shooter is unmoving
	public static final double minTimeToCock = 1;
	
	/*
	 AutoShoting constants
	 */
	public static final double ShootingmaxAngleError = 0.5;
	//TODO: Make distance range reasonable.
	public static final double ShootingMinGoalDistance = 4;
	public static final double ShootingMaxGoalDistance = 12;
	public static final double ShootingJetsonCameraAngleOffset = 0;

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
