package org.usfirst.frc.team1806.robot;

public class Constants {
	
	public static final double intakeRollerSpeed = .75;
	public static final double outtakeRollerSpeed = -.75;
	
	
	//FIXME this
	public static final double encoderCountsPerRevolution = .01;
	
	public static final double liftEncoderCountsPerRevolution = .01;
	
	//DRIVETRAIN PID
	public static final double drivetrainDriveP = .5;
	public static final double drivetrainDriveI = 0;
	public static final double drivetrainDriveD = 0;
	public static final double drivetrainDrivePIDTolerance = .5; //.5 inches
	
	public static final double drivetrainTurnP = .1;
	public static final double drivetrainTurnI = 0;
	public static final double drivetrainTurnD = 0;
	public static final double drivetrainTurnPIDTolerance = .1; //.1 degrees
	
	//ELEVATOR PID
	public static final double elevatorPIDp = .01;
	public static final double elevatorPIDi = 0;
	public static final double elevatorPIDd = 0;
	public static final double elevatorAbsoluteTolerance = .25;
	
	public static final double elevatorShootingHeight = 1;
	public static final double resetSpeed = .2;
	
	public static final double maxPowerDiffential = .05;
	
	//INTAKE STUFF
	public static final double timeToLower = .5;
	public static final double timeToRaise = .75;
	
	//SHOOTER STUFF
	public static final double timeToEngageDogGear = .25;
	public static final double gearEngageSpeed = .5;
	public static final double timeToPinch = .5;
	public static final double timeToUnpinch = .5;
	public static final double timeToShoot = 1.5; //Time in seconds that it takes for the puncher to travel all the way through,
												  //after this time elapses you can begin recocking bc the shooter is unmoving
	//Networking stuff
	public static final double jetsonConnectionLostTimeout = 1; //timeout until conn. failure for Jetson
	//AUTONOMOUS CONSTANTS
	
	/*
	 * ONE BALL!!!!!!!!!!!!!
	 */
	
	//distances are in inches
	
	//these two distances get you to just past the defense
	public static double overDefense = 36;
	public static double overDefensePlusDelay = 72; //because we also have to start further back
	
	//this is the drivestraight defense for shooting the angle from low bar
	public static double lowBarToAngledShot = 54;
	
	//these 2 distances are the distances you need to drive from pos. 2 and 5 to get in front of defense 4 (the dream shot)
	public static double defense2toDefense4 = 48;
	public static double defense5toDefense4 = 24;
	
	//this angle is for what you need to turn to to aim roughly at the goal after crossing defense 3
	public static double defense3angleToGoal = 15;
	public static double lowBarToShotDelaySeconds = 5;
	public static double lowBarAngleToGoal = 30;
	
	//STEAL AUTO
	public static double timeToDeployOnBall = 1;
	
}
