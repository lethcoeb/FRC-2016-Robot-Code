package org.usfirst.frc.team1806.robot;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	//MOTORS
    public static final int rightMotor1 = 3;
    public static final int rightMotor2 = 4;
    public static final int rightMotor3 = 5;
    public static final int leftMotor1 = 0;
    public static final int leftMotor2 = 1;
    public static final int leftMotor3 = 2;
    
    public static final int intakeRoller = 6;
    
    public static final int liftMotor = 7;
    
    public static final int cockingMotor = 8;
    
    //SOLENOIDS
    //double solenoids
    public static final int shiftLow = 0;
    public static final int shiftHigh = 1;
    public static final int deployerExtend = 2;
    public static final int deployerRetract = 3;
    public static final int pincherPinch = 4;
    public static final int pincherRelease = 5;
    
    //single solenoids
    public static final int winchEngageSolenoid = 6;
    public static final int shooterReleaseSolenoid = 7;
    
    //SENSORS
    public static final int rightEncoderA = 0;
    public static final int rightEncoderB = 1;
    public static final int leftEncoderA = 2;
    public static final int leftEncoderB = 3;
    public static final int liftEncoderA = 4;
    public static final int liftEncoderB = 5;
    
    public static final int bottomElevatorLimit = 6;
    public static final int topElevatorLimit = 7;
    
    public static final int cockedShooterLimit = 8;
    
    public static final int hasBallSensor = 9;

}
