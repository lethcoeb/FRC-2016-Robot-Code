package org.usfirst.frc.team1806.robot;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	//MOTORS
	public static final int leftMotor1 = 4;
    public static final int leftMotor2 = 5;
    public static final int leftMotor3 = 6;
    public static final int rightMotor1 = 9;
    public static final int rightMotor2 = 8;
    public static final int rightMotor3 = 7;
    
    public static final int intakeRoller = 3;
    //public static final int liftMotor = 1;
    public static final int cockingMotor = 2;
    public static final int tapeMeasureMotor = 0;
    
    //SOLENOIDS
    //double solenoids
    public static final int shiftLow = 0;
    public static final int shiftHigh = 1;
    //this is flipped
    public static final int deployerExtend = 3;
    public static final int deployerRetract = 2;
    public static final int pincherPinch = 4;
    public static final int pincherRelease = 5;
    
    //single solenoids
    /*public static final int winchEngageSolenoid = 6;
    public static final int shooterReleaseSolenoid = 7;*/
    public static final int winchEngageSolenoid = 6;
    public static final int shooterReleaseSolenoid = 7;
    
    //SENSORS
    public static final int rightEncoderA = 2;
    public static final int rightEncoderB = 3;
    public static final int leftEncoderA = 5;
    public static final int leftEncoderB = 4;
    public static final int cockedShooterLimit = 9;
    //4
    //5
    
    public static final int bottomElevatorLimit = 7;
    //public static final int topElevatorLimit = 7;
    
   public static final int hasBallSensor = 8;
   
   
   //PDP Slots
   //TODO Check this!
   public static final int PDPcockingWinchSlot = 13;
   public static final int PDPelevatorSlot = 12;
   
}
