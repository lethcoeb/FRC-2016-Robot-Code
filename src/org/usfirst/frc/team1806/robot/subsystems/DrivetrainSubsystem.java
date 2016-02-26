package org.usfirst.frc.team1806.robot.subsystems;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.RobotStates.DrivetrainGear;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DrivetrainSubsystem extends Subsystem {
    
    Talon right1, right2, right3, left1, left2, left3;
    DoubleSolenoid shifter;
    Encoder rightEncoder, leftEncoder;
    AHRS navx;
    
    PIDSource drivePS;
    PIDOutput drivePO;
    PIDController drivePC;
    
    PIDSource turnPS;
    PIDSource turnAbsolutePS;
    PIDOutput turnPO;
    PIDController turnPC;
    PIDController turnAbsolutePC;
    
    
    double lastPower, currPower = 0;
    
    public DrivetrainSubsystem(){
    	right1 = new Talon(RobotMap.rightMotor1);
    	right2 = new Talon(RobotMap.rightMotor2);
    	right3 = new Talon(RobotMap.rightMotor3);
    	left1 = new Talon(RobotMap.leftMotor1);
    	left2 = new Talon(RobotMap.leftMotor2);
    	left3 = new Talon(RobotMap.leftMotor3);
    	
    	shifter = new DoubleSolenoid(RobotMap.shiftLow, RobotMap.shiftHigh);
    	
    	rightEncoder = new Encoder(RobotMap.rightEncoderA, RobotMap.rightEncoderB);
    	leftEncoder = new Encoder(RobotMap.leftEncoderA, RobotMap.leftEncoderB);
    	rightEncoder.setDistancePerPulse(Constants.encoderCountsPerRevolution);
    	leftEncoder.setDistancePerPulse(Constants.encoderCountsPerRevolution);
    	
    	navx = new AHRS(Port.kMXP);
    	
    	drivePS = new PIDSource() {
			
			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				setPIDSourceType(PIDSourceType.kDisplacement);
			}
			
			@Override
			public double pidGet() {
				return getAverageEncoderDistance();
			}
			
			@Override
			public PIDSourceType getPIDSourceType() {
				return PIDSourceType.kDisplacement;
			}
		};
		
		drivePO = new PIDOutput() {
			
			@Override
			public void pidWrite(double output) {
				//TODO: Check if this is right
				execute(output, navx.getQuaternionX() * -.1);
			}
		};
		
		
		turnPS = new PIDSource() {
			
			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				setPIDSourceType(PIDSourceType.kDisplacement);
			}
			
			@Override
			public double pidGet() {
				return navx.getQuaternionX();
			}
			
			@Override
			public PIDSourceType getPIDSourceType() {
				return PIDSourceType.kDisplacement;
			}
		};
		
		turnAbsolutePS = new PIDSource() {
			
			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				setPIDSourceType(PIDSourceType.kDisplacement);
			}
			
			@Override
			public double pidGet() {
				return navx.getAngle();
			}
			
			@Override
			public PIDSourceType getPIDSourceType() {
				// TODO Auto-generated method stub
				return PIDSourceType.kDisplacement;
			}
		};
		
		turnPO = new PIDOutput() {
			
			@Override
			public void pidWrite(double output) {
				execute(0, output);
			}
		};
		
		drivePC = new PIDController(Constants.drivetrainDriveP, Constants.drivetrainDriveI, Constants.drivetrainDriveD, drivePS, drivePO);
		turnPC = new PIDController(Constants.drivetrainTurnP, Constants.drivetrainTurnI, Constants.drivetrainTurnD, turnPS, turnPO);
		turnAbsolutePC = new PIDController(Constants.drivetrainTurnP, Constants.drivetrainTurnI,Constants.drivetrainTurnD, turnAbsolutePS, turnPO);
		
		drivePC.setContinuous(false);
		drivePC.setOutputRange(-1, 1);
		drivePC.setAbsoluteTolerance(Constants.drivetrainDrivePIDTolerance);
		
		turnPC.setContinuous(true);
		turnPC.setInputRange(-180, 180);
		turnPC.setOutputRange(-1, 1);
		turnPC.setAbsoluteTolerance(Constants.drivetrainTurnPIDTolerance);
		
		turnAbsolutePC.setContinuous(true);
		turnAbsolutePC.setInputRange(-180, 180);
		turnAbsolutePC.setOutputRange(-1, 1);
		turnAbsolutePC.setAbsoluteTolerance(Constants.drivetrainTurnPIDTolerance);
		
    }
    
    private void setLeft(double speed){
    	left1.set(speed);
    	left2.set(speed);
    	left3.set(speed);
    }
    
    private void setRight(double speed){
    	right1.set(speed);
    	right2.set(speed);
    	right3.set(speed);
    }
    
    public void execute(double power, double turn){
    	
    	
    	//lastTurn = turn;
		lastPower = currPower;
		currPower = power;
		//turn = pTurn;
    	
    	if(Math.abs(currPower - lastPower) > Constants.maxPowerDiffential){
    		if(currPower > lastPower){
        		currPower = lastPower + Constants.maxPowerDiffential;
    		}else{
        		currPower = lastPower - Constants.maxPowerDiffential;
    		}
    	}
    	
    	arcadeDrive(currPower, turn);
    }
    
    public void arcadeDrive(double power, double turn){
    	setLeft(power - turn);
    	setRight(power + turn);
    }
    
    public void shiftHigh(){
    	shifter.set(Value.kForward);
    	Robot.states.drivetrainGearTracker = DrivetrainGear.HIGH;
    }
    
    public void shiftLow(){
    	shifter.set(Value.kReverse);
    	Robot.states.drivetrainGearTracker = DrivetrainGear.LOW;
    }
    
    public double getRightEncoderDistance(){
    	return rightEncoder.getDistance();
    }
    
    public double getLeftEncoderDistance(){
    	return leftEncoder.getDistance();
    }
    
    public double getAverageEncoderDistance(){
    	return (rightEncoder.getDistance() + leftEncoder.getDistance())/2;
    }
    
    //NAVX
    
    public double getTrueAngle(){
    	return navx.getAngle();
    }
    
    public double getTilt(){
    	return Math.sqrt(Math.pow(navx.getPitch(), 2) + Math.pow(navx.getRoll(), 2));
    }
    
    public void drivetrainDrivePIDEnable(){
    	if(turnPC.isEnabled()){
    		turnPC.disable();
    	}
    	
    	if(turnAbsolutePC.isEnabled()){
    		turnAbsolutePC.disable();
    	}
    	
    	drivePC.enable();
    }
    
    public void drivetrainDrivePIDDisable(){
    	drivePC.disable();
    }
    
    public void drivetrainDrivePIDSetSetpoint(double setpoint){
    	drivePC.setSetpoint(setpoint);
    }
    
    public boolean drivetrainDrivePIDisOnTarget(){
    	return drivePC.onTarget();
    }
    
    public void drivetrainTurnPIDEnable(){
    	if(drivePC.isEnabled()){
    		drivePC.disable();
    	}
    	
    	if(turnAbsolutePC.isEnabled()){
    		turnAbsolutePC.disable();
    	}
    	navx.zeroYaw();
    	turnPC.enable();
    }
    
    public void drivetrainTurnPIDDisable(){
    	turnPC.disable();
    }
    
    public void drivetrainTurnPIDSetSetpoint(double setpoint){
    	turnPC.setSetpoint(setpoint);
    }
    
    public boolean drivetrainTurnPIDisOnTarget(){
    	return turnPC.onTarget();
    }
    
    public void drivetrainTurnAbsolutePIDEnable(){
    	if(drivePC.isEnabled()){
    		drivePC.disable();
    	}
    	
    	if(turnPC.isEnabled()){
    		turnPC.disable();
    	}
    	turnAbsolutePC.enable();
    }
    
    public void drivetrainTurnAbsolutePIDDisable(){
    	turnAbsolutePC.disable();
    }
    
    public void drivetrainTurnAbsolutePIDSetSetpoint(double setpoint){
    	turnAbsolutePC.setSetpoint(setpoint);
    }
    
    public boolean drivetrainTurnAbsolutePIDisOnTarget(){
    	return turnAbsolutePC.onTarget();
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

