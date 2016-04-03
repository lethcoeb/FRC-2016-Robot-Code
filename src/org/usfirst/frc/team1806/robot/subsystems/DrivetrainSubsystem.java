package org.usfirst.frc.team1806.robot.subsystems;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.RobotStates.DrivetrainGear;
import org.usfirst.frc.team1806.robot.commands.DriverControlDrivetrain;
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
    
    final double kJoystickDeadzoneConstant = Constants.joystickDeadzone;
    double lastPower, currPower, lastTurnPower, currTurnPower = 0;
    double PIDTolerance = 30;
    double MaxRotationPID = Constants.drivetrainMaxRotationPIDStage1;
    double maxSpeed = 1;
    Boolean autoShift;
    Boolean lowGearLock;
	Boolean liningUpShot;

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
    	//rightEncoder.setReverseDirection(true);
    	//leftEncoder.setReverseDirection(true);
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
				//FIXME use two encoders
				return getRightEncoderDistance();
			}
			
			@Override
			public PIDSourceType getPIDSourceType() {
				return PIDSourceType.kDisplacement;
			}
		};
		
		drivePO = new PIDOutput() {
			
			@Override
			public void pidWrite(double output) {
				
				if(Math.abs(output) > maxSpeed){
					output = maxSpeed * Math.signum(output);
				}
				execute(output, getYaw() * .05);
			}
		};
		
		
		turnPS = new PIDSource() {
			
			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				setPIDSourceType(PIDSourceType.kDisplacement);
			}
			
			@Override
			public double pidGet() {
				return navx.getYaw();
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
				/*pass the opposite of the value from the PID to the turn value of the drivetrain.
				If the value isn't zero, we need to re-scale it to be between the minimum power to cause movement and max power
				Otherwise, just pass the 0 value to the drivetrain.
				*/
				/*if(output != 0){
					output = output * (1-Constants.drivetrainTurnMinPowerToMove) + Constants.drivetrainTurnMinPowerToMove * Math.signum(output);
				}*/
				execute(0, -output);
				System.out.println(-output);
			}
		};
		
		drivePC = new PIDController(Constants.drivetrainDriveP, Constants.drivetrainDriveI, Constants.drivetrainDriveD, drivePS, drivePO);
		turnPC = new PIDController(Constants.drivetrainTurn3P, Constants.drivetrainTurn3I, Constants.drivetrainTurn3D, turnPS, turnPO);
		turnAbsolutePC = new PIDController(Constants.drivetrainTurn3P, Constants.drivetrainTurn3I,Constants.drivetrainTurn3D, turnAbsolutePS, turnPO);
		
		
		drivePC.setContinuous(false);
		drivePC.setOutputRange(-1, 1);
		drivePC.setAbsoluteTolerance(Constants.drivetrainDrivePIDTolerance);
		
		turnPC.setContinuous(true);
		turnPC.setInputRange(-180, 180);
		turnPC.setOutputRange(-1, 1);
		turnPC.setAbsoluteTolerance(Constants.drivetrainTurnPID1Tolerance);
		
		turnAbsolutePC.setContinuous(true);
		turnAbsolutePC.setInputRange(-180, 180);
		turnAbsolutePC.setOutputRange(-1, 1);
		turnAbsolutePC.setAbsoluteTolerance(Constants.drivetrainTurnPID1Tolerance);
		
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
    
    public double zoneInput(double val){
    	
    	if(Math.abs(val) < kJoystickDeadzoneConstant){
    		val = 0;
    	}
    	
    	return val;
    }
    
    public void execute(double power, double turn){
    	
		lastPower = currPower;
		currPower = power;
		
		lastTurnPower = currTurnPower;
		currTurnPower = turn;
    	
    	if(Math.abs(currPower - lastPower) > Constants.maxPowerDiffential){
    		if(currPower > lastPower){
        		currPower = lastPower + Constants.maxPowerDiffential;
    		}else{
        		currPower = lastPower - Constants.maxPowerDiffential;
    		}
    	}
    	
    	if(Math.abs(currTurnPower - lastTurnPower) > Constants.maxTurnPowerDifferential){
    		if(currTurnPower > lastTurnPower){
        		currTurnPower = lastTurnPower + Constants.maxTurnPowerDifferential;
    		}else{
        		currTurnPower = lastTurnPower - Constants.maxTurnPowerDifferential;
    		}
    	}
		if (autoShift) {
			shiftAutomatically();
		} else {
			shiftLow();
		}
    	arcadeDrive(currPower, currTurnPower);

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
    	return -rightEncoder.getDistance();
    }
    
    public double getLeftEncoderDistance(){
    	return leftEncoder.getDistance();
    }
    
    public double getAverageEncoderDistance(){
    	return (rightEncoder.getDistance() + leftEncoder.getDistance())/2;
    }
    
    public void drivetrainDrivePIDSetMaxSpeed(double speed){
    	maxSpeed = speed;
    }
    
    public void drivetrainDrivePIDResetMaxSpeed(){
    	maxSpeed = 1;
    }
    
    public void resetEncoders(){
    	leftEncoder.reset();
    	rightEncoder.reset();
    }
    //NAVX
    
    public void resetYaw(){
    	navx.zeroYaw();
    }
    
    public void resetNavx(){
    	navx.reset();
    }
    
    public boolean isNavxConnected(){
    	return navx.isConnected();
    }
    
    public boolean isNavxFlat(){
    	return navx.getRoll() < Constants.navxMinPitchToBeFlat;
    }
    
    public double getTrueAngle(){
    	return navx.getAngle();
    }
    
    public double getYaw(){
    	return navx.getYaw();
    }
    
    public double getPitch(){
    	return navx.getPitch();
    }
    
    public double getRoll(){
    	return navx.getRoll();
    }
    
    public double getRotationalSpeed(){
    	return navx.getRate();
    }
    
    public double getQuaternion(){
    	return navx.getQuaternionZ() * 180;
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
    	drivePC.reset();
    }
    
    public void drivetrainDrivePIDSetSetpoint(double setpoint){
    	drivePC.setSetpoint(setpoint);
    }
    
    public boolean drivetrainDrivePIDisOnTarget(){
    	return Math.abs(drivePC.getError()) < Constants.drivetrainDrivePIDTolerance;
    }
    
    public void drivetrainTurnPIDEnable(){
    	if(drivePC.isEnabled()){
    		drivePC.disable();
    	}
    	
    	if(turnAbsolutePC.isEnabled()){
    		turnAbsolutePC.disable();
    	}
    	turnPC.enable();
    }
    
    public void drivetrainTurnPIDDisable(){
    	turnPC.disable();
    }
    
    public void drivetrainTurnPIDSetSetpoint(double setpoint){
    	turnPC.setSetpoint(setpoint);
    }
    
    public boolean drivetrainTurnPIDisOnTarget(){
    	return Math.abs(turnPC.getError()) < PIDTolerance && Math.abs(getRotationalSpeed()) < MaxRotationPID;
    }
    
    public double getTurnPCError(){
    	return turnPC.getError();
    }
    
    public void drivetrainTurnPIDchangePID(double p, double i, double d){
    	turnPC.setPID(p, i, d);
    }
    
    public void drivetrainTurnPIDchangeMaxRotation(double maxRot){
    	MaxRotationPID = maxRot;
    }
    
    public void drivetrainTurnPIDReset(){
    	turnPC.reset();
    }
    
    public void drivetrainTurnPIDSetTolerance(double tolerance){
    	PIDTolerance = tolerance;
    }
    
    public void drivetrainTurnAbsolutePIDEnable(){
    	if(drivePC.isEnabled()){
    		drivePC.disable();
    	}
    	
    	if(turnPC.isEnabled()){
    		turnPC.disable();
    	}
    	turnAbsolutePC.reset();
    	turnAbsolutePC.enable();
    }
    
    public void drivetrainTurnAbsolutePIDDisable(){
    	turnAbsolutePC.disable();
    	turnAbsolutePC.reset();
    }
    
    public void drivetrainTurnAbsolutePIDSetSetpoint(double setpoint){
    	turnAbsolutePC.setSetpoint(setpoint);
    }
    
    public boolean drivetrainTurnAbsolutePIDisOnTarget(){
    	return Math.abs(turnAbsolutePC.getError()) < PIDTolerance && Math.abs(getRotationalSpeed()) < MaxRotationPID;
    }
    
    public void drivetrainTurnAbsolutePIDchangePID(double p, double i, double d){
    	turnAbsolutePC.setPID(p, i, d);
    }
    
    public void drivetrainTurnAbsolutePIDchangeMaxRotation(double maxRot){
    	MaxRotationPID = maxRot;
    }
    
    public void drivetrainTurnAbsolutePIDReset(){
    	turnAbsolutePC.reset();
    }
    
    public void drivetrainTurnAbsolutePIDSetTolerance(double tolerance){
    	PIDTolerance = tolerance;
    }
    //////////////////////////////////////////////
	public boolean isInLowGear() {
		return shifter.get() == DoubleSolenoid.Value.kReverse;
	}

	public boolean isInHighGear() {
		return shifter.get() == DoubleSolenoid.Value.kForward;
	}

	public void lowGearLockDisable() {
		lowGearLock = false;
	}
	public double getDriveSpeedFPS() {
		return Math.abs(getDriveVelocityFPS());
	}
	public double getDriveVelocityFPS() {
		// TODO: This should convert to fps but fix it if it doesn't
		
		//FOR NAVX WHICH SUCKS
		//return getDriveVelocity() * 3.28083989501;
		
		 return getDriveVelocity() / 12;
	}
	public double getDriveVelocity() {
		// returns the average speed of the left and right side in inches per
		// second
		
		
		// getVelocity is in m/s
		
		//return SWATLib.convertTo2DVector(navX.getVelocityX(), navX.getVelocityY());
		//return navX.getVelocityX();
		
		// Old version, using encoders:
		return ((leftEncoder.getRate() + rightEncoder.getRate()) / 2);
	}
	public boolean isSpeedingUp() {
		return getDriveAccelFPSPS() > Constants.drivetrainAccelerationThreshold;
	}
	public double getDriveAccelFPSPS() {		
		return navx.getRawAccelY();
		// Old version:
		//return (currentSpeed - lastSpeed) / period;
	}
	public boolean isSlowingDown() {
		return getDriveAccelFPSPS() < -Constants.drivetrainAccelerationThreshold;
	}
	public boolean isAutoShifting(){
		return autoShift;
	}
	public void disableAutoShift() {
		autoShift = false;
	}
	public void enableAutoShift() {
		autoShift = true;
	}
	public void setLiningUp(){
		liningUpShot = true;
	}
	
	public void disableLiningUp(){
		liningUpShot = false;
	}
	public boolean isLiningUp(){
		return liningUpShot;
	}
	private void shiftAutomatically() {
		// shifts if neccessary, returns whether shifting was done
		if (getDriveSpeedFPS() > Constants.drivetrainUpshiftSpeedThreshold
				&& Math.abs(currPower) > Constants.drivetrainUpshiftPowerThreshold && isSpeedingUp() && isInLowGear()
				&& !lowGearLock) {
			// Normal Upshift
			// if fast enough to need to upshift, driver is applying sufficient
			// throttle, the robot is speeding up and it's in low gear, upshift
			shiftHigh();
		} else if (getDriveSpeedFPS() > Constants.drivetrainMaxLowGearSpeed && isInLowGear()
				&& !lowGearLock) {
			// the rev limiter was hit because driver wasn't hitting the
			// throttle hard enough to change gear
			shiftHigh();
		} else if (getDriveSpeedFPS() < Constants.drivetrainMaxLowGearSpeed
				&& Math.abs(currPower) > Constants.drivetrainPowerDownshiftPowerThreshold && isSlowingDown()
				&& isInHighGear()) {
			// if the robot is slowing down while the driver is applying
			// sufficient power, and is at a reasonable speed to be in low gear,
			// downshift.
			// Think of a pushing match that started at high speed
			shiftLow();
		} else if (getDriveSpeedFPS() < Constants.drivetrainDownshiftSpeedThreshold
				&& Math.abs(currPower) > Constants.drivetrainPowerDownshiftPowerThreshold && isInHighGear()) {
				shiftLow();
		} else if (getDriveSpeedFPS() < Constants.drivetrainDownshiftSpeedThreshold
				&& Math.abs(currPower) < Constants.drivetrainDownshiftPowerThreshold && isInHighGear()) {
			// if the robot is slowing down, not being given considerable
			// throttle
			// a coasting/stopping downshift
			shiftLow();
		}
	}
    public void initDefaultCommand() {
    	
    	setDefaultCommand(new DriverControlDrivetrain());
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

