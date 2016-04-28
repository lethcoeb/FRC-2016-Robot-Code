package org.usfirst.frc.team1806.robot.subsystems;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ShooterSubsystem extends Subsystem {
    
    Talon cockingMotor;
    DigitalInput cockedLimitSwitch;
    Solenoid puncherReleaseSolenoid;
    DoubleSolenoid ballPincherSolenoid;
    DigitalInput ballSensor;
    Relay flashlight;
    
    double kGearEngageSpeed = Constants.gearEngageSpeed;
    
    public ShooterSubsystem(){
    	//cockingMotor goes one direction. Only valid values are positive
    	/*
    	 *  So DO NOT send it in a negative direction!!
    	 */
    	cockingMotor = new Talon(RobotMap.cockingMotor);
    	cockingMotor.setInverted(true);
    	cockedLimitSwitch = new DigitalInput(RobotMap.cockedShooterLimit);
    	puncherReleaseSolenoid = new Solenoid(RobotMap.shooterReleaseSolenoid);
    	ballPincherSolenoid = new DoubleSolenoid(RobotMap.pincherPinch, RobotMap.pincherRelease);
    	//ballPincherSolenoid = new DoubleSolenoid(1, RobotMap.pincherPinch, RobotMap.pincherRelease);
    	ballSensor = new DigitalInput(RobotMap.hasBallSensor);
    	flashlight = new Relay(0);
    	flashlight.setDirection(Relay.Direction.kForward);
    }
    
    public void cockShooterEngageGear(){
    	//go at slow speed so the gear can catch
    	cockingMotor.set(.25);
    }
    
    public void cockShooterFullSpeed(){
    	cockingMotor.set(.6);
    }
    
    public void cockShooterReleaseDogGear(){
    	cockingMotor.set(0);
    }
    
    public void stopCocking(){
    	cockingMotor.set(0);
    }
    
    public void cockingDogGearDisengage(){
    	puncherReleaseSolenoid.set(true);
    }
    
    public void cockingDogGearEngage(){
    	puncherReleaseSolenoid.set(false);
    }
    
    public void pinchBall(){
    	ballPincherSolenoid.set(Value.kForward);
    }
    
    public void releaseBall(){
    	ballPincherSolenoid.set(Value.kReverse);
    }
    
    public boolean isShooterCocked(){
    	return !cockedLimitSwitch.get();
    }
    
    public boolean isCocking(){
    	return cockingMotor.get() != 0;
    }
    
    public boolean hasBallSensor(){
    	return !ballSensor.get();
    }
    
    public void getLit(){
    	flashlight.set(Relay.Value.kOn);
    }
    
    public void unLit(){
    	flashlight.set(Relay.Value.kOff);
    }

    public void initDefaultCommand() {

    }
}

