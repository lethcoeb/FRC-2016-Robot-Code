package org.usfirst.frc.team1806.robot.subsystems;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
    
    double kGearEngageSpeed = Constants.gearEngageSpeed;
    
    /*
     * @param meme a string
     */
    public ShooterSubsystem(){
    	//cockingMotor goes one direction. Only valid values are positive
    	cockingMotor = new Talon(RobotMap.cockingMotor);
    	cockingMotor.setInverted(true);
    	cockedLimitSwitch = new DigitalInput(RobotMap.cockedShooterLimit);
    	puncherReleaseSolenoid = new Solenoid(RobotMap.shooterReleaseSolenoid);
    	ballPincherSolenoid = new DoubleSolenoid(1, RobotMap.pincherPinch, RobotMap.pincherRelease);
    	ballSensor = new DigitalInput(RobotMap.hasBallSensor);
    }
    
    public void cockShooterEngageGear(){
    	//go at slow speed so the gear can catch
    	//kgearengagespeed
    	cockingMotor.set(.25);
    	//cockingMotor.set(.1);

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
    
    public boolean shooterIsCocked(){
    	return !cockedLimitSwitch.get();
    }
    
    public boolean hasBallSensor(){
    	return !ballSensor.get();
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

