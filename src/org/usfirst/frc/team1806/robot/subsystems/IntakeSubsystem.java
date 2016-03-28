package org.usfirst.frc.team1806.robot.subsystems;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.RobotStates.IntakeRollerState;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class IntakeSubsystem extends Subsystem {

	Talon roller;
	DoubleSolenoid deployer;

	double kIntakeRollerSpeed = Constants.intakeRollerSpeed;
	double kOuttakeRollerSpeed = Constants.outtakeRollerSpeed;

	public IntakeSubsystem() {
		roller = new Talon(RobotMap.intakeRoller);
		roller.setInverted(true);
		deployer = new DoubleSolenoid(RobotMap.deployerExtend, RobotMap.deployerRetract);
	}

	public void intakeBall() {
		roller.set(kIntakeRollerSpeed);
		Robot.states.intakeRollerStateTracker = IntakeRollerState.INTAKING;
	}

	public void outtakeBall() {
		roller.set(kOuttakeRollerSpeed);
		Robot.states.intakeRollerStateTracker = IntakeRollerState.OUTTAKING;
	}
	
	public void runAtSpeed(double speed){
		roller.set(speed);
		if(speed > 0){
			Robot.states.intakeRollerStateTracker = IntakeRollerState.INTAKING;
		}else if(speed < 0){
			Robot.states.intakeRollerStateTracker = IntakeRollerState.OUTTAKING;
		}else{
			Robot.states.intakeRollerStateTracker = IntakeRollerState.STOPPED;
		}
	}

	public void stopIntaking() {
		roller.set(0);
		Robot.states.intakeRollerStateTracker = IntakeRollerState.STOPPED;
	}

	public void deployIntake() {

		deployer.set(Value.kForward);

	}

	public void retractIntake() {

		deployer.set(Value.kReverse);

	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
