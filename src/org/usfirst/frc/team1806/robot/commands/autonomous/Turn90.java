package org.usfirst.frc.team1806.robot.commands.autonomous;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.DriveControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Turn90 extends Command {

	double m_timeout;
	boolean m_clockwise;
	
	boolean pidEngaged = false;
	
	Timer t;

	public Turn90(double timeout, boolean clockwise) {
		requires(Robot.drivetrainSS);
		// Pass in 0 for no timeout
		m_timeout = timeout;
		m_clockwise = clockwise;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		
		Robot.states.driveControlModeTracker = DriveControlMode.AUTO;
		
		t = new Timer();
		t.reset();
		t.start();

		Robot.drivetrainSS.resetYaw();
		Robot.drivetrainSS.drivetrainTurnPIDReset();
		Robot.drivetrainSS.drivetrainTurnPIDchangeMaxRotation(1);
		Robot.drivetrainSS.drivetrainTurnPIDchangePID(.01, 0, 0);

		if (m_clockwise) {
			Robot.drivetrainSS.drivetrainTurnPIDSetSetpoint(98);
		}else{
			//counter clockwise
			Robot.drivetrainSS.drivetrainTurnPIDSetSetpoint(-98);
		}
		
		Robot.drivetrainSS.drivetrainTurnPIDSetTolerance(2);
		Robot.drivetrainSS.drivetrainTurnPIDEnable();
		

		

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		
		if(!pidEngaged){
			if(Math.abs(Robot.drivetrainSS.getTurnPCError()) <= 60){
				//Time to engage the PID!:!:!:!:!: :)
				
				pidEngaged = true;
				Robot.drivetrainSS.drivetrainTurnPIDReset();
				Robot.drivetrainSS.drivetrainTurnPIDchangeMaxRotation(1);
				Robot.drivetrainSS.drivetrainTurnPIDchangePID(0, 0, 0.02);				
				Robot.drivetrainSS.drivetrainTurnPIDSetTolerance(2);
				Robot.drivetrainSS.drivetrainTurnPIDEnable();
				
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if(m_timeout == 0){
			return Robot.drivetrainSS.drivetrainTurnPIDisOnTarget();
		}else{
			return Robot.drivetrainSS.drivetrainTurnPIDisOnTarget() || t.get() > m_timeout;
		}
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.drivetrainSS.drivetrainTurnPIDReset();
		System.out.println("turn90 ended");
		Robot.drivetrainSS.arcadeDrive(0, 0);
		Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
		
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.drivetrainSS.drivetrainTurnPIDReset();
		Robot.drivetrainSS.arcadeDrive(0, 0);
		Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
		
	}
}
