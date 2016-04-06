package org.usfirst.frc.team1806.robot.commands;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.DriveControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriverControlDrivetrain extends Command {
	
	double kJoystickDeadzone = .15;
	Timer timer;
	double dlsY;
	double drsX;
	double a;
    public DriverControlDrivetrain() {
        requires(Robot.drivetrainSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Timer timer = new Timer();
    	dlsY = Robot.oi.dc.getLeftJoyY();
    	drsX = Robot.oi.dc.getRightJoyX();
    	a = Robot.oi.dc.getLeftTrigger();
    	timer.reset();
    	if (Robot.states.driveControlModeTracker == DriveControlMode.DRIVER) {
			if (Math.abs(dlsY) > kJoystickDeadzone && Math.abs(drsX) > kJoystickDeadzone) {
				Robot.drivetrainSS.execute(dlsY, -drsX);
			} else if (Math.abs(dlsY) > kJoystickDeadzone) {
				Robot.drivetrainSS.execute(dlsY, 0);
			} else if (Math.abs(drsX) > kJoystickDeadzone) {
				Robot.drivetrainSS.execute(0, -drsX);
			} else {
				Robot.drivetrainSS.execute(0, 0);
			}
		} else {
			// Automatic driving stuff
		}
    
    	if(Robot.oi.dc.getLeftTrigger() >= .6 && !Robot.drivetrainSS.isAutoShifting()){
    		Robot.drivetrainSS.enableAutoShift();
    	} else if(Robot.oi.dc.getLeftTrigger() < .6 && Robot.drivetrainSS.isAutoShifting()) {
    			Robot.drivetrainSS.disableAutoShift();
    	}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
