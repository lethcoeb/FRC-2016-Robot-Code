package org.usfirst.frc.team1806.robot.commands;

import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class StopRobotMovement extends Command {

	boolean forwards;
	Timer t;
	
    public StopRobotMovement() {
        requires(Robot.drivetrainSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	t = new Timer();
    	t.start();
    	if(Robot.drivetrainSS.getDriveVelocityFPS() >= 0){
    		Robot.drivetrainSS.arcadeDrive(-.3, 0);
    	}else{
    		Robot.drivetrainSS.arcadeDrive(.3, 0);
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return t.get() > .35;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
