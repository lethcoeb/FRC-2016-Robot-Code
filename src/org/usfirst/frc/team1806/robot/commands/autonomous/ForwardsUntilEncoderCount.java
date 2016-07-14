package org.usfirst.frc.team1806.robot.commands.autonomous;

import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ForwardsUntilEncoderCount extends Command {
	
	boolean invalid = false;
	
	double m_distance;
	double m_speed;
	
    public ForwardsUntilEncoderCount(double distance, double speed) {
    	m_distance = Math.abs(distance);
    	m_speed = speed;
    	
    	if(Math.signum(distance) != Math.signum(speed)){
    		invalid = true;
    	}
    	
    	Robot.drivetrainSS.drivetrainControlLoopsDisable();
    	Robot.drivetrainSS.resetYaw();
        requires(Robot.drivetrainSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(!invalid){
    		Robot.drivetrainSS.arcadeDrive(m_speed, Robot.drivetrainSS.getYaw() * .05);
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Math.abs(Robot.drivetrainSS.getRightEncoderDistance()) >= m_distance) || (Math.abs(Robot.drivetrainSS.getLeftEncoderDistance()) >= m_distance) || invalid;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrainSS.arcadeDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
