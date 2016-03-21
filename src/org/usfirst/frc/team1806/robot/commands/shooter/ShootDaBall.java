package org.usfirst.frc.team1806.robot.commands.shooter;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterCocked;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ShootDaBall extends Command {
	
	Timer timer;
	final double kTimeToSettle = Constants.timeToSettle;
	
    public ShootDaBall() {
        requires(Robot.shooterSS);
        timer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	timer.reset();
    	timer.start();
    	Robot.shooterSS.releaseBall();
    	//ezpz
    	Robot.shooterSS.cockShooterReleaseDogGear();
    	Robot.states.shooterCockedTracker = ShooterCocked.NOTCOCKED;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return timer.get() >= kTimeToSettle;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shooterSS.cockingDogGearDisengage();
    	Robot.states.hasBall = false;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
