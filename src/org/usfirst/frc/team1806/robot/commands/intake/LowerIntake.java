package org.usfirst.frc.team1806.robot.commands.intake;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.IntakePosition;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LowerIntake extends Command {
	
	double kTimeToLower = Constants.intakeTimeToLower;
	
	Timer timer;
	boolean runWithTimeout;
	double mTimeout;
	
    public LowerIntake() {
        requires(Robot.intakeSS);
        timer = new Timer();
        runWithTimeout = false;
    }
    
    public LowerIntake(double timeout){
    	timer = new Timer();
    	runWithTimeout = true;
    	mTimeout = timeout;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.states.intakePositionTracker = IntakePosition.MOVING;
    	Robot.intakeSS.deployIntake();
    	timer.reset();
    	timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(!runWithTimeout){
    		return timer.get() >= kTimeToLower;
    	}else{
    		return timer.get() > mTimeout;
    	}
        
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.states.intakePositionTracker = IntakePosition.DEPLOYED;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.states.intakePositionTracker = IntakePosition.DEPLOYED;
    }
}
