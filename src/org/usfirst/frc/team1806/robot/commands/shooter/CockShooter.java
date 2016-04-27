package org.usfirst.frc.team1806.robot.commands.shooter;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotMap;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterCocked;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CockShooter extends Command {
	
	Timer t;
	boolean cocking = false;
	
	//boolean finished = false;
	boolean didNotStart = false;
	
	double kMinTimeToCock = Constants.minTimeToCock;
	
    public CockShooter() {
        requires(Robot.shooterSS);
        this.setInterruptible(false);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	t = new Timer();
    	t.reset();
    	t.start();
    	if(!Robot.shooterSS.isShooterCocked()){
        	Robot.shooterSS.cockingDogGearEngage();
        	Robot.shooterSS.cockShooterEngageGear();
    	}
    	else{
    		didNotStart = true;
    	}
    	
    	/*
    	//Becker's original code
    	t = new Timer();
    	t.reset();
    	t.start();
    	
    	
    	//engage gear, start pulling
    	Robot.shooterSS.cockingDogGearEngage();
    	Robot.shooterSS.cockShooterEngageGear();
    	
    	if(Robot.shooterSS.shooterIsCocked()){
    		finished = true;
    	*/
    	
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	
    	//Becker's original code
    	if(!cocking && t.get() > Constants.timeToEngageDogGear){
    		//you've waited for long enough, the gear is engaged fam, COCK THAT SHIT BRUH
    		cocking = true;
    		Robot.shooterSS.cockShooterFullSpeed();
    	}
    	/*
    	if(Robot.pdp.getCurrent(RobotMap.PDPcockingWinchSlot) > 100){
    		//kill pls
    		finished = true;
    	}
    	
    	if(Robot.shooterSS.shooterIsCocked() && t.get() > kMinTimeToCock){
    		finished = true;
    	}
    	*/
    	
    	if(Robot.pdp.getCurrent(RobotMap.PDPcockingWinchSlot) > 100){
    		Robot.states.overCocked = true;
    	}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Robot.pdp.getCurrent(RobotMap.PDPcockingWinchSlot) > 100 || didNotStart){
    		return true;
    	}
    	else if(t.get() <= kMinTimeToCock) {
    		return false;
    	}
    	else{
    		return Robot.shooterSS.isShooterCocked();
    	}
    	/*
    	//Becker's original code
        return finished;
        */
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shooterSS.stopCocking();
    	Robot.states.autoalignmentShooting = false;
    	Robot.states.shooterCockedTracker = ShooterCocked.COCKED;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	//if this ever gets interrupted I'll literally kill my self
    	Robot.shooterSS.stopCocking();
    }
}
