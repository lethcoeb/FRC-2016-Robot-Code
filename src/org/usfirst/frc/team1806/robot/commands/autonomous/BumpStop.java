package org.usfirst.frc.team1806.robot.commands.autonomous;

import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class BumpStop extends Command {
	
	boolean forwards;
	Timer t;
	
    public BumpStop(boolean forwards) {
        requires(Robot.drivetrainSS);
        this.forwards = forwards;
        t = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	t.reset();
    	t.start();
    	if(forwards){
    		Robot.drivetrainSS.execute(-1, 0);
    	}else{
    		Robot.drivetrainSS.execute(1, 0);
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(forwards){
    		Robot.drivetrainSS.execute(-1, 0);
    	}else{
    		Robot.drivetrainSS.execute(1, 0);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return t.get() >= .1;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrainSS.arcadeDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drivetrainSS.arcadeDrive(0, 0);
    }
}
