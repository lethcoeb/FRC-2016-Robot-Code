package org.usfirst.frc.team1806.robot.commands;

import edu.wpi.first.wpilibj.Joystick.RumbleType;

import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import util.XboxController;

/**
 *
 */
public class RumbleController extends Command {


	Timer t;
	XboxController xbc;
	
    public RumbleController(XboxController cont) {
        t = new Timer();
        xbc = cont;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	t.start();
    	xbc.setRumble(RumbleType.kLeftRumble, 1);

    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return t.get() > .3;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.oi.stopRumbles();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.oi.stopRumbles();
    }
}
