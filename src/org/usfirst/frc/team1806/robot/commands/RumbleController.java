package org.usfirst.frc.team1806.robot.commands;

import edu.wpi.first.wpilibj.Joystick.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import util.XboxController;

/**
 *
 */
public class RumbleController extends Command {

	XboxController xbc;
	Timer t;
	
    public RumbleController(XboxController cont) {
        xbc = cont;
        t = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	t.start();
    	xbc.setRumble(RumbleType.kLeftRumble, (float) .3);
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return t.get() > .1;
    }

    // Called once after isFinished returns true
    protected void end() {
    	xbc.setRumble(RumbleType.kLeftRumble, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	xbc.setRumble(RumbleType.kLeftRumble, 0);
    }
}
