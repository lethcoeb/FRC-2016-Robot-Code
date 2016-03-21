package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetStateHolding extends Command {

    public SetStateHolding() {
        requires(Robot.elevatorSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.states.shooterArmPositionTracker = ShooterArmPosition.HOLDING;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
