package org.usfirst.frc.team1806.robot.commands.intake;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.IntakePosition;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RaiseIntake extends Command {

double kTimeToRaise = Constants.timeToRaise;
	
	Timer timer;
	
    public RaiseIntake() {
        requires(Robot.intakeSS);
        timer = new Timer();
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
        return timer.get() >= kTimeToRaise;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.states.intakePositionTracker = IntakePosition.RETRACTED;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.states.intakePositionTracker = IntakePosition.RETRACTED;
    }
}
