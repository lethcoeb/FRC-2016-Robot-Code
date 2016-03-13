package org.usfirst.frc.team1806.robot.commands.autonomous;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.Mode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DoNothing extends Command {

    public DoNothing() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drivetrainSS);
    	requires(Robot.elevatorSS);
    	requires(Robot.intakeSS);
    	requires(Robot.shooterSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrainSS.arcadeDrive(0, 0);
    	Robot.elevatorSS.elevatorDisable();
    	//Robot.intakeSS.retractIntake();
    	Robot.shooterSS.pinchBall();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drivetrainSS.arcadeDrive(0, 0);
    	//Robot.elevatorSS.elevatorDisable();
    	Robot.intakeSS.runAtSpeed(0);
    	Robot.shooterSS.stopCocking();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
        return Robot.states.mode == Mode.TELEOP;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevatorSS.elevatorEnable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
