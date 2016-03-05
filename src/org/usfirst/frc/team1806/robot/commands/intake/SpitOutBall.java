package org.usfirst.frc.team1806.robot.commands.intake;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHolding;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SpitOutBall extends Command {

    public SpitOutBall() {
    	//HELLA REQUIREMENTS
        requires(Robot.elevatorSS);
        requires(Robot.intakeSS);
        requires(Robot.shooterSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevatorSS.resetSrxPID();
    	Robot.elevatorSS.elevatorSetSetpoint(0);
    	Robot.intakeSS.outtakeBall();
    	Robot.shooterSS.releaseBall();
    	Robot.states.hasBall = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.oi.dc.getRightTrigger() < .4;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.intakeSS.stopIntaking();
    	new MoveToHolding().start();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.intakeSS.stopIntaking();
    }
}
