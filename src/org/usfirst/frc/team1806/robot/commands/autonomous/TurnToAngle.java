package org.usfirst.frc.team1806.robot.commands.autonomous;

import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**Turns the robot to a universal angle on the field. It is relative to the angle at which the robot initializes itself at (0).
 *@author Becker Lethcoe
 */
public class TurnToAngle extends Command {

	double kTargetAngle;
	
    public TurnToAngle(double targetAngle) {
        requires(Robot.drivetrainSS);
        kTargetAngle = targetAngle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrainSS.drivetrainTurnPIDEnable();
    	Robot.drivetrainSS.drivetrainTurnPIDSetSetpoint(kTargetAngle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.drivetrainSS.drivetrainTurnPIDisOnTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrainSS.drivetrainTurnPIDDisable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drivetrainSS.drivetrainTurnPIDDisable();
    }
}
