package org.usfirst.frc.team1806.robot.commands.autonomous;

import org.usfirst.frc.team1806.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToVisionAngle extends Command {

	double kTargetAngle;
	boolean found = false;

	public TurnToVisionAngle() {
		requires(Robot.drivetrainSS);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.drivetrainSS.drivetrainTurnPIDEnable();
		if (Robot.jr.isGoalFound()) {
			found = true;
			Robot.drivetrainSS.drivetrainTurnPIDSetSetpoint(Robot.jr.getAngleToGoal());
		}else{
			//wait
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if(!found && Robot.jr.isGoalFound()){
			found = true;
			Robot.drivetrainSS.drivetrainTurnPIDSetSetpoint(Robot.jr.getAngleToGoal());
		}
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
