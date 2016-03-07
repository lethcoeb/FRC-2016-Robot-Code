package org.usfirst.frc.team1806.robot.commands.autotarget;

import org.usfirst.frc.team1806.robot.OperatorInterface;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.DriveControlMode;
import org.usfirst.frc.team1806.robot.commands.RumbleController;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LineUpShot extends Command {

	boolean goalFound, withinRange, hasRumbled;
	
    public LineUpShot() {
        requires(Robot.drivetrainSS);
    	Robot.states.autoLiningUp = true;

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	//Robot.drivetrainSS.resetYaw();
    	goalFound = false;
    	withinRange = false;
    	hasRumbled = false;
    	
    	System.out.println("auto line up started");
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	//System.out.println(Robot.jr.getAngleToGoal());
    	
    	if(!goalFound){
    		//allow driver to move the drivetrain to where the bot sees the goal
    		if(Robot.jr.isGoalFound()){
    			System.out.println("goal found!");
    			goalFound = true;
    			Robot.states.driveControlModeTracker = DriveControlMode.AUTO;
    			System.out.println(Robot.jr.getAngleToGoal());
    			Robot.drivetrainSS.drivetrainTurnPIDSetSetpoint(Robot.jr.getAngleToGoal());
        		Robot.drivetrainSS.drivetrainTurnPIDEnable();
        		
    		}
    	}else if(goalFound && !withinRange){
    		//found goal now line up
    		if(Math.abs(Robot.drivetrainSS.getTurnPCError()) < .5){
    			
    			System.out.println("on target");
    			withinRange = true;
    			
    			Robot.drivetrainSS.drivetrainTurnPIDDisable();
    			
    		}
    	}else if(goalFound && withinRange && !hasRumbled){
    		System.out.println("rumbled");
    		new RumbleController(Robot.oi.dc).start();
    		hasRumbled = true;
    	}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Robot.oi.dc.getRightTrigger() < .4);
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    	System.out.println("lineupshot finished");
    	
    	Robot.drivetrainSS.drivetrainTurnPIDDisable();
    	Robot.states.autoLiningUp = false;
    	Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drivetrainSS.drivetrainTurnPIDDisable();
    	Robot.states.autoLiningUp = false;
    	Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
    }
}
