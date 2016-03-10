package org.usfirst.frc.team1806.robot.commands.autonomous;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.DriveControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.Mode;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToAngle extends Command {

	boolean finished;
	double targetAngle;
	int stage;
	double loops = 0;
	double kLoopsUntilCheck = Constants.loopsToCheckSensorDisconnect;
	
    public TurnToAngle(double target) {
        requires(Robot.drivetrainSS);
        targetAngle = target;
    }
    
    public TurnToAngle(double target, int accuracy){
    	requires(Robot.drivetrainSS);
    	targetAngle = target;
    	//this constructor should be used if you want to turn without all 3 pid loop stages, as in you want less accuracy faster
    	//ie a 90deg turn in auto, because sorry jones but you do not need a .2 degree accuracy on it
    	//TODO implement this
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.states.driveControlModeTracker = DriveControlMode.AUTO;
    	Robot.drivetrainSS.drivetrainTurnPIDReset();
    	Robot.drivetrainSS.resetYaw();
    	
    	if(Math.abs(targetAngle) > 15){
    		//use big ol' pid
    		Robot.drivetrainSS.drivetrainTurnPIDchangePID(Constants.drivetrainTurn1P, Constants.drivetrainTurn1I, Constants.drivetrainTurn1D);
    		Robot.drivetrainSS.drivetrainTurnPIDSetTolerance(Constants.drivetrainTurnPID1Tolerance);
    		Robot.drivetrainSS.drivetrainTurnPIDchangeMaxRotation(Constants.drivetrainMaxRotationPIDStage3);
    		stage = 3;
    	}else if(Math.abs(targetAngle) > 5){
    		Robot.drivetrainSS.drivetrainTurnPIDchangePID(Constants.drivetrainTurn2P, Constants.drivetrainTurn2I, Constants.drivetrainTurn2D);
    		Robot.drivetrainSS.drivetrainTurnPIDSetTolerance(Constants.drivetrainTurnPID2Tolerance);
    		Robot.drivetrainSS.drivetrainTurnPIDchangeMaxRotation(Constants.drivetrainMaxRotationPIDStage2);
    		stage = 2;
    	}else{
    		//you hella close bruh use dat small loop
    		Robot.drivetrainSS.drivetrainTurnPIDchangePID(Constants.drivetrainTurn3P, Constants.drivetrainTurn3I, Constants.drivetrainTurn3D);
    		Robot.drivetrainSS.drivetrainTurnPIDSetTolerance(Constants.drivetrainTurnPID3Tolerance);
    		Robot.drivetrainSS.drivetrainTurnPIDchangeMaxRotation(Constants.drivetrainMaxRotationPIDStage1);
    		stage = 1;
    	}
    	
    	Robot.drivetrainSS.drivetrainTurnPIDSetSetpoint(targetAngle);
    	Robot.drivetrainSS.drivetrainTurnPIDEnable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	loops++;
    	if(loops >= kLoopsUntilCheck){
    		loops = 0;
    		if(!Robot.drivetrainSS.isNavxConnected()){
    			//navx is dead bruh kill it
    			Robot.drivetrainSS.drivetrainTurnPIDDisable();
    			finished = true;
    		}
    	}
    	if(Robot.drivetrainSS.drivetrainTurnPIDisOnTarget()){
    		System.out.println("PID on target");
    		Robot.drivetrainSS.drivetrainTurnPIDDisable();
    		Robot.drivetrainSS.drivetrainTurnPIDReset();
    		if(stage == 3){
    			//step down to stage 2
    			stage = 2;
    			Robot.drivetrainSS.drivetrainTurnPIDchangePID(Constants.drivetrainTurn2P, Constants.drivetrainTurn2I, Constants.drivetrainTurn2D);
    			Robot.drivetrainSS.drivetrainTurnPIDSetTolerance(Constants.drivetrainTurnPID2Tolerance);
    			Robot.drivetrainSS.drivetrainTurnPIDchangeMaxRotation(Constants.drivetrainMaxRotationPIDStage2);
    			Robot.drivetrainSS.drivetrainTurnPIDEnable();
    			
    			System.out.println("moving to stage 2");
    		}else if(stage == 2){
    			//step down to stage 1
    			stage = 1;
    			Robot.drivetrainSS.drivetrainTurnPIDchangePID(Constants.drivetrainTurn3P, Constants.drivetrainTurn3I, Constants.drivetrainTurn3D);
    			Robot.drivetrainSS.drivetrainTurnPIDSetTolerance(Constants.drivetrainTurnPID3Tolerance);
    			Robot.drivetrainSS.drivetrainTurnPIDchangeMaxRotation(Constants.drivetrainMaxRotationPIDStage3);
    			Robot.drivetrainSS.drivetrainTurnPIDEnable();
    			
    			System.out.println("moving to stage 1");
    		}else if(stage == 1 && Robot.jr.isAngleAcceptable()){
    			//should be done.
    			finished = true;
    			System.out.println("stage one done");
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	//LT for an override
        return finished || (Math.abs(Robot.oi.dc.getLeftTrigger()) > .5 && Robot.states.mode == Mode.AUTONOMOUS);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
    	Robot.drivetrainSS.drivetrainTurnPIDDisable();
    	System.out.println("TurnToAngleFinished");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
    	Robot.drivetrainSS.drivetrainTurnPIDDisable();
    }
}
