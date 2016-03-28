package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LowBarTeleop extends Command {

	boolean stopped = false;
	boolean timeout = false;
	Timer timer;
	
    public LowBarTeleop() {
        requires(Robot.intakeSS);
        requires(Robot.elevatorSS);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.states.lowBarring = true;
    	timer = new Timer();
    	timer.reset();
    	timer.start();
    	Robot.intakeSS.runAtSpeed(-.3);
    	Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
    	Robot.elevatorSS.elevatorSetSetpoint(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	if(Robot.elevatorSS.isBottomLimitHit() && !stopped){
    		stopped = true;
    		System.out.println("stopped by limit switch");
    		Robot.elevatorSS.elevatorStopMovement();
    		Robot.states.shooterArmPositionTracker = ShooterArmPosition.DOWN;
    		Robot.intakeSS.stopIntaking();
    	}else if(Math.abs(Robot.elevatorSS.getElevatorPosition()) < 500 && !stopped){
    		stopped = true;
    		System.out.println("stopped by PID loop");
    		Robot.elevatorSS.elevatorStopMovement();
    		Robot.states.shooterArmPositionTracker = ShooterArmPosition.DOWN;
    		Robot.intakeSS.stopIntaking();
    	}
    	
    	if(timer.get() > .25){
    		timeout = true;
    		timer.stop();
    		timer.reset();
    	}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.oi.lowBarLatch.update(Robot.oi.dA) && timeout;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("LowBarTeleop finished");
    	Robot.states.lowBarring = false;
    	new MoveToHoldingFromLow(Robot.states.hasBall);
    	timer.stop();
    	timer.reset();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	timer.stop();
    	timer.reset();
    	Robot.states.lowBarring = false;
    }
}
