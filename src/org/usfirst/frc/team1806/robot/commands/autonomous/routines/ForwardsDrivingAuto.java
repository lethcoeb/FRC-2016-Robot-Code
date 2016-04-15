package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.autonomous.DoNothing;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveUntilFlat;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAngle;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToGrabPosition;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToGrabPosition_Deprecated;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID_Deprecated;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToLocationPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight_Deprecated;
import org.usfirst.frc.team1806.robot.commands.elevator.ResetElevator;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;
import org.usfirst.frc.team1806.robot.commands.shooter.ShootThenCock;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ForwardsDrivingAuto extends CommandGroup {
    
    public  ForwardsDrivingAuto(boolean leaveArmUp, boolean shouldTakeShot, int lane)
    {
    	
    	//always lower the intake at the start
		/*addSequential(new LowerIntake());

    	if(lane == 1 || !leaveArmUp){
    		addSequential(new MoveToHoldingPID());
    	}
    	if(lane == 1){
    		//if going low bar, get arm and intake ready for low bar
    		addParallel(new MoveToGrabPosition());
    	}
    	addSequential(new DriveUntilFlat(.8, 3.5));

    	if(!shouldTakeShot){
    		addSequential(new DoNothing());
    	}
    	else{
        	if(lane == 1 || !leaveArmUp){
        		addSequential(new MoveToShootingHeight());
        	}
    		addSequential(new TurnToAngle(Constants.autoForwardsNearGoalAngles[lane-1], 3));
    		addSequential(new LineUpShot());
    		addSequential(new ShootThenCock());
    		addSequential(new DoNothing());
    	}*/


    	addParallel(new MoveToLocationPID(Constants.elevatorChevaldeFunHeight));
    	addSequential(new DriveToPosition(120, .8));
    	addParallel(new MoveToShootingHeight());
    	addParallel(new LowerIntake(.5));
    	addSequential(new LineUpShot());
    	
    	
    }
}
