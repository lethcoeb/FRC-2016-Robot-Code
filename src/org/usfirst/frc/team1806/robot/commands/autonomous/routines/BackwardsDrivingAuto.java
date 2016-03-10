package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.autonomous.DoNothing;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveUntilFlat;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAngle;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.ResetElevator;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class BackwardsDrivingAuto extends CommandGroup {
    
    public  BackwardsDrivingAuto(boolean leaveArmUp, boolean shouldTakeShot, int lane) {
    	if(lane == 1){
    		//WE CANNOT GO BACKWARDS THROUGH THE LOW BAR
    		System.out.println("Becker pls.");
    		addSequential(new DoNothing());
    	}
    	if(!leaveArmUp){
    		addParallel(new LowerIntake());
    		addSequential(new ResetElevator());
    	}
    	addSequential(new DriveUntilFlat(-.6, 10));
    	if(!shouldTakeShot){
    		addSequential(new DoNothing());
    	}
    	else{
        	if(lane == 1 || !leaveArmUp){
        		addSequential(new MoveToShootingHeight());
        	}
    		addSequential(new TurnToAngle(Constants.autoBackwardsNearGoalAngles[lane-1], 3));
    		addSequential(new LineUpShot());
    		addSequential(new DoNothing());
    	}   	

    }
}
