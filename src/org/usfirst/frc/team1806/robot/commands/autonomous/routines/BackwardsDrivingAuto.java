package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.autonomous.DoNothing;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveUntilFlat;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAngle;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID_Deprecated;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight_Deprecated;
import org.usfirst.frc.team1806.robot.commands.elevator.ResetElevator;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;
import org.usfirst.frc.team1806.robot.commands.shooter.ShootThenCock;

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
    	
    	//always lower the intake at the start
    	addSequential(new LowerIntake(2));
    	
    	if(!leaveArmUp){
    		addSequential(new MoveToHoldingPID());
    	}
    	addSequential(new DriveUntilFlat(-.8, 2));
    	if(!shouldTakeShot){
    		addSequential(new DoNothing());
    	}
    	else{
        	if(lane == 1 || !leaveArmUp){
        		addSequential(new MoveToShootingHeight());
        	}
    		addSequential(new TurnToAngle(Constants.autoBackwardsNearGoalAngles[lane-1], 3));
    		addSequential(new LineUpShot());
    		addSequential(new ShootThenCock());
    		addSequential(new DoNothing());
    	}   	

    }
}
