package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.Wait;
import org.usfirst.frc.team1806.robot.commands.autonomous.BumpStop;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAbsoluteAngle;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.subroutines.GyroOverDefense;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToBatterShotHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToChevalHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToLocationPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToOuterworksShootingHeight;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ForwardsOverPos2or5Defense extends CommandGroup {
    
	int inchesAfterDefense = 100;
	int initialAngleToGoal = 0;
	
    public  ForwardsOverPos2or5Defense(int lane) {
    	
    	if(lane == 2){
    		inchesAfterDefense = 110;
    		initialAngleToGoal = 60;
    	}else if(lane == 5){
    		inchesAfterDefense = 100;
    		initialAngleToGoal = -20;
    	}
    	
    	addParallel(new MoveToLocationPID(Constants.elevatorChevaldeFunHeight));
    	addSequential(new GyroOverDefense());
    	addParallel(new MoveToOuterworksShootingHeight());
    	addSequential(new DriveToPosition(inchesAfterDefense, .8));
    	addSequential(new BumpStop(true));
    	addParallel(new MoveToBatterShotHeight());
    	addSequential(new TurnToAbsoluteAngle(initialAngleToGoal));
    	addSequential(new Wait(10.0));
    	addSequential(new LineUpShot());
    	
    }
}
