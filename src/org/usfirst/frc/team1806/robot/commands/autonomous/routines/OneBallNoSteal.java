package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.Wait;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAngle;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToVisionAngle;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToGrabPosition;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;
import org.usfirst.frc.team1806.robot.commands.shooter.ShootThenCock;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class OneBallNoSteal extends CommandGroup {

	boolean kDelay;
	int startingDefense;
	int returnDefense;

	public OneBallNoSteal(ArrayList<String> commandList) {

		if (commandList.get(2) == "Delay") {
			// delay it
			kDelay = true;
		} else {
			// no delay
			kDelay = false;
		}

		startingDefense = Integer.valueOf(commandList.get(3));
		returnDefense = Integer.valueOf(commandList.get(4));

		if (!kDelay) {
			// starting right in front of defense
			addParallel(new LowerIntake());
			addSequential(new MoveToGrabPosition());
			addSequential(new DriveToPosition(Constants.overDefense, 1));
		} else {
			// you're waiting
			addParallel(new LowerIntake());
			addParallel(new MoveToGrabPosition());
			addSequential(new Wait(Constants.lowBarToShotDelaySeconds));
			addSequential(new DriveToPosition(Constants.overDefensePlusDelay, 1));
		}

		//You have driven past the defense
		// Now you're at where you need to line up a shot

		if (startingDefense == 1) {
			//defense 1, low bar
			addSequential(new DriveToPosition(Constants.lowBarToAngledShot, 1));
			addParallel(new MoveToShootingHeight());
			addSequential(new TurnToAngle(Constants.lowBarAngleToGoal));
			addSequential(new TurnToVisionAngle());
			addSequential(new ShootThenCock());
			addSequential(new MoveToGrabPosition());
		}else if(startingDefense == 2){
			//defense 2
			addSequential(new TurnToAngle(90));
			addSequential(new DriveToPosition(Constants.defense2toDefense4, 1));
			addParallel(new MoveToShootingHeight());
			addSequential(new TurnToAngle(0));
			addSequential(new TurnToVisionAngle());
			addSequential(new ShootThenCock());
			addSequential(new MoveToGrabPosition());
		}else if(startingDefense == 3){
			addParallel(new MoveToShootingHeight());
			addSequential(new TurnToAngle(Constants.defense3angleToGoal));
			addSequential(new TurnToVisionAngle());
			addSequential(new ShootThenCock());
			addSequential(new MoveToGrabPosition());
			
			//defense 3
		}else if(startingDefense == 4){
			//defense 4
			//basically already on target
			addParallel(new MoveToShootingHeight());
			addSequential(new TurnToVisionAngle());
			addSequential(new ShootThenCock());
			addSequential(new MoveToGrabPosition());
		}else{
			//defense 5
			addSequential(new TurnToAngle(-90));
			addSequential(new DriveToPosition(Constants.defense2toDefense4, 1));
			addParallel(new MoveToShootingHeight());
			addSequential(new TurnToAngle(0));
			addSequential(new TurnToVisionAngle());
			addSequential(new ShootThenCock());
			addSequential(new MoveToGrabPosition());
		}

	}
}
