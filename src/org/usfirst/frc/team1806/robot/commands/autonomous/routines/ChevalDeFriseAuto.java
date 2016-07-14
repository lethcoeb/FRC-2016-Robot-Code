package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.StopRobotMovement;
import org.usfirst.frc.team1806.robot.commands.Wait;
import org.usfirst.frc.team1806.robot.commands.autonomous.DoNothing;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveForTime;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.ForwardsUntilEncoderCount;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAbsoluteAngle;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToBatterShotHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToLocationPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToOuterworksShootingHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.SetStateShooting;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ChevalDeFriseAuto extends CommandGroup {

	public ChevalDeFriseAuto(int lane) {

		
		CommandGroup crossCheval = new CommandGroup();
		crossCheval.addParallel(new MoveToLocationPID(Constants.elevatorChevaldeFunHeight));
		crossCheval.addSequential(new DriveToPosition(44, .8));
		crossCheval.addSequential(new LowerIntake(1.5));
		crossCheval.addSequential(new DriveToPosition(85, .7));
		
		if (lane == 3 || lane == 4) {
			CommandGroup drivingTowardsGoal = new CommandGroup();
			
			drivingTowardsGoal.addSequential(new TurnToAbsoluteAngle(Constants.autoForwardsNearGoalAngles[lane - 1]));
			drivingTowardsGoal.addSequential(new DriveToPosition(18, .4));

			addSequential(crossCheval);
			addSequential(new ForwardsUntilEncoderCount(5, .4));
			addParallel(new MoveToOuterworksShootingHeight(96000));
			addSequential(drivingTowardsGoal);
			addSequential(new LineUpShot());
		}else if(lane == 2){
			addSequential(crossCheval);
			addParallel(new MoveToOuterworksShootingHeight());
			addSequential(new DriveToPosition(110, .8));
			addParallel(new MoveToBatterShotHeight());
			addSequential(new TurnToAbsoluteAngle(60));
			addSequential(new LineUpShot());
		}else if(lane == 5){
			addSequential(crossCheval);
			addParallel(new MoveToOuterworksShootingHeight());
			addSequential(new DriveToPosition(100, .8));
			addParallel(new MoveToBatterShotHeight());
			addSequential(new TurnToAbsoluteAngle(-10));
			addSequential(new LineUpShot());
		}

		/*
		 * if(shoot){ addParallel(new MoveToShootingHeight()); addSequential(new
		 * TurnToAbsoluteAngle(Constants.autoForwardsNearGoalAngles[lane]));
		 * addSequential(new LineUpShot()); addSequential(new DoNothing());
		 * }else{ addSequential(new MoveToHoldingPID()); addSequential(new
		 * DoNothing()); }
		 */

	}
}
