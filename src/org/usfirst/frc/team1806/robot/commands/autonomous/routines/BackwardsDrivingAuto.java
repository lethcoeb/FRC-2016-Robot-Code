package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.autonomous.DoNothing;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveUntilFlat;
import org.usfirst.frc.team1806.robot.commands.autonomous.Turn90;
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

	public BackwardsDrivingAuto(boolean leaveArmUp, boolean shouldTakeShot, int lane) {
		if (lane == 1) {
			// WE CANNOT GO BACKWARDS THROUGH THE LOW BAR
			System.out.println("Becker pls.");
			addSequential(new DoNothing());
		} else {

			// always lower the intake at the start
			addSequential(new LowerIntake(1));

			if (!leaveArmUp) {
				addSequential(new MoveToHoldingPID());
			}
			//addSequential(new DriveUntilFlat(-.6, 3, 25));
			addSequential(new DriveToPosition(-14, .6));
			if (!shouldTakeShot) {
				addSequential(new DoNothing());
			} else {
				if (lane == 1 || !leaveArmUp) {
					addParallel(new MoveToShootingHeight());
				}
				
				if(lane == 5){
					addSequential(new Turn90(1.5, false));
					addSequential(new DriveToPosition(-8, .5));
					addSequential(new Turn90(1.5, false));
				}else if(lane == 2){
					addSequential(new Turn90(1.5, true));
					addSequential(new DriveToPosition(-8, .5));
					addSequential(new Turn90(1.5, true));
				}else{
					addSequential(new TurnToAngle(Constants.autoBackwardsNearGoalAngles[lane], 3));
				}
				
				addSequential(new LineUpShot());
				addSequential(new DoNothing());
			}

		}
	}
}
