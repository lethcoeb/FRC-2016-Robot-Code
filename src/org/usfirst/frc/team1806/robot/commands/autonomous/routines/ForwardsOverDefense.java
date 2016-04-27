package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.ShiftLow;
import org.usfirst.frc.team1806.robot.commands.StopRobotMovement;
import org.usfirst.frc.team1806.robot.commands.Wait;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveUntilFlat;
import org.usfirst.frc.team1806.robot.commands.autonomous.ForwardsUntilEncoderCount;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAbsoluteAngle;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToBatterShotHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToLocationPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToOuterworksShootingHeight;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ForwardsOverDefense extends CommandGroup {
	
    public  ForwardsOverDefense(int lane) {
    	
    	CommandGroup driveoverdefenseCommandGroup = new CommandGroup();
    	driveoverdefenseCommandGroup.addSequential(new ShiftLow());
    	driveoverdefenseCommandGroup.addSequential(new ForwardsUntilEncoderCount(Constants.distanceFromStartingPosToDefenses, .6));
    	driveoverdefenseCommandGroup.addSequential(new DriveUntilFlat(.6, false, 60));
    	
    	CommandGroup drivingTowardsGoal = new CommandGroup();
    	drivingTowardsGoal.addSequential(new TurnToAbsoluteAngle(Constants.autoForwardsNearGoalAngles[lane-1]));
    	drivingTowardsGoal.addSequential(new DriveToPosition(24, .45));
    	drivingTowardsGoal.addSequential(new Wait(1));
    	
        addParallel(new MoveToLocationPID(Constants.elevatorChevaldeFunHeight));
        addSequential(driveoverdefenseCommandGroup);
        addParallel(new LowerIntake(.1));
        addParallel(new MoveToOuterworksShootingHeight());
        addSequential(drivingTowardsGoal);
        addSequential(new LineUpShot());
    }
}
