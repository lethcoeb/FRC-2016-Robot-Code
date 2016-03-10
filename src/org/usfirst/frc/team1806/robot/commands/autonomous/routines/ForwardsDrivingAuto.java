package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.autonomous.DoNothing;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveUntilFlat;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAngle;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToGrabPosition;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.ResetElevator;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ForwardsDrivingAuto extends CommandGroup {
    
    public  ForwardsDrivingAuto(boolean leaveArmUp, boolean shouldTakeShot, int lane) {
    	if(lane == 1 || !leaveArmUp){
    		addParallel(new LowerIntake());
    		addSequential(new ResetElevator());
    	}
    	if(lane == 1){
    		//if going low bar, get arm and intake ready for low bar
    		addParallel(new MoveToGrabPosition());
    	}
    	addSequential(new DriveUntilFlat(.6, 10));

    	if(!shouldTakeShot){
    		addSequential(new DoNothing());
    	}
    	else{
        	if(lane == 1 || !leaveArmUp){
        		addSequential(new MoveToShootingHeight());
        	}
    		addSequential(new TurnToAngle(Constants.autoForwardsNearGoalAngles[lane], 3));
    		addSequential(new LineUpShot());
    		addSequential(new DoNothing());
    	}
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
