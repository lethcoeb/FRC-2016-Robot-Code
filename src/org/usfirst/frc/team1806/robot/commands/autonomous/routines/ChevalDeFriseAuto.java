package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.Wait;
import org.usfirst.frc.team1806.robot.commands.autonomous.DoNothing;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveForTime;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAbsoluteAngle;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToLocationPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.SetStateShooting;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ChevalDeFriseAuto extends CommandGroup {
    
    public  ChevalDeFriseAuto(boolean shoot, int lane) {
        addParallel(new MoveToLocationPID(Constants.elevatorChevaldeFunHeight));
        addSequential(new DriveToPosition(42, .6));
        addSequential(new LowerIntake(1.5));
        addSequential(new DriveToPosition(68, .55));
        addParallel(new MoveToLocationPID(98800));
        addSequential(new DriveToPosition(132, .8));
        addSequential(new TurnToAbsoluteAngle(60));
        addSequential(new SetStateShooting());
        addSequential(new DriveToPosition(-24, .6));
        addSequential(new LineUpShot());
        
        /*if(shoot){
        	addParallel(new MoveToShootingHeight());
        	addSequential(new TurnToAbsoluteAngle(Constants.autoForwardsNearGoalAngles[lane]));
        	addSequential(new LineUpShot());
        	addSequential(new DoNothing());
        }else{
        	addSequential(new MoveToHoldingPID());
        	addSequential(new DoNothing());
        }*/
        
    }
}
