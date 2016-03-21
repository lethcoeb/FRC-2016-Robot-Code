package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.Wait;
import org.usfirst.frc.team1806.robot.commands.autonomous.DoNothing;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAbsoluteAngle;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToLocationPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ChevalDeFriseAuto extends CommandGroup {
    
    public  ChevalDeFriseAuto(boolean shoot, int lane) {
        addParallel(new MoveToLocationPID(Constants.elevatorChevaldeFunHeight));
        addSequential(new DriveToPosition(14, .5));
        addParallel(new LowerIntake());
        addSequential(new Wait(1));
        addSequential(new DriveToPosition(14, .4));
        
        if(shoot){
        	addParallel(new MoveToShootingHeight());
        	addSequential(new TurnToAbsoluteAngle(Constants.autoForwardsNearGoalAngles[lane]));
        	addSequential(new LineUpShot());
        	addSequential(new DoNothing());
        }else{
        	addSequential(new MoveToHoldingPID());
        	addSequential(new DoNothing());
        }
        
    }
}
