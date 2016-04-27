package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAbsoluteAngle;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToFlushHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToLocationPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToOuterworksShootingHeight;
import org.usfirst.frc.team1806.robot.commands.intake.IntakeRunAtSpeed;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;
import org.usfirst.frc.team1806.robot.commands.intake.IntakeRunAtSpeed;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LowBarAuto extends CommandGroup {
    
    public  LowBarAuto() {
    	
    	addParallel(new IntakeRunAtSpeed(-.3));
    	
    	CommandGroup turnThenForwards = new CommandGroup();
    	turnThenForwards.addSequential(new TurnToAbsoluteAngle(60));
    	turnThenForwards.addSequential(new DriveToPosition(30, .5));
    	
    	CommandGroup lowerArms = new CommandGroup();
    	lowerArms.addSequential(new LowerIntake(.1));
    	lowerArms.addSequential(new MoveToLocationPID(0));
    	lowerArms.addSequential(new IntakeRunAtSpeed(0));
    	
    	
    	CommandGroup beforeLowBar = new CommandGroup();
    	beforeLowBar.addParallel(new DriveToPosition(24, .4));
    	beforeLowBar.addSequential(lowerArms);
    	
    	addSequential(beforeLowBar);
    	addSequential(new DriveToPosition(170, .95));
    	addParallel(turnThenForwards);
    	addSequential(new MoveToFlushHeight());
		addSequential(new LineUpShot());
    	
    }
}
