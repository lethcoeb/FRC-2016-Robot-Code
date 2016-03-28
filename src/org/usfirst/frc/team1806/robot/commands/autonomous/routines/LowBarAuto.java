package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAbsoluteAngle;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToLocationPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.SetStateShooting;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;
import org.usfirst.frc.team1806.robot.commands.intake.RunAtSpeed;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LowBarAuto extends CommandGroup {
    
    public  LowBarAuto() {
    	
    	addParallel(new RunAtSpeed(-.3));
    	CommandGroup lowerArms = new CommandGroup();
    	lowerArms.addSequential(new LowerIntake(.5));
    	lowerArms.addSequential(new MoveToLocationPID(0));
    	
    	CommandGroup beforeLowBar = new CommandGroup();
    	beforeLowBar.addParallel(lowerArms);
    	beforeLowBar.addSequential(new DriveToPosition(24, .4));
    	
    	addSequential(beforeLowBar);
    	
    	addSequential(new RunAtSpeed(0));
    	addSequential(new DriveToPosition(170, .75));
    	addParallel(new MoveToShootingHeight());
    	addSequential(new TurnToAbsoluteAngle(66));
		addSequential(new DriveToPosition(30, .5));
		addSequential(new LineUpShot());
    	
    }
}
