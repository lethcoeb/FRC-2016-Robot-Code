package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAbsoluteAngle;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToLocationPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.SetStateShooting;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class LowBarAuto extends CommandGroup {
    
    public  LowBarAuto() {
    	
    	CommandGroup lowerArms = new CommandGroup();
    	lowerArms.addSequential(new LowerIntake(1));
    	lowerArms.addSequential(new MoveToLocationPID(0));
    	
    	CommandGroup beforeLowBar = new CommandGroup();
    	beforeLowBar.addParallel(lowerArms);
    	beforeLowBar.addSequential(new DriveToPosition(30, .4));
    	
    	addSequential(beforeLowBar);
    	
    	addSequential(new DriveToPosition(155, .75));
    	addParallel(new MoveToLocationPID(97750));
    	addSequential(new TurnToAbsoluteAngle(62));
		addSequential(new DriveToPosition(44, .5));
		addSequential(new SetStateShooting());
		addSequential(new LineUpShot());
    	
    }
}
