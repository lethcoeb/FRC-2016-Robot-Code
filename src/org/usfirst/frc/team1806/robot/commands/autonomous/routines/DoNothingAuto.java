package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.commands.autonomous.DoNothing;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DoNothingAuto extends CommandGroup {
    
    public  DoNothingAuto() {
    	addSequential(new DoNothing());
    }
}
