package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Constants;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MoveToChevalHeight extends CommandGroup {
    
    public  MoveToChevalHeight() {
        addSequential(new SetStateOther());
        addSequential(new MoveToLocationPID(Constants.elevatorChevaldeFunHeight));
    }
}
