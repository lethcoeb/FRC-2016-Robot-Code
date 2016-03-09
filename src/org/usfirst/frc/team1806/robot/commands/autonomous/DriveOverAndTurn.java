package org.usfirst.frc.team1806.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveOverAndTurn extends CommandGroup {
    
    public  DriveOverAndTurn() {
        addSequential(new DriveToPosition(-15, .6));
        addSequential(new TurnToAbsoluteAngle(90));
        addSequential(new DriveToPosition(3, .5));
        addSequential(new TurnToAbsoluteAngle(180));
    }
}
