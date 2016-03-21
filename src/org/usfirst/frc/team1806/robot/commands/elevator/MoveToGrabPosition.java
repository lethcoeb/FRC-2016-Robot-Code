package org.usfirst.frc.team1806.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MoveToGrabPosition extends CommandGroup {
    
    public  MoveToGrabPosition() {
        addSequential(new MoveToLocationPID(0));
        addSequential(new SetStateGrab());
    }
}
