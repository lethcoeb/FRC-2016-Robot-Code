package org.usfirst.frc.team1806.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MoveToFlushHeight extends CommandGroup {
    
    public  MoveToFlushHeight() {
        
    	addSequential(new SetStateOther());
    	addSequential(new MoveToLocationPID(96000));
    	addSequential(new SetStateShooting());
    	
    }
}
