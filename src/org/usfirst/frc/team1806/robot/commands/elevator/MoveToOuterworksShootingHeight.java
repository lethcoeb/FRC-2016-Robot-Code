package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Constants;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MoveToOuterworksShootingHeight extends CommandGroup {
    
    public  MoveToOuterworksShootingHeight() {
    	
    	addSequential(new SetStateOther());
        addSequential(new MoveToLocationPID(Constants.elevatorShootingHeight));
        addSequential(new SetStateShooting());
        
    }
}
