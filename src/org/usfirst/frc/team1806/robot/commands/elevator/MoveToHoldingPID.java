package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Constants;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MoveToHoldingPID extends CommandGroup {
    
    public  MoveToHoldingPID() {
        
    	addSequential(new SetStateOther());
    	addSequential(new MoveToLocationPID(Constants.elevatorHoldingHeight));
    	addSequential(new SetStateHolding());
    	
    }
}
