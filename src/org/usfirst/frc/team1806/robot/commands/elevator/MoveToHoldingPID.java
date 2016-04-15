package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.ResetOvershoot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MoveToHoldingPID extends CommandGroup {
    
    public  MoveToHoldingPID() {
        
    	addSequential(new ResetOvershoot());
    	addSequential(new SetStateOther());
    	addSequential(new MoveToLocationPID(Constants.elevatorHoldingHeight));
    	addSequential(new SetStateHolding());
    	
    }
}
