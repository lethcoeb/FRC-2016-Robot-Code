package org.usfirst.frc.team1806.robot.commands.elevator;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.RobotStates.Mode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MoveToFlushHeight extends CommandGroup {
    
    public  MoveToFlushHeight(double timeout) {
    	
    	addSequential(new SetStateOther());
    	addSequential(new MoveToLocationPID(98000, timeout));
    	addSequential(new SetStateShooting());
    	
    }
}
