package org.usfirst.frc.team1806.robot.commands.autonomous.routines.subroutines;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.ShiftLow;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveUntilFlat;
import org.usfirst.frc.team1806.robot.commands.autonomous.ForwardsUntilEncoderCount;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class GyroOverDefense extends CommandGroup {
    
    public  GyroOverDefense() {
        
    	addSequential(new ShiftLow());
    	addSequential(new ForwardsUntilEncoderCount(Constants.distanceFromStartingPosToDefenses, .6));
    	addSequential(new DriveUntilFlat(.6, false, 60));
    	
    }
}
