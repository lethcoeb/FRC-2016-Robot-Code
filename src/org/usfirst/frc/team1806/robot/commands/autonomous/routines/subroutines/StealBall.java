package org.usfirst.frc.team1806.robot.commands.autonomous.routines.subroutines;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.Wait;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.subroutines.intake.IntakeBall;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.subroutines.intake.ThrowOutIntake;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class StealBall extends CommandGroup {
    
    public  StealBall() {
        addSequential(new ThrowOutIntake());
        addSequential(new Wait(Constants.timeToDeployOnBall));
        addParallel(new IntakeBall());
        addSequential(new DriveToPosition(-24, 1));
    }
}
