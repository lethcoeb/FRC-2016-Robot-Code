package org.usfirst.frc.team1806.robot.commands.autonomous;

import org.usfirst.frc.team1806.robot.commands.elevator.MoveToGrabPosition;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class FourteenInchMode extends CommandGroup {
    
    public  FourteenInchMode() {
        addParallel(new LowerIntake());
        addParallel(new MoveToGrabPosition());
    }
}
