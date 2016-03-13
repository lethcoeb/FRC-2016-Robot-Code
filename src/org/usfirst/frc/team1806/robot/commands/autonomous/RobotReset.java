package org.usfirst.frc.team1806.robot.commands.autonomous;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RobotReset extends CommandGroup {
    
    public  RobotReset() {
        addSequential(new LowerIntake(2));
        addSequential(new MoveToHoldingPID());
    }
}
