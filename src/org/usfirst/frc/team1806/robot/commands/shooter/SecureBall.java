package org.usfirst.frc.team1806.robot.commands.shooter;

import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingFromLow;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SecureBall extends CommandGroup {
    
    public  SecureBall() {
        addSequential(new PinchBall());
        addSequential(new MoveToHoldingFromLow());
    }
}
