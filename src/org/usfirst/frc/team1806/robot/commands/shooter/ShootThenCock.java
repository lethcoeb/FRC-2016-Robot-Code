package org.usfirst.frc.team1806.robot.commands.shooter;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.Wait;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ShootThenCock extends CommandGroup {
    
    public  ShootThenCock() {
        addSequential(new ShootDaBall());
        addSequential(new Wait(Constants.timeToShoot));
        addSequential(new CockShooter());
    }
}
