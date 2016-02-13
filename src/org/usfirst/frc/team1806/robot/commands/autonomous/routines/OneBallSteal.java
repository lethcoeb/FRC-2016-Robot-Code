package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.Robot;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.subroutines.StealBall;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToGrabPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class OneBallSteal extends CommandGroup {
    
    public  OneBallSteal(ArrayList<String> commandList) {
    	//dis not done
    	Robot.states.hasBall = false;
        addParallel(new StealBall());
        addSequential(new MoveToGrabPosition());
    }
}
