package org.usfirst.frc.team1806.robot.commands.autonomous.routines;

import org.usfirst.frc.team1806.robot.Constants;
import org.usfirst.frc.team1806.robot.commands.Wait;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToLocationPID;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;
import org.usfirst.frc.team1806.robot.commands.intake.IntakeRunAtSpeed;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class StealAuto extends CommandGroup {
    
    public  StealAuto() {
    	
    	CommandGroup grabBall = new CommandGroup();
    	
        grabBall.addSequential(new LowerIntake(0));
        grabBall.addSequential(new Wait(.5));
        grabBall.addParallel(new IntakeRunAtSpeed(-1));
        //grabBall.addSequential(command);
        
        
        addParallel(grabBall);
        addSequential(new MoveToLocationPID(Constants.elevatorHoldingHeight));
        
    }
}
