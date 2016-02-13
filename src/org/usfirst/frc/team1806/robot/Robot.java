
package org.usfirst.frc.team1806.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc.team1806.robot.subsystems.ElevatorSubsystem;
import org.usfirst.frc.team1806.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team1806.robot.subsystems.ShooterSubsystem;

import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.RobotStates.DriveControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.OneBallNoSteal;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.OneBallSteal;
import org.usfirst.frc.team1806.robot.commands.shooter.PinchBall;
import org.usfirst.frc.team1806.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static DrivetrainSubsystem drivetrainSS;
	public static ElevatorSubsystem elevatorSS;
	public static IntakeSubsystem intakeSS;
	public static ShooterSubsystem shooterSS;

	public static OI oi;
	public static RobotStates states;

	Command autonomousCommand;
	SendableChooser xBall, robotStartingPos, waitPosition;

	public static JetsonReceiver jr;
	public static AutonomousReader ar;
	public static SmartDashboardUpdater sdu;
	public static RioVisionThread rvt;

	public void robotInit() {

		drivetrainSS = new DrivetrainSubsystem();
		elevatorSS = new ElevatorSubsystem();
		intakeSS = new IntakeSubsystem();
		shooterSS = new ShooterSubsystem();

		oi = new OI();
		states = new RobotStates();

		jr = new JetsonReceiver();
		jr.start();

		ar = new AutonomousReader();
		ar.start();
		
		sdu = new SmartDashboardUpdater();
		
		rvt = new RioVisionThread();
		rvt.start();

	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	public void disabledInit() {

	}

	public void disabledPeriodic() {
		
		Scheduler.getInstance().run();
		SmartDashboard.putNumber("angle", drivetrainSS.getTrueAngle());
		sdu.push();

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	public void autonomousInit() {

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */
		
		ar.stopThread();
		
		ArrayList<String>  commandStrings = new ArrayList<String>();
		if (ar.CommandsReceived()) {
			commandStrings = ar.getCommandStringArray();
		}

		// You got a routine boy, now parse it
		if (commandStrings.get(0) == "OneBall") {
			if (commandStrings.get(1) == "Steal") {
				new OneBallSteal(commandStrings);
			} else if (commandStrings.get(1) == "NoSteal") {
				new OneBallNoSteal(commandStrings);
			}
		} else if (commandStrings.get(0) == "TwoBall") {
			// run 2 ball auto
			// literally a dream
		}

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		sdu.push();
	}

	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		//if (autonomousCommand != null)
		//	autonomousCommand.cancel();
		Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

		Scheduler.getInstance().run();
		oi.update();

		// automatic sensor listener, put this somewhere else bc it's so ugleh
		if (Robot.shooterSS.hasBallSensor() && !Robot.states.hasBall
				&& Robot.states.shooterArmPositionTracker == ShooterArmPosition.DOWN) {
			new PinchBall().start();
		}
		
		sdu.push();
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
		sdu.push();
	}

}
