
package org.usfirst.frc.team1806.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick.RumbleType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.usfirst.frc.team1806.robot.subsystems.ElevatorSubsystem;
import org.usfirst.frc.team1806.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team1806.robot.subsystems.ShooterSubsystem;

import java.util.ArrayList;

import org.usfirst.frc.team1806.robot.RobotStates.DriveControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.Mode;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;
import org.usfirst.frc.team1806.robot.commands.DriverControlDrivetrain;
import org.usfirst.frc.team1806.robot.commands.RumbleController;
import org.usfirst.frc.team1806.robot.commands.Wait;
import org.usfirst.frc.team1806.robot.commands.autonomous.DoNothing;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.FourteenInchMode;
import org.usfirst.frc.team1806.robot.commands.autonomous.RobotReset;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAbsoluteAngle;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.BackwardsDrivingAuto;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.DoNothingAuto;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.DoSomething;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.ForwardsDrivingAuto;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.OneBallNoSteal;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.OneBallSteal;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID_Deprecated;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight_Deprecated;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;
import org.usfirst.frc.team1806.robot.commands.shooter.CockShooter;
import org.usfirst.frc.team1806.robot.commands.shooter.PinchBall;
import org.usfirst.frc.team1806.robot.commands.shooter.SecureBall;
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
	public static PowerDistributionPanel pdp;
	public static Compressor compressor;

	public static OperatorInterface oi;
	public static RobotStates states;
	

	CommandGroup autonomousCommand;
	SendableChooser xBall, robotStartingPos, waitPosition;

	public static JetsonReceiver jr;
	public static AutonomousReader ar;
	public static SmartDashboardUpdater sdu;
	public static RioVisionThread rvt;
	
	public static SendableChooser autonomous;
	public static SendableChooser autoArmUpOrDown;
	public static SendableChooser autoForwardOrBackward;
	public static SendableChooser autoShoot;
	public static SendableChooser autoLane;
	
	boolean hasBeenEnabled = false;

	public static double getPDPResistance(int channel) {
		return pdp.getVoltage() / pdp.getCurrent(channel);
	}

	public void robotInit() {
		//init subsystems
		
		
		drivetrainSS = new DrivetrainSubsystem();
		elevatorSS = new ElevatorSubsystem();
		intakeSS = new IntakeSubsystem();
		shooterSS = new ShooterSubsystem();
		pdp = new PowerDistributionPanel();
		compressor = new Compressor();

		oi = new OperatorInterface();
		states = new RobotStates();

		if (elevatorSS.isBottomLimitHit()) {
			Robot.elevatorSS.resetElevatorEncoder();
			states.shooterArmPositionTracker = ShooterArmPosition.DOWN;// to
																		// speed
																		// up
																		// testing
		}
		//jetson
		jr = new JetsonReceiver();
		jr.start();
		//autoreader for future events
		ar = new AutonomousReader();
		ar.start();
		
		//Temp QaDAS. Make sure all object in same chooser are of same type.


		sdu = new SmartDashboardUpdater();
		
		
		autonomous = new SendableChooser();
		autoArmUpOrDown = new SendableChooser();
		autoForwardOrBackward = new SendableChooser();
		autoShoot = new SendableChooser();
		autoLane = new SendableChooser();
		
		
		autonomous.addDefault("Run Auto: No","N");
		autonomous.addObject("Run Auto: Yes", "Y");
		SmartDashboard.putData("Run Autonomous?", autonomous);
		
		autoArmUpOrDown.addDefault("ArmUp", true);
		autoArmUpOrDown.addObject("ArmDown", false);
		SmartDashboard.putData("Reset Claw and put in hold position over defense?", autoArmUpOrDown);
		
		autoForwardOrBackward.addDefault("DriveBackward", "B");
		autoForwardOrBackward.addObject("DriveForward", "F");
		SmartDashboard.putData("Auto Direction", autoForwardOrBackward);
		
		
		autoShoot.addDefault("Don't Shoot", false);
		autoShoot.addObject("Shoot", true);
		SmartDashboard.putData("Shoot?", autoShoot);
		
		
		autoLane.addDefault("Low Bar", 1);
		autoLane.addObject("2", 2);
		autoLane.addObject("3", 3);
		autoLane.addObject("4", 4);
		autoLane.addObject("5", 5);
		SmartDashboard.putData("Defense Position? (Selecting Low bar will override any arm settings)",autoLane);

		//rvt = new RioVisionThread();
		//rvt.start();



		//compressor.setClosedLoopControl(false);
		
		
		
		
		
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */

	public void disabledInit() {
		oi.stopRumbles();
	}

	public void disabledPeriodic() {

		Scheduler.getInstance().run();
		sdu.push();
		
		if(!hasBeenEnabled){
			Robot.elevatorSS.elevatorSetPosition(-Constants.elevatorShootingHeight);
		}
		//System.out.println("Get POV: " + oi.oc.getPOV());
		//System.out.println("Get POV Count: " + oi.oc.getPOVCount());

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
		
		Robot.states.mode = Mode.AUTONOMOUS;
		hasBeenEnabled = true;
		
		autonomousCommand = new CommandGroup();
		CommandGroup subgroupCommandGroup = new CommandGroup();
		subgroupCommandGroup.addSequential(new Wait(4));
		subgroupCommandGroup.addSequential(new MoveToShootingHeight());
		
		Robot.drivetrainSS.shiftLow();
		Robot.drivetrainSS.resetYaw();
		Robot.drivetrainSS.resetEncoders();
		autonomousCommand.addSequential(new LowerIntake(.5));
		autonomousCommand.addSequential(new MoveToHoldingPID());
		//autonomousCommand.addParallel(subgroupCommandGroup);
		autonomousCommand.addSequential(new DriveToPosition(21, .85));
		autonomousCommand.addParallel(new MoveToShootingHeight());
		autonomousCommand.addSequential(new TurnToAbsoluteAngle(62));
		autonomousCommand.addSequential(new DriveToPosition(2, .5));
		autonomousCommand.addSequential(new LineUpShot());
		
		
		/*if(autonomous.getSelected() == "N"){
			autonomousCommand = new RobotReset();
		}
		else{
			if(autoForwardOrBackward.getSelected() == "F"){
				autonomousCommand = new ForwardsDrivingAuto((boolean) autoArmUpOrDown.getSelected(), (boolean) autoShoot.getSelected(), (int) autoLane.getSelected());
			}
			else{
				autonomousCommand = new BackwardsDrivingAuto((boolean) autoArmUpOrDown.getSelected(), (boolean) autoShoot.getSelected(), (int) autoLane.getSelected());
			}
		}*/
		
		autonomousCommand.start();
		/*String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		switch (autoSelected) {
		case "My Auto":
			autonomousCommand = new MyAutoCommand();
			break;
		case "Default Auto":
		default:
			autonomousCommand = new ExampleCommand();
			break;
		}*/
		/* 
		autonomousCommand = new FourteenInchMode();

		ar.stopThread();

		ArrayList<String> commandStrings = new ArrayList<String>();
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
	}
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		sdu.push();
	}

	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		// if (autonomousCommand != null)
		// autonomousCommand.cancel();type name = new type();
		hasBeenEnabled = true;
		Robot.states.mode = Mode.TELEOP;
		Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
		
		new DriverControlDrivetrain().start();
	}

	/**
	 * This function is called periodically during operator control
	 */
	boolean memes = false;

	public void teleopPeriodic() {

		Scheduler.getInstance().run();
		oi.update();

		/*
		 * // automatic sensor listener, put this somewhere else bc it's so
		 * ugleh if (Robot.shooterSS.hasBallSensor() && !Robot.states.hasBall
		 * //FIXME oi.drt is dirty af && Robot.states.shooterArmPositionTracker
		 * == ShooterArmPosition.DOWN && oi.dRT == 0) { new PinchBall().start();
		 * }
		 */

		/*if (pdp.getVoltage() < 9) {
			compressor.setClosedLoopControl(false);
		} else {
			compressor.setClosedLoopControl(true);
		}*/

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
