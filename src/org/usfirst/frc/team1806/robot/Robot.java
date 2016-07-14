
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
import org.usfirst.frc.team1806.robot.RobotStates.IntakePosition;
import org.usfirst.frc.team1806.robot.RobotStates.Mode;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;
import org.usfirst.frc.team1806.robot.commands.DriverControlDrivetrain;
import org.usfirst.frc.team1806.robot.commands.RumbleController;
import org.usfirst.frc.team1806.robot.commands.Wait;
import org.usfirst.frc.team1806.robot.commands.autonomous.DoNothing;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveUntilFlat;
import org.usfirst.frc.team1806.robot.commands.autonomous.FourteenInchMode;
import org.usfirst.frc.team1806.robot.commands.autonomous.RobotReset;
import org.usfirst.frc.team1806.robot.commands.autonomous.Turn90;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAbsoluteAngle;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.BackwardsDrivingAuto;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.ChevalDeFriseAuto;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.DoNothingAuto;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.DoSomething;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.ForwardsDrivingAuto;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.ForwardsOverDefense;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.ForwardsOverPos2or5Defense;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.LowBarAuto;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.OneBallNoSteal;
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.OneBallSteal;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID_Deprecated;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToOuterworksShootingHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight_Deprecated;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;
import org.usfirst.frc.team1806.robot.commands.shooter.CockShooter;
import org.usfirst.frc.team1806.robot.commands.shooter.ManualCock;
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
	public static SensorSuite ss;

	public static SendableChooser autonomous;
	public static SendableChooser autoArmUpOrDown;
	public static SendableChooser autoForwardOrBackward;
	public static SendableChooser autoShoot;
	public static SendableChooser autoLane;
	public static SendableChooser defenseToBeCrossed;

	boolean hasBeenEnabled = false;

	public enum Defenses {
		LOWBAR, ROUGHTERRAIN, ROCKWALL, MOAT, RAMPARTS, PORTCULLIS, CHEVALDEFRISE, DRAWBRIDGE, SALLYPORT
	}

	public static double getPDPResistance(int channel) {
		return pdp.getVoltage() / pdp.getCurrent(channel);
	}

	public void robotInit() {
		
		//BEEP BOOP BEEP BOOP!
		//INITIALIZE ROBOT SUBSYSTEMS!
		drivetrainSS = new DrivetrainSubsystem();
		elevatorSS = new ElevatorSubsystem();
		intakeSS = new IntakeSubsystem();
		shooterSS = new ShooterSubsystem();
		pdp = new PowerDistributionPanel();
		compressor = new Compressor();
		
		//INITIALIZE OPERATOR INTERFACE AND STATE TRACKER!
		oi = new OperatorInterface();
		states = new RobotStates();

		if (elevatorSS.isBottomLimitHit()) {
			Robot.elevatorSS.resetElevatorEncoder();
			states.shooterArmPositionTracker = ShooterArmPosition.DOWN;// to
																		// speed
																		// up
																		// testing
		}
		
		
		
		// Vision tracking data receiver. Was used for jetson coprocessor, now used for receiving vals from DS over NetworkTables
		jr = new JetsonReceiver();
		jr.start();
		
		ss = new SensorSuite();
		// autoreader for future events

		//ar = new AutonomousReader();
		//ar.start();

		// Temp QaDAS. Make sure all object in same chooser are of same type.

		sdu = new SmartDashboardUpdater();

		SmartDashboard.putNumber("PulsePower", .20);
		SmartDashboard.putNumber("PulseWidth", .01);
		
		//Init SendableChoosers for autonomous selection
		autonomous = new SendableChooser();
		autoArmUpOrDown = new SendableChooser();
		autoForwardOrBackward = new SendableChooser();
		autoShoot = new SendableChooser();
		autoLane = new SendableChooser();
		defenseToBeCrossed = new SendableChooser();

		autonomous.addDefault("Run Auto: No", "N");
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
		SmartDashboard.putData("Defense Position? (Selecting Low bar will override any arm settings)", autoLane);

		defenseToBeCrossed.addDefault("Rock Wall", Defenses.ROCKWALL);
		defenseToBeCrossed.addObject("Low Bar", Defenses.LOWBAR);
		defenseToBeCrossed.addObject("Rough Terrain", Defenses.ROUGHTERRAIN);
		defenseToBeCrossed.addObject("Drawbridge", Defenses.DRAWBRIDGE);
		defenseToBeCrossed.addObject("Sally Port", Defenses.SALLYPORT);
		defenseToBeCrossed.addObject("Moat", Defenses.MOAT);
		defenseToBeCrossed.addObject("Ramparts", Defenses.RAMPARTS);
		defenseToBeCrossed.addObject("Portcullis", Defenses.PORTCULLIS);
		defenseToBeCrossed.addObject("Cheval De Frise", Defenses.CHEVALDEFRISE);
		SmartDashboard.putData("Defense to be crossed", defenseToBeCrossed);
		
		
		
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */

	public void disabledInit() {
		
		drivetrainSS.shiftLow();
		
		oi.stopRumbles();
	}

	public void disabledPeriodic() {

		Scheduler.getInstance().run();
		sdu.push();
		ss.update();

		if (!hasBeenEnabled) {
			Robot.elevatorSS.elevatorSetPosition(-101100);
			//Robot.elevatorSS.elevatorSetPosition(-Constants.elevatorShootingHeight);
		}
		
		if ((String) autonomous.getSelected() == "N") {
			SmartDashboard.putString("Auto running?", "N");
		}else{
			SmartDashboard.putString("Auto running?", "Y");
		}
		
		// System.out.println("Get POV: " + oi.oc.getPOV());
		// System.out.println("Get POV Count: " + oi.oc.getPOVCount());

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
		
		//Shift robot to low gear and zero out sensors
		Robot.drivetrainSS.shiftLow();
		Robot.drivetrainSS.resetYaw();
		Robot.drivetrainSS.resetEncoders();
		
		int lane = (int) autoLane.getSelected();
		Defenses defense = (Defenses) defenseToBeCrossed.getSelected();
		
		autonomousCommand = new CommandGroup();

		//Assign autonomous command group based on driver-selected sendable choosers
		if (autonomous.getSelected() == "N") {
			autonomousCommand = new RobotReset();
		} else {

			if(defense == Defenses.LOWBAR || lane == 1){
				autonomousCommand.addSequential(new LowBarAuto());
			}else if (defense == Defenses.CHEVALDEFRISE) {
				autonomousCommand.addSequential(
						new ChevalDeFriseAuto(lane));
			}else if(defense == Defenses.MOAT || defense == Defenses.RAMPARTS || defense == Defenses.ROCKWALL || defense == Defenses.ROUGHTERRAIN){
				if(lane == 5 || lane == 2){
					//Run 2/5 over defense auto
					autonomousCommand.addSequential(new ForwardsOverPos2or5Defense(lane));
				}else{
					//Run 3/4 over defense auto
					autonomousCommand.addSequential(new ForwardsOverDefense(lane));
				}
			}

			/*else if (autoForwardOrBackward.getSelected() == "F") {
				autonomousCommand = new ForwardsDrivingAuto((boolean) autoArmUpOrDown.getSelected(),
						(boolean) autoShoot.getSelected(), (int) autoLane.getSelected());
			} else {
				autonomousCommand = new BackwardsDrivingAuto((boolean) autoArmUpOrDown.getSelected(),
						(boolean) autoShoot.getSelected(), (int) autoLane.getSelected());
			}*/
		}
		
		//Start assigned command
		//autonomousCommand = new ForwardsOverDefense((int) autoLane.getSelected());
		autonomousCommand.start();		
		
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch (autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */
		/*
		 * autonomousCommand = new FourteenInchMode();
		 * 
		 * ar.stopThread();
		 * 
		 * ArrayList<String> commandStrings = new ArrayList<String>(); if
		 * (ar.CommandsReceived()) { commandStrings =
		 * ar.getCommandStringArray(); }
		 * 
		 * // You got a routine boy, now parse it if (commandStrings.get(0) ==
		 * "OneBall") { if (commandStrings.get(1) == "Steal") { new
		 * OneBallSteal(commandStrings); } else if (commandStrings.get(1) ==
		 * "NoSteal") { new OneBallNoSteal(commandStrings); } } else if
		 * (commandStrings.get(0) == "TwoBall") { // run 2 ball auto //
		 * literally a dream }
		 * 
		 * // schedule the autonomous command (example) if (autonomousCommand !=
		 * null) autonomousCommand.start(); }
		 * 
		 * /** This function is called periodically during autonomous
		 */
	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		sdu.push();
		ss.update();
		
		//Stop autonomous command if the robot is tilted too far forwards or backwards
		//Basically avoid pulling a 935 hehehe
		if (Math.abs(Robot.drivetrainSS.getRoll()) > 65) {
			autonomousCommand.cancel();
		}
		
		Robot.shooterSS.unLit();

	}

	public void teleopInit() {
		
		//Cancel any auto command if it's still running
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		
		//Sets tracking states that we use for some stuff
		hasBeenEnabled = true;
		Robot.states.mode = Mode.TELEOP;
		Robot.states.driveControlModeTracker = DriveControlMode.DRIVER;
		
		//I don't know why but this fixed a bug where we couldn't autoline up if the auto didn't finish
		Robot.states.autoLiningUp = false;
		
		//Recock shooter and lower arm automatically since sometimes the auto doesn't run all the way through
		if(!Robot.shooterSS.isShooterCocked()){
			new ManualCock().start();
		}
		
		
		/*if(states.shooterArmPositionTracker != states.shooterArmPositionTracker.HOLDING && !states.hasBall && states.intakePositionTracker == IntakePosition.DEPLOYED){
			new MoveToHoldingPID().start();
		}*/
		
		//Give manual control of drivetrain to driver
		new DriverControlDrivetrain().start();
	}

	/**
	 * This function is called periodically during operator control
	 */

	public void teleopPeriodic() {

		Scheduler.getInstance().run();
		oi.update();
		ss.update();
		
		//Quick and dirty override to make sure the driver always has control of the drivetrain if not using autoalignment
		if(states.autoLiningUp){
			if(Robot.oi.dRT < .4){
				states.autoLiningUp = false;
				Robot.states.autoalignmentShooting = false;
				System.out.println("Overriding autolineup, returning control to driver.");
				new DriverControlDrivetrain().start();
				compressor.setClosedLoopControl(true);
				compressor.start();
			}
		}
		
		if(Robot.elevatorSS.getElevatorPosition() >= Constants.elevatorShootingHeight - 10000 && Robot.oi.dc.getRightTrigger() < .3 && Robot.states.hasBall && !Robot.oi.dc.getButtonLS()){
			//high enough to get lit
			Robot.shooterSS.getLit();
		}else{
			Robot.shooterSS.unLit();
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
