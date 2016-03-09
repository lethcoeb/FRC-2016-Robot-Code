package org.usfirst.frc.team1806.robot;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.RumbleType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Scheduler;
import util.Latch;
import util.XboxController;

import org.usfirst.frc.team1806.robot.Commands.ArmDefenseCommand;
import org.usfirst.frc.team1806.robot.Commands.ElevatorControlMode;
import org.usfirst.frc.team1806.robot.Commands.ElevatorPositionRequest;
import org.usfirst.frc.team1806.robot.Commands.ManualCockCommand;
import org.usfirst.frc.team1806.robot.Commands.RunIntakeCommand;
import org.usfirst.frc.team1806.robot.Commands.ShiftRequest;
import org.usfirst.frc.team1806.robot.Commands.ShootRequest;
import org.usfirst.frc.team1806.robot.RobotStates.DriveControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.DrivetrainGear;
import org.usfirst.frc.team1806.robot.RobotStates.ElevatorOperatorControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.IntakePosition;
import org.usfirst.frc.team1806.robot.RobotStates.IntakeRollerState;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterCocked;
import org.usfirst.frc.team1806.robot.commands.ResetNavx;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveOverAndTurn;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveToPosition;
import org.usfirst.frc.team1806.robot.commands.autonomous.DriveUntilFlat;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAbsoluteAngle;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAngle;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToGrabPosition;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID;
import org.usfirst.frc.team1806.robot.commands.elevator.TempMoveToChevalDeFunHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.TempMoveToGrabHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.ResetElevator;
import org.usfirst.frc.team1806.robot.commands.intake.CollectBall;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;
import org.usfirst.frc.team1806.robot.commands.intake.RaiseIntake;
import org.usfirst.frc.team1806.robot.commands.intake.SpitOutBall;
import org.usfirst.frc.team1806.robot.commands.shooter.CockShooter;
import org.usfirst.frc.team1806.robot.commands.shooter.ManualCock;
import org.usfirst.frc.team1806.robot.commands.shooter.ShootDaBall;
import org.usfirst.frc.team1806.robot.commands.shooter.ShootThenCock;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OperatorInterface {

	public XboxController dc;
	public static XboxController oc;
	Commands m_commands;

	// d for driver ;)
	double dlsY, drsX, dRT, dLT;
	boolean dA, dB, dX, dY, dRB, dLB, dStart, dBack, dPOVUp, dPOVDown, dPOVLeft, dPOVRight;

	double olsY, orsY, oRT, oLT;
	boolean oA, oB, oX, oY, oRB, oLB, oStart, oBack, oRsClick, oPOVUp, oPOVDown;

	Latch intakeDeployLatch, moveShooterLatch, shootBallLatch, elevatorManualAutoLatch, cockingRequestLatch,
			chevalDeFunLatch, elevatorLowBarModeLatch;

	final double kJoystickDeadzone = .15;

	// TODO remove this
	Joystick j;
	JoystickButton a;
	JoystickButton b;

	public OperatorInterface() {

		dc = new XboxController(0);
		oc = new XboxController(1);

		intakeDeployLatch = new Latch();
		moveShooterLatch = new Latch();
		shootBallLatch = new Latch();
		elevatorManualAutoLatch = new Latch();
		cockingRequestLatch = new Latch();
		chevalDeFunLatch = new Latch();
		elevatorLowBarModeLatch = new Latch();

		m_commands = new Commands();

		// TODO remove this
		j = new Joystick(0);
		a = new JoystickButton(j, 1);
		//a.whenPressed(new DriveOverAndTurn());
		a.whenPressed(new DriveUntilFlat(-.6));
		b = new JoystickButton(j, 2);
		b.whenPressed(new ResetNavx());

	}

	public void update() {

		updateInputs();

		// drivetrain is separate because it's so important :-)
		if (Robot.states.driveControlModeTracker == DriveControlMode.DRIVER) {
			if (Math.abs(dlsY) > kJoystickDeadzone && Math.abs(drsX) > kJoystickDeadzone) {
				Robot.drivetrainSS.execute(dlsY, -drsX);
			} else if (Math.abs(dlsY) > kJoystickDeadzone) {
				Robot.drivetrainSS.execute(dlsY, 0);
			} else if (Math.abs(drsX) > kJoystickDeadzone) {
				Robot.drivetrainSS.execute(0, -drsX);
			} else {
				Robot.drivetrainSS.execute(0, 0);
			}
		} else {
			// Automatic driving stuff
		}

		executeCommands(setCommands());

	}

	private Commands setCommands() {

		// TODO make this a command
		if (intakeDeployLatch.update(dY)) {
			// switch intake deployment
			if (Robot.states.intakePositionTracker == IntakePosition.DEPLOYED) {
				// Robot.intakeSS.retractIntake();
				new RaiseIntake().start();
			} else if (Robot.states.intakePositionTracker == IntakePosition.RETRACTED) {
				// Robot.intakeSS.deployIntake();
				new LowerIntake().start();
			}
		}

		if (chevalDeFunLatch.update(oPOVUp)) {
			m_commands.armDefenseCommandTracker = ArmDefenseCommand.CHEVALDEFUN;
			System.out.println("chevaldefun");
		} else if (elevatorLowBarModeLatch.update(oPOVDown)) {
			m_commands.armDefenseCommandTracker = ArmDefenseCommand.LOWBAR;
		} else {
			m_commands.armDefenseCommandTracker = ArmDefenseCommand.NONE;
		}

		if (dLB) {
			// suck dat ball in boi
			m_commands.intakeCommandTracker = RunIntakeCommand.INTAKE;
		} else if (dRB) {
			// spit dat ball out boi
			m_commands.intakeCommandTracker = RunIntakeCommand.OUTTAKE;

		} else {
			// leave dat ball where it be boi
			m_commands.intakeCommandTracker = RunIntakeCommand.STOP;
		}

		// TODO fix dis
		if (dc.getButtonLS()) {
			m_commands.shiftRequestCommandTracker = ShiftRequest.LOW;
			Robot.drivetrainSS.shiftLow();
		} else if (dc.getButtonRS()) {
			m_commands.shiftRequestCommandTracker = ShiftRequest.HIGH;
			Robot.drivetrainSS.shiftHigh();
		} else {
			m_commands.shiftRequestCommandTracker = ShiftRequest.NONE;
		}

		m_commands.shiftRequestCommandTracker = ShiftRequest.NONE;

		if (dRT >= .6) {
			m_commands.autoLineUp = true;
		} else {
			m_commands.autoLineUp = false;
		}

		if (moveShooterLatch.update(dB)) {
			// reposition it
			m_commands.elevatorPositionRequestTracker = ElevatorPositionRequest.SWITCH;
		} else {
			m_commands.elevatorPositionRequestTracker = ElevatorPositionRequest.NONE;
		}

		if (shootBallLatch.update(dX)) {
			m_commands.shootRequestTracker = ShootRequest.SHOOT;
		} else {
			m_commands.shootRequestTracker = ShootRequest.NONE;
		}

		if (elevatorManualAutoLatch.update(dc.getButtonRS())) {
			if (Robot.states.elevatorOperatorControlModeTracker == ElevatorOperatorControlMode.AUTO) {
				m_commands.elevatorControlModeTracker = ElevatorControlMode.MANUAL;
			} else {
				m_commands.elevatorControlModeTracker = ElevatorControlMode.AUTO;
			}
		} else {
			m_commands.elevatorControlModeTracker = ElevatorControlMode.NONE;
		}

		// FIXME again quick and dirty
		if (cockingRequestLatch.update(dStart)) {
			m_commands.manualCockCommandTracker = ManualCockCommand.COCK;
		} else {
			m_commands.manualCockCommandTracker = ManualCockCommand.NONE;
		}

		return m_commands;
	}

	private void executeCommands(Commands c) {

		if (c.shiftRequestCommandTracker == ShiftRequest.HIGH
				&& Robot.states.drivetrainGearTracker == DrivetrainGear.LOW
				|| c.shiftRequestCommandTracker == ShiftRequest.LOW
						&& Robot.states.drivetrainGearTracker == DrivetrainGear.HIGH) {
			if (c.shiftRequestCommandTracker == ShiftRequest.HIGH) {
				Robot.drivetrainSS.shiftHigh();
			} else if (c.shiftRequestCommandTracker == ShiftRequest.LOW) {
				Robot.drivetrainSS.shiftLow();
			}
		}

		if (c.autoLineUp == true && Robot.states.autoLiningUp == false) {
			new LineUpShot().start();
		}

		if (/*
			 * Robot.states.intakeControlModeTracker == IntakeControlMode.DRIVER
			 */true) {
			if (m_commands.intakeCommandTracker == RunIntakeCommand.INTAKE && !Robot.states.hasBall
					&& Robot.states.intakeRollerStateTracker != RobotStates.IntakeRollerState.INTAKING) {
				if (Robot.elevatorSS.getElevatorSetpoint() != Constants.elevatorShootingHeight) {
					// TODO is there a better way to do this other than reading
					// the setpoint?
					new CollectBall().start();
				}
				Robot.states.intakeRollerStateTracker = IntakeRollerState.INTAKING;
			} else if (m_commands.intakeCommandTracker == RunIntakeCommand.OUTTAKE) {
				if (Robot.states.intakeRollerStateTracker != RobotStates.IntakeRollerState.OUTTAKING
						&& (Robot.elevatorSS.getElevatorPosition() < 8000
								|| Robot.states.shooterArmPositionTracker == ShooterArmPosition.DOWN
								|| Robot.states.shooterArmPositionTracker == ShooterArmPosition.HOLDING)) {
					// can only release ball when it's close to the ground
					System.out.println("spitout started");
					new SpitOutBall().start();
				}
			} else if (m_commands.intakeCommandTracker == RunIntakeCommand.STOP
					&& Robot.states.intakeRollerStateTracker != IntakeRollerState.STOPPED) {
				Robot.intakeSS.stopIntaking();
				Robot.states.intakeRollerStateTracker = IntakeRollerState.STOPPED;
			}
		}
		
		//TODO clean this up
		if(m_commands.intakeCommandTracker == RunIntakeCommand.STOP && Robot.states.intakeRollerStateTracker == IntakeRollerState.STOPPED){
			if(Math.abs(orsY) > .2){
				Robot.intakeSS.runAtSpeed(orsY);
			}
		}

		if (c.armDefenseCommandTracker == ArmDefenseCommand.NONE) {

		} else if (c.armDefenseCommandTracker == ArmDefenseCommand.CHEVALDEFUN) {
			if (c.intakeCommandTracker == RunIntakeCommand.STOP
					&& Robot.elevatorSS.getElevatorSetpoint() == Constants.elevatorHoldingHeight) {
				new TempMoveToChevalDeFunHeight().start();
				System.out.println("moving to temp chevaldefun");
			}
		} else if (c.armDefenseCommandTracker == ArmDefenseCommand.LOWBAR) {
			if (c.intakeCommandTracker == RunIntakeCommand.STOP
					&& Robot.elevatorSS.getElevatorSetpoint() == Constants.elevatorHoldingHeight) {
				new TempMoveToGrabHeight().start();
				System.out.println("moving to temp grab height");
			}
		}

		if (c.elevatorControlModeTracker == ElevatorControlMode.AUTO) {
			Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
			Robot.states.elevatorOperatorControlModeTracker = ElevatorOperatorControlMode.AUTO;
		} else if (c.elevatorControlModeTracker == ElevatorControlMode.MANUAL) {
			Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.PercentVbus);
			Robot.states.elevatorOperatorControlModeTracker = ElevatorOperatorControlMode.MANUAL;
		}

		if (c.elevatorPositionRequestTracker == ElevatorPositionRequest.SWITCH
				&& Robot.states.elevatorOperatorControlModeTracker == ElevatorOperatorControlMode.AUTO) {
			if (Robot.states.shooterArmPositionTracker == ShooterArmPosition.HOLDING) {
				new MoveToShootingHeight().start();
			} else if (Robot.states.shooterArmPositionTracker == ShooterArmPosition.UP) {
				new MoveToHoldingPID().start();
			} else if (Robot.states.shooterArmPositionTracker == ShooterArmPosition.OTHER) {
				if (Robot.elevatorSS.getElevatorSetpoint() == Constants.elevatorShootingHeight) {
					new MoveToGrabPosition().start();
				} else {
					new MoveToShootingHeight().start();
				}
			}
		} else if (Robot.states.elevatorOperatorControlModeTracker == ElevatorOperatorControlMode.MANUAL) {
			if (orsY > kJoystickDeadzone) {
				Robot.elevatorSS.elevatorMoveAtSpeed(olsY);
			} else {
				Robot.elevatorSS.elevatorMoveAtSpeed(0);
			}
		}

		if (m_commands.shootRequestTracker == ShootRequest.SHOOT && Robot.states.hasBall
				&& Robot.states.shooterArmPositionTracker == ShooterArmPosition.UP
				&& Robot.states.shooterCockedTracker == ShooterCocked.COCKED) {

			new ShootThenCock().start();
		}

		if (m_commands.manualCockCommandTracker == ManualCockCommand.COCK && !Robot.shooterSS.shooterIsCocked()) {
			new ManualCock().start();
		} else if (m_commands.manualCockCommandTracker == ManualCockCommand.COCK && Robot.shooterSS.shooterIsCocked()) {
			new ShootDaBall().start();
		}

		// FIXME THIS
		if (dBack && Robot.states.elevatorOperatorControlModeTracker != RobotStates.ElevatorOperatorControlMode.RESET) {
			System.out.println("reseting elevator");
			new ResetElevator().start();
		}
	}

	private void updateInputs() {

		// driver buttons
		dlsY = dc.getLeftJoyY();
		drsX = dc.getRightJoyX();
		dRT = dc.getRightTrigger();
		dLT = dc.getLeftTrigger();

		dA = dc.getButtonA();
		dB = dc.getButtonB();
		dX = dc.getButtonX();
		dY = dc.getButtonY();
		dRB = dc.getButtonRB();
		dLB = dc.getButtonLB();
		dStart = dc.getButtonStart();
		dBack = dc.getButtonBack();
		dPOVUp = dc.getPOVUp();
		dPOVDown = dc.getPOVDown();
		dPOVLeft = dc.getPOVLeft();
		dPOVRight = dc.getPOVRight();

		// op buttons
		olsY = oc.getLeftJoyY();
		orsY = oc.getRightJoyY();
		oRT = oc.getRightTrigger();
		oLT = oc.getLeftTrigger();

		oA = oc.getButtonA();
		oB = oc.getButtonB();
		oX = oc.getButtonX();
		oY = oc.getButtonY();
		oRB = oc.getButtonRB();
		oLB = oc.getButtonLB();
		oStart = oc.getButtonStart();
		oBack = oc.getButtonBack();
		oRsClick = oc.getButtonRS();
		oPOVUp = oc.getPOVUp();
		oPOVDown = oc.getPOVDown();

	}

	public void stopRumbles() {
		dc.setRumble(RumbleType.kLeftRumble, 0);
		dc.setRumble(RumbleType.kRightRumble, 0);
		oc.setRumble(RumbleType.kLeftRumble, 0);
		oc.setRumble(RumbleType.kRightRumble, 0);
	}

}
