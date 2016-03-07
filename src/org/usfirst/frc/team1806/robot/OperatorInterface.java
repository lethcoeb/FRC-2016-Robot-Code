package org.usfirst.frc.team1806.robot;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.RumbleType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Scheduler;
import util.Latch;
import util.XboxController;

import org.usfirst.frc.team1806.robot.Commands.ElevatorControlMode;
import org.usfirst.frc.team1806.robot.Commands.ElevatorPositionRequest;
import org.usfirst.frc.team1806.robot.Commands.ManualCockCommand;
import org.usfirst.frc.team1806.robot.Commands.RunIntakeCommand;
import org.usfirst.frc.team1806.robot.Commands.ShiftRequest;
import org.usfirst.frc.team1806.robot.Commands.ShootRequest;
import org.usfirst.frc.team1806.robot.RobotStates.DriveControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.DrivetrainGear;
import org.usfirst.frc.team1806.robot.RobotStates.ElevatorOperatorControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.IntakeControlMode;
import org.usfirst.frc.team1806.robot.RobotStates.IntakePosition;
import org.usfirst.frc.team1806.robot.RobotStates.IntakeRollerState;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterArmPosition;
import org.usfirst.frc.team1806.robot.RobotStates.ShooterCocked;
import org.usfirst.frc.team1806.robot.commands.autonomous.TurnToAngle;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToGrabPosition;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingFromLow;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.ResetElevator;
import org.usfirst.frc.team1806.robot.commands.intake.CollectBall;
import org.usfirst.frc.team1806.robot.commands.intake.LowerIntake;
import org.usfirst.frc.team1806.robot.commands.intake.RaiseIntake;
import org.usfirst.frc.team1806.robot.commands.intake.SpitOutBall;
import org.usfirst.frc.team1806.robot.commands.shooter.CockShooter;
import org.usfirst.frc.team1806.robot.commands.shooter.ReleaseBall;
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
	boolean dA, dB, dX, dY, dRB, dLB, dStart, dBack;

	double orsY;
	boolean oA, oRsClick, oStart;

	Latch intakeDeployLatch, moveShooterLatch, shootBallLatch, elevatorManualAutoLatch, cockingRequestLatch;

	final double kJoystickDeadzone = .15;

	//TODO remove this
	Joystick j;
	JoystickButton a;
	
	public OperatorInterface() {

		dc = new XboxController(0);
		oc = new XboxController(1);

		intakeDeployLatch = new Latch();
		moveShooterLatch = new Latch();
		shootBallLatch = new Latch();
		elevatorManualAutoLatch = new Latch();
		cockingRequestLatch = new Latch();

		m_commands = new Commands();
		
		//TODO remove this
		j = new Joystick(0);
		a = new JoystickButton(j, 1);
		a.whenPressed(new TurnToAngle(90));

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

		//TODO fix dis
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
		
		if(dRT >= .6){
			m_commands.autoLineUp = true;
		}else{
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
		}else{
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
				Robot.states.drivetrainGearTracker = DrivetrainGear.HIGH;
			} else if (c.shiftRequestCommandTracker == ShiftRequest.LOW) {
				Robot.drivetrainSS.shiftLow();
				Robot.states.drivetrainGearTracker = DrivetrainGear.LOW;
			}
		}
		
		if(c.autoLineUp == true && Robot.states.autoLiningUp == false){
			new LineUpShot().start();
		}

		if (/*Robot.states.intakeControlModeTracker == IntakeControlMode.DRIVER*/true) {
			if (m_commands.intakeCommandTracker == RunIntakeCommand.INTAKE && !Robot.states.hasBall && Robot.states.intakeRollerStateTracker != RobotStates.IntakeRollerState.INTAKING) {
				if (Robot.elevatorSS.getElevatorSetpoint() != Constants.elevatorShootingHeight) {
					//TODO is there a better way to do this other than reading the setpoint?
					new CollectBall().start();
				} else {
					// somewhere in between holding and grabbing, or maybe the
					// arm is up. do nothing
				}
				Robot.states.intakeRollerStateTracker = IntakeRollerState.INTAKING;
			} else if (m_commands.intakeCommandTracker == RunIntakeCommand.OUTTAKE) {
				if (Robot.states.intakeRollerStateTracker != RobotStates.IntakeRollerState.OUTTAKING && (Robot.elevatorSS.getElevatorPosition() < 8000 || Robot.states.shooterArmPositionTracker == ShooterArmPosition.DOWN
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
				Robot.elevatorSS.elevatorMoveAtSpeed(orsY);
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
			new CockShooter().start();
		}else if(m_commands.manualCockCommandTracker == ManualCockCommand.COCK && Robot.shooterSS.shooterIsCocked()){
			new ShootDaBall().start();
		}

		// FIXME THIS
		if (dBack && Robot.states.elevatorOperatorControlModeTracker != RobotStates.ElevatorOperatorControlMode.RESET) {
			System.out.println("reseting elevator");
			new ResetElevator().start();
		}
	}

	private void updateInputs() {

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

		orsY = oc.getRightJoyY();

		oA = oc.getButtonA();
		oRsClick = oc.getButtonRS();
		oStart = oc.getButtonStart();

	}
	
	public void stopRumbles(){
		dc.setRumble(RumbleType.kLeftRumble, 0);
		dc.setRumble(RumbleType.kRightRumble, 0);
		oc.setRumble(RumbleType.kLeftRumble, 0);
		oc.setRumble(RumbleType.kRightRumble, 0);
	}
	
}
