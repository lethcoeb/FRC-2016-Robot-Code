package org.usfirst.frc.team1806.robot;

import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.RumbleType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
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
import org.usfirst.frc.team1806.robot.commands.autonomous.routines.LowBarAuto;
import org.usfirst.frc.team1806.robot.commands.autotarget.LineUpShot;
import org.usfirst.frc.team1806.robot.commands.elevator.IncrementUp;
import org.usfirst.frc.team1806.robot.commands.elevator.LowBarTeleop;
import org.usfirst.frc.team1806.robot.commands.elevator.LowBarTeleopCG;
import org.usfirst.frc.team1806.robot.commands.elevator.ManualMove;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToBatterShotHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToChevalHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToGrabPosition_Deprecated;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingFromLow;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToHoldingPID_Deprecated;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToLocationPID;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToOuterworksShootingHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.TempMoveToChevalDeFunHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.TempMoveToGrabHeight;
import org.usfirst.frc.team1806.robot.commands.elevator.MoveToShootingHeight_Deprecated;
import org.usfirst.frc.team1806.robot.commands.elevator.ResetElevator;
import org.usfirst.frc.team1806.robot.commands.elevator.SetStateHolding;
import org.usfirst.frc.team1806.robot.commands.elevator.SetStateShooting;
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
	boolean dB, dY, dRB, dLB, dStart, dPOVUp, dPOVDown, dPOVLeft, dPOVRight;
	public boolean dBack, dA, dX;

	double olsY, orsY, oRT, oLT;
	boolean oA, oB, oX, oY, oRB, oLB, oStart, oBack, oRsClick;
	public boolean oPOVUp, oPOVDown;

	Latch intakeDeployLatch, moveShooterLatch, elevatorManualAutoLatch, cockingRequestLatch,
			chevalDeFunLatch, elevatorLowBarModeLatch, incrementLatch, testLatch, eggIntakeLatch, latch1, latch2, latch3, latch4;
	public static Latch lowBarLatch, shootBallLatch;

	final double kJoystickDeadzone = .15;

	public OperatorInterface() {

		//Init driver and operator controllers
		dc = new XboxController(0);
		oc = new XboxController(1);

		intakeDeployLatch = new Latch();
		moveShooterLatch = new Latch();
		shootBallLatch = new Latch();
		elevatorManualAutoLatch = new Latch();
		cockingRequestLatch = new Latch();
		chevalDeFunLatch = new Latch();
		elevatorLowBarModeLatch = new Latch();
		incrementLatch = new Latch();
		testLatch = new Latch();
		lowBarLatch = new Latch();
		eggIntakeLatch = new Latch();
		latch1 = new Latch();
		latch2 = new Latch();
		latch3 = new Latch();
		latch4 = new Latch();

		m_commands = new Commands();

	}
	
	//The update function is what should be called in the teleopPeriodic function
	//It updates the contoller's inputs, then sets the command object's enums accordingly,
	//then executes the commands if they're valid requests
	public void update() {

		updateInputs();
		executeCommands(setCommands());

	}

	private Commands setCommands() {

		// TODO make this a command
		if (intakeDeployLatch.update(dY) || eggIntakeLatch.update(oLB)) {
			// switch intake deployment
			
			System.out.println("Intake Deployed: " + (Robot.states.intakePositionTracker == IntakePosition.DEPLOYED));
			System.out.println("Moving Down? : " + Robot.elevatorSS.isMovingDown());
			System.out.println("High enough?: " + (Robot.elevatorSS.getElevatorPosition() >= (Constants.elevatorChevaldeFunHeight - 10000)));
			
			if (Robot.states.intakePositionTracker == IntakePosition.DEPLOYED) {
				//Disallow retracting of the intake if the elevator is moving downwards so we dont break stuff
				if(!Robot.elevatorSS.isMovingDown() && ((Robot.elevatorSS.getElevatorPosition() >= (Constants.elevatorChevaldeFunHeight - 10000)) || Robot.states.lowBarring)){
					new RaiseIntake().start();
				}
			} else if (Robot.states.intakePositionTracker == IntakePosition.RETRACTED) {
				//It should always be safe to lower the intake
				new LowerIntake().start();
			}
		}

		if (chevalDeFunLatch.update(oPOVUp)) {
			//m_commands.armDefenseCommandTracker = ArmDefenseCommand.CHEVALDEFUN;
			m_commands.armDefenseCommandTracker = ArmDefenseCommand.NONE;
			//System.out.println("chevaldefun");
		} else if (elevatorLowBarModeLatch.update(oPOVDown)) {
			//m_commands.armDefenseCommandTracker = ArmDefenseCommand.LOWBAR;
			m_commands.armDefenseCommandTracker = ArmDefenseCommand.NONE;
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

		if(dStart){
			Robot.states.pulsePower = null;
		}
		
		if (dRT >= .6) {
			m_commands.autoLineUp = true;
		} else {
			m_commands.autoLineUp = false;
		}

		//what position do you wanna move the arm to
		if (latch1.update(oA)) {
			m_commands.elevatorPositionRequestTracker = ElevatorPositionRequest.HOLDING;
		} else if (latch2.update(oB)) {
			m_commands.elevatorPositionRequestTracker = ElevatorPositionRequest.OUTERWORKS;
		}else if (latch3.update(oX)) {
			m_commands.elevatorPositionRequestTracker = ElevatorPositionRequest.BATTER;
		}else if (latch4.update(oY)) {
			m_commands.elevatorPositionRequestTracker = ElevatorPositionRequest.CHEVAL;
		}else {
			m_commands.elevatorPositionRequestTracker = ElevatorPositionRequest.NONE;
		}

		if (shootBallLatch.update(dX)) {
			m_commands.shootRequestTracker = ShootRequest.SHOOT;
		} else {
			m_commands.shootRequestTracker = ShootRequest.NONE;
		}

		/*if (elevatorManualAutoLatch.update(oc.getButtonRB())) {
			if (Robot.states.elevatorOperatorControlModeTracker == ElevatorOperatorControlMode.AUTO) {
				m_commands.elevatorControlModeTracker = ElevatorControlMode.MANUAL;
			} else {
				m_commands.elevatorControlModeTracker = ElevatorControlMode.AUTO;
			}
		} else {
			m_commands.elevatorControlModeTracker = ElevatorControlMode.NONE;
		}*/
		
		m_commands.elevatorControlModeTracker = ElevatorControlMode.NONE;

		if (cockingRequestLatch.update(dA)) {
			m_commands.manualCockCommandTracker = ManualCockCommand.COCK;
		} else {
			m_commands.manualCockCommandTracker = ManualCockCommand.NONE;
		}

		if (incrementLatch.update(oRB)) {
			System.out.println("incrementing up");
			new IncrementUp(125).start();
		}

		if (lowBarLatch.update(oRT > .5)) {
			// Low bar?

			if (Robot.states.intakePositionTracker == IntakePosition.DEPLOYED
					&& Robot.states.shooterArmPositionTracker != RobotStates.ShooterArmPosition.UP
					&& !Robot.states.lowBarring) {
				new LowBarTeleopCG().start();
			} else if (Robot.states.intakePositionTracker == IntakePosition.DEPLOYED && Robot.states.lowBarring) {
				Robot.states.lowBarring = false;
				new MoveToHoldingFromLow(Robot.states.hasBall).start();
			}

		}

		return m_commands;
	}

	private void executeCommands(Commands c) {

		// TODO clean dis
		if (oBack) {
			Robot.shooterSS.releaseBall();
			Robot.states.hasBall = false;
		} else if (oStart) {
			Robot.shooterSS.pinchBall();
			Robot.states.hasBall = true;
		}

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
		
		//Start a new lineup command if the driver wants to and one isn't currently running
		if (c.autoLineUp == true && Robot.states.autoLiningUp == false) {
			Robot.states.autoLiningUp = true;
			System.out.println("Starting autolineup");
			new LineUpShot().start();
			Robot.states.initialTarget = Robot.jr.getAngleToGoal();
			Robot.states.loopsUntilDone = 0;
		}

		if (/*
			 * Robot.states.intakeControlModeTracker == IntakeControlMode.DRIVER
			 */true) {
			if (m_commands.intakeCommandTracker == RunIntakeCommand.INTAKE && !Robot.states.hasBall
					&& Robot.states.intakeRollerStateTracker != RobotStates.IntakeRollerState.INTAKING
					&& !Robot.states.collectingBalling && Robot.states.intakePositionTracker == IntakePosition.DEPLOYED) {
				if (Robot.elevatorSS.getElevatorSetpoint() <= Constants.elevatorChevaldeFunHeight - 10000
						&& Robot.states.shooterCockedTracker == ShooterCocked.COCKED) {
					// TODO is there a better way to do this other than reading
					// the setpoint?
					new CollectBall().start();
					Robot.states.intakeRollerStateTracker = IntakeRollerState.INTAKING;
				} else if (Robot.states.shooterCockedTracker == ShooterCocked.NOTCOCKED && !Robot.shooterSS.isCocking() && !Robot.states.overCocked) {
					new ManualCock().start();
				}

			} else if (m_commands.intakeCommandTracker == RunIntakeCommand.OUTTAKE) {
				if (Robot.states.intakeRollerStateTracker != RobotStates.IntakeRollerState.OUTTAKING
						&& (Robot.elevatorSS.getElevatorPosition() <= Constants.elevatorHoldingHeight + 4000
								|| Robot.states.shooterArmPositionTracker == ShooterArmPosition.DOWN
								|| Robot.states.shooterArmPositionTracker == ShooterArmPosition.HOLDING)) {
					// can only release ball when it's close to the ground
					System.out.println("spitout started");
					new SpitOutBall().start();
				}
			} else if (m_commands.intakeCommandTracker == RunIntakeCommand.STOP
					&& Robot.states.intakeRollerStateTracker != IntakeRollerState.STOPPED) {
				// Robot.intakeSS.stopIntaking();
				// Robot.states.intakeRollerStateTracker =
				// IntakeRollerState.STOPPED;
			}
		}

		// TODO clean this up
		if (m_commands.intakeCommandTracker == RunIntakeCommand.STOP) {
			/*
			 * if(Math.abs(orsY) > .2){ Robot.intakeSS.runAtSpeed(orsY); }else{
			 * Robot.intakeSS.stopIntaking(); }
			 */
		}

		if (c.armDefenseCommandTracker == ArmDefenseCommand.NONE) {

		} else if (c.armDefenseCommandTracker == ArmDefenseCommand.CHEVALDEFUN) {
			if (c.intakeCommandTracker == RunIntakeCommand.STOP) {
				//new TempMoveToChevalDeFunHeight().start();
				System.out.println("moving to temp chevaldefun");
			}
		} else if (c.armDefenseCommandTracker == ArmDefenseCommand.LOWBAR) {
			if (c.intakeCommandTracker == RunIntakeCommand.STOP) {
				//new TempMoveToGrabHeight().start();
				System.out.println("moving to temp grab height");
			}
		}

		if (c.elevatorControlModeTracker == ElevatorControlMode.AUTO) {
			Robot.elevatorSS.elevatorStopMovement();
			Robot.elevatorSS.elevatorSetControlMode(TalonControlMode.Position);
			Robot.states.elevatorOperatorControlModeTracker = ElevatorOperatorControlMode.AUTO;
		} else if (c.elevatorControlModeTracker == ElevatorControlMode.MANUAL
				&& !(Robot.states.elevatorOperatorControlModeTracker == RobotStates.ElevatorOperatorControlMode.MANUAL)) {
			Robot.states.elevatorOperatorControlModeTracker = ElevatorOperatorControlMode.MANUAL;
		}

		/*if (c.elevatorPositionRequestTracker == ElevatorPositionRequest.SWITCH
				&& Robot.states.elevatorOperatorControlModeTracker == ElevatorOperatorControlMode.AUTO) {
			if (Robot.states.shooterArmPositionTracker == ShooterArmPosition.HOLDING) {
				new MoveToShootingHeight().start();
			} else if (Robot.states.shooterArmPositionTracker == ShooterArmPosition.UP) {
				if (Robot.states.intakePositionTracker == IntakePosition.DEPLOYED) {
					new MoveToHoldingPID().start();
				}
			} else if (Robot.elevatorSS.elevatorGetPowerOutput() < 0) {
				new MoveToShootingHeight().start();
			} else if (Robot.elevatorSS.elevatorGetPowerOutput() >= 0 && !Robot.states.lowBarring) {
				// You're moving up so go to holding to interrupt. also this
				// allows to get out of any weird positions or states by sending elevator to
				// holding if it isn't moving
				new MoveToHoldingPID().start();
			}

		} else if (Robot.states.elevatorOperatorControlModeTracker == ElevatorOperatorControlMode.MANUAL) {
			if (Math.abs(olsY) > .15) {
				Robot.elevatorSS.elevatorMoveAtSpeed(olsY);
			} else {
				Robot.elevatorSS.elevatorStopMovement();
			}
		}*/
		
		if(c.elevatorPositionRequestTracker != ElevatorPositionRequest.NONE){
		
		if(Robot.states.intakePositionTracker == IntakePosition.DEPLOYED){
		
		if(c.elevatorPositionRequestTracker == ElevatorPositionRequest.HOLDING){
			new MoveToHoldingPID().start();
		}else if(c.elevatorPositionRequestTracker == ElevatorPositionRequest.OUTERWORKS){
			new MoveToOuterworksShootingHeight().start();
		}else if(c.elevatorPositionRequestTracker == ElevatorPositionRequest.BATTER){
			new MoveToBatterShotHeight().start();
		}else if(c.elevatorPositionRequestTracker == ElevatorPositionRequest.CHEVAL){
			new MoveToChevalHeight().start();
		}
		
		}
		}
		

		if (m_commands.shootRequestTracker == ShootRequest.SHOOT && Robot.states.hasBall
				&& Robot.states.shooterArmPositionTracker == ShooterArmPosition.UP
				&& Robot.states.shooterCockedTracker == ShooterCocked.COCKED && dRT < .3) {

			new ShootThenCock().start();
		}

		if (m_commands.manualCockCommandTracker == ManualCockCommand.COCK && !Robot.shooterSS.isShooterCocked()) {
			new ManualCock().start();
		} else if (m_commands.manualCockCommandTracker == ManualCockCommand.COCK && Robot.shooterSS.isShooterCocked()) {
			new ShootDaBall().start();
		}

		// FIXME THIS
		if (dBack) {
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

		// operator buttons
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

	//Stop all controller rumbles because the computer doesn't do it even if the robot is disabled :(
	public void stopRumbles() {
		dc.setRumble(RumbleType.kLeftRumble, 0);
		dc.setRumble(RumbleType.kRightRumble, 0);
		oc.setRumble(RumbleType.kLeftRumble, 0);
		oc.setRumble(RumbleType.kRightRumble, 0);
	}

}
