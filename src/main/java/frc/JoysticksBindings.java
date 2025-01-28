package frc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.constants.field.enums.ReefSide;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.states.DriveRelative;
import frc.robot.subsystems.swerve.states.RotateAxis;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.robot.superstructure.RobotState;
import frc.robot.superstructure.Timeouts;
import frc.robot.superstructure.Tolerances;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.subsystems.swerve.Swerve;
import frc.utils.time.TimeUtils;

public class JoysticksBindings {

	private static final double NOTE_IN_RUMBLE_POWER = 0.5;
	private static final double TIME_BETWEEN_RUMBLE_SECONDS = 0.5;

	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN);
	private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(JoystickPorts.SECOND);
	private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(JoystickPorts.THIRD);
	public static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(JoystickPorts.FOURTH);
	private static final SmartJoystick FIFTH_JOYSTICK = new SmartJoystick(JoystickPorts.FIFTH);
	private static final SmartJoystick SIXTH_JOYSTICK = new SmartJoystick(JoystickPorts.SIXTH);

	private static double lastTimeRumbled = -1;

	private static boolean isTimeToRumble() {
		return TimeUtils.getCurrentTimeSeconds() - lastTimeRumbled > TIME_BETWEEN_RUMBLE_SECONDS;
	}

	public static void configureBindings(Robot robot) {
		mainJoystickButtons(robot);
		secondJoystickButtons(robot);
		thirdJoystickButtons(robot);
		fourthJoystickButtons(robot);
		fifthJoystickButtons(robot);
		sixthJoystickButtons(robot);

		Trigger isObjectIn = new Trigger(() -> robot.getSuperstructureRobot().isObjectIn());
		Trigger isTimeToRumble = new Trigger(JoysticksBindings::isTimeToRumble);
		isObjectIn.and(isTimeToRumble).onTrue(noteInRumble(MAIN_JOYSTICK).alongWith(noteInRumble(SECOND_JOYSTICK)));
	}

	public static void setDriversInputsToSwerve(Swerve swerve) {
		if (MAIN_JOYSTICK.isConnected()) {
			swerve.setDriversPowerInputs(
				new ChassisPowers(
					MAIN_JOYSTICK.getAxisValue(Axis.LEFT_Y),
					MAIN_JOYSTICK.getAxisValue(Axis.LEFT_X),
					MAIN_JOYSTICK.getAxisValue(Axis.RIGHT_X)
				)
			);
		} else if (THIRD_JOYSTICK.isConnected()) {
			swerve.setDriversPowerInputs(
				new ChassisPowers(
					THIRD_JOYSTICK.getAxisValue(Axis.LEFT_Y),
					THIRD_JOYSTICK.getAxisValue(Axis.LEFT_X),
					THIRD_JOYSTICK.getAxisValue(Axis.RIGHT_X)
				)
			);
		} else {
			swerve.setDriversPowerInputs(new ChassisPowers(0, 0, 0));
		}
	}

	private static Command noteInRumble(SmartJoystick joystick) {
		return new FunctionalCommand(
			() -> lastTimeRumbled = TimeUtils.getCurrentTimeSeconds(),
			() -> joystick.setRumble(GenericHID.RumbleType.kBothRumble, NOTE_IN_RUMBLE_POWER),
			interrupted -> joystick.stopRumble(GenericHID.RumbleType.kBothRumble),
			() -> false
		).withTimeout(Timeouts.NOTE_IN_RUMBLE);
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...

		usedJoystick.L1.onTrue(robot.getSuperstructureRobot().setState(RobotState.SHOOT_L2));
		usedJoystick.R1.onTrue(robot.getStatesMotionPlanner().feederIntakeToArm());
		usedJoystick.A.onTrue(robot.getSuperstructureRobot().setState(RobotState.IDLE));

		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER, 0.15).whileTrue(robot.getSwerve().setSavedState(() ->
			SwerveState.DEFAULT_DRIVE.withRotateAxis(robot.getSwerve().getStateHandler().getFarLeftRotateAxis()))
		).onFalse(robot.getSwerve().setSavedState(() -> null));;
		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER, 0.15).whileTrue(robot.getSwerve().setSavedState(() ->
			SwerveState.DEFAULT_DRIVE.withRotateAxis(robot.getSwerve().getStateHandler().getFarRightRotateAxis()))
		).onFalse(robot.getSwerve().setSavedState(() -> null));
	}

	private static boolean idleShouldWork(Robot robot) {
		return robot.getSuperstructureRobot().getCurrentState() != RobotState.FEED
			&& robot.getSuperstructureRobot().getCurrentState() != RobotState.TRANSFER_SHOOTER_TO_ARM;
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...

		usedJoystick.POV_LEFT.and((usedJoystick.L1).onTrue(new InstantCommand(() -> CodeCode.reefSide = ReefSide.E)));
		usedJoystick.POV_LEFT.and(() -> !usedJoystick.L1.getAsBoolean()).onTrue(new InstantCommand(() -> CodeCode.reefSide = ReefSide.F));
		usedJoystick.POV_RIGHT.and((usedJoystick.L1)).onTrue(new InstantCommand(() -> CodeCode.reefSide = ReefSide.C));
		usedJoystick.POV_RIGHT.and(() -> !usedJoystick.L1.getAsBoolean()).onTrue(new InstantCommand(() -> CodeCode.reefSide = ReefSide.B));
		usedJoystick.POV_UP.and(usedJoystick.L1).onTrue(new InstantCommand(() -> CodeCode.reefSide = ReefSide.D));
		usedJoystick.POV_UP.and(() -> !usedJoystick.L1.getAsBoolean()).onTrue(new InstantCommand(() -> CodeCode.reefSide = ReefSide.A));

		usedJoystick.R1.toggleOnTrue(new InstantCommand(() -> CodeCode.leftBrnach = !CodeCode.leftBrnach));

		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER).onTrue(
				robot.getSuperstructureRobot().setState(RobotState.PRE_SCORE_REEF));
		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER).onTrue(
				robot.getSuperstructureRobot().setState(RobotState.ALIGN_REEF));

		usedJoystick.START.and(() -> idleShouldWork(robot)).onTrue(robot.getSuperstructureRobot().setState(RobotState.IDLE));
		usedJoystick.START.and(usedJoystick.BACK).onTrue(robot.getSuperstructureRobot().setState(RobotState.IDLE));
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...
		usedJoystick.A.onTrue(robot.getSuperstructureRobot().setState(RobotState.INTAKE));
		usedJoystick.X.onTrue(robot.getSuperstructureRobot().setState(RobotState.ARM_INTAKE));
		usedJoystick.Y.onTrue(robot.getSuperstructureRobot().setState(RobotState.TRANSFER_ARM_TO_SHOOTER));
		usedJoystick.B.onTrue(robot.getSuperstructureRobot().setState(RobotState.TRANSFER_SHOOTER_TO_ARM));
		usedJoystick.POV_LEFT.onTrue(robot.getSuperstructureRobot().setState(RobotState.AMP));
		usedJoystick.POV_RIGHT.onTrue(robot.getSuperstructureRobot().setState(RobotState.PRE_AMP));
		usedJoystick.POV_UP.onTrue(robot.getStatesMotionPlanner().feederIntakeToArm());
		usedJoystick.POV_DOWN.onTrue(robot.getSuperstructureRobot().setState(RobotState.SHOOT_L2));
	}

	private static void fourthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FOURTH_JOYSTICK;
		// bindings...

		usedJoystick.B.onTrue(new InstantCommand(() -> robot.getPoseEstimator().resetPose(new Pose2d(5, 5, new Rotation2d()))));

		// usedJoystick.A.whileTrue(robot.getSwerve().getCommandsBuilder().pointWheelsInX());
		usedJoystick.X.whileTrue(robot.getSwerve().getCommandsBuilder().pointWheels(Rotation2d.fromDegrees(90), true));

		usedJoystick.POV_UP.whileTrue(robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(180)));
		usedJoystick.POV_DOWN.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.turnToHeading(Rotation2d.fromDegrees(-17))
				.until(
					() -> robot.getSwerve()
						.isAtHeading(Rotation2d.fromDegrees(-17), Tolerances.SWERVE_HEADING, Tolerances.ROTATION_VELOCITY_DEADBAND)
				)
		);

		usedJoystick.POV_LEFT
			.whileTrue(robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(-17), RotateAxis.FRONT_LEFT_MODULE));
		usedJoystick.POV_RIGHT
			.whileTrue(robot.getSwerve().getCommandsBuilder().turnToHeading(Rotation2d.fromDegrees(180), RotateAxis.BACK_RIGHT_MODULE));

		usedJoystick.R1.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveByState(
					() -> new ChassisPowers(
						usedJoystick.getAxisValue(Axis.LEFT_Y),
						usedJoystick.getAxisValue(Axis.LEFT_X),
						usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X)
					),
					SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.REEF)
				)
		);
		usedJoystick.L1.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveByState(
					() -> new ChassisPowers(
						usedJoystick.getAxisValue(Axis.LEFT_Y),
						usedJoystick.getAxisValue(Axis.LEFT_X),
						usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X)
					),
					SwerveState.DEFAULT_DRIVE.withDriveRelative(DriveRelative.ROBOT_RELATIVE).withAimAssist(AimAssist.BRANCH)
				)
		);

		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER)
			.whileTrue(
				robot.getSwerve()
					.getCommandsBuilder()
					.driveByState(
						() -> new ChassisPowers(
							usedJoystick.getAxisValue(Axis.LEFT_Y),
							usedJoystick.getAxisValue(Axis.LEFT_X),
							usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X)
						),
						() -> SwerveState.DEFAULT_DRIVE.withRotateAxis(robot.getSwerve().getStateHandler().getFarRightRotateAxis())
					)
			);
		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER)
			.whileTrue(
				robot.getSwerve()
					.getCommandsBuilder()
					.driveByState(
						() -> new ChassisPowers(
							usedJoystick.getAxisValue(Axis.LEFT_Y),
							usedJoystick.getAxisValue(Axis.LEFT_X),
							usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X)
						),
						() -> SwerveState.DEFAULT_DRIVE.withRotateAxis(robot.getSwerve().getStateHandler().getFarLeftRotateAxis())
					)
			);

		// robot.getSwerve()
		// .setDefaultCommand(
		// robot.getSwerve()
		// .getCommandsBuilder()
		// .drive(
		// () -> usedJoystick.getAxisValue(Axis.LEFT_Y),
		// () -> usedJoystick.getAxisValue(Axis.LEFT_X),
		// () -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X)
		// )
		// );

		usedJoystick.BACK.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> new Pose2d(4, 4, Rotation2d.fromDegrees(17)))
				.until(() -> robot.getSuperstructureFunny().isAtPose(new Pose2d(4, 4, Rotation2d.fromDegrees(17))))
		);
		usedJoystick.START.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> new Pose2d(6, 6, Rotation2d.fromDegrees(90)))
				.until(() -> robot.getSuperstructureFunny().isAtPose(new Pose2d(6, 6, Rotation2d.fromDegrees(90))))
		);
	}

	private static void fifthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FIFTH_JOYSTICK;
		// bindings...
		usedJoystick.A.whileTrue(robot.getSwerve().getCommandsBuilder().wheelRadiusCalibration());
		usedJoystick.B.whileTrue(robot.getSwerve().getCommandsBuilder().steerCalibration(true, SysIdRoutine.Direction.kForward));
		usedJoystick.Y.whileTrue(robot.getSwerve().getCommandsBuilder().driveCalibration(true, SysIdRoutine.Direction.kForward));

		usedJoystick.POV_DOWN.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> new ChassisPowers(0.2, 0, 0)));
		usedJoystick.POV_LEFT.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> new ChassisPowers(0.5, 0, 0)));
		usedJoystick.POV_RIGHT.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> new ChassisPowers(-0.2, 0, 0)));
		usedJoystick.POV_UP.whileTrue(robot.getSwerve().getCommandsBuilder().drive(() -> new ChassisPowers(-0.5, 0, 0)));
	}

	private static void sixthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SIXTH_JOYSTICK;
		// bindings...
	}

}
