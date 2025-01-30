package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.elbow.ElbowState;
import frc.robot.subsystems.elbow.ElbowStateHandler;
import frc.robot.subsystems.flywheel.FlywheelState;
import frc.robot.subsystems.flywheel.FlywheelStateHandler;
import frc.robot.subsystems.funnel.FunnelState;
import frc.robot.subsystems.funnel.FunnelStateHandler;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.intake.IntakeStateHandler;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.subsystems.pivot.PivotStateHandler;
import frc.robot.subsystems.roller.RollerState;
import frc.robot.subsystems.roller.RollerStateHandler;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.subsystems.wrist.WristStateHandler;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

public class Superstructure extends GBSubsystem {

	private interface RobotCommandGenerator {

		Command makeRobotCommand(Command command, RobotState state);

	}

	private final RobotCommandGenerator robotCommandGenerator = (command, state) -> {
		command.addRequirements(this);
		return command.beforeStarting(setCurrentStateName(state));
	};

	private final Robot robot;
	private final Swerve swerve;
	private final ElbowStateHandler elbowStateHandler;
	private final FlywheelStateHandler flywheelStateHandler;
	private final FunnelStateHandler funnelStateHandler;
	private final IntakeStateHandler intakeStateHandler;
	private final PivotStateHandler pivotStateHandler;
	private final RollerStateHandler rollerStateHandler;
	private final WristStateHandler wristStateHandler;
	private final EndBehaviorManager endBehaviorManager;

	private RobotState currentState;

	public Superstructure(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.elbowStateHandler = new ElbowStateHandler(null);
		this.flywheelStateHandler = new FlywheelStateHandler(null);
		this.funnelStateHandler = new FunnelStateHandler(null);
		this.intakeStateHandler = new IntakeStateHandler(null);
		this.pivotStateHandler = new PivotStateHandler(null, null);
		this.rollerStateHandler = new RollerStateHandler(null);
		this.wristStateHandler = new WristStateHandler(null);

		this.currentState = RobotState.IDLE;
		this.endBehaviorManager = new EndBehaviorManager(this);

		setDefaultCommand(new DeferredCommand(() -> endBehaviorManager.endState(currentState), Set.of(this)));
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public void logStatus() {
		Logger.recordOutput(getLogPath() + "CurrentState", currentState);
	}

	public boolean isObjectInIntake() {
		return true; // robot.getIntake().isObjectIn();
	}

	public boolean isObjectInFunnel() {
		return true; // robot.getFunnel().isObjectIn();
	}

	public boolean isObjectIn() {
		return isObjectInFunnel() || isObjectInIntake();
	}

	private boolean isReadyToTransfer() {
//		boolean isPivotReady = robot.getPivot().isAtPosition(PivotState.TRANSFER.getTargetPosition(), Tolerances.PIVOT_POSITION);
//		boolean isElbowReady = robot.getElbow().isAtAngle(ElbowState.TRANSFER.getTargetPosition(), Tolerances.ELBOW_POSITION_TRANSFER);
//
		return true; // isElbowReady && isPivotReady;
	}

	private boolean isReadyToShootClose() {
//		boolean isPivotReady = robot.getPivot().isAtPosition(PivotState.PRE_SPEAKER.getTargetPosition(), Tolerances.PIVOT_POSITION);

//		boolean isFlywheelReady = robot.getFlywheel()
//			.isAtVelocities(
//				FlywheelState.PRE_SPEAKER.getRightVelocity(),
//				FlywheelState.PRE_SPEAKER.getLeftVelocity(),
//				Tolerances.FLYWHEEL_VELOCITY_PER_SECOND
//			);

		return false; // isFlywheelReady && isPivotReady;
	}

	private boolean isReadyToPass() {
//		boolean isPivotReady = robot.getPivot().isAtPosition(PivotState.PASSING.getTargetPosition(), Tolerances.PIVOT_POSITION);

//		boolean isFlywheelReady = robot.getFlywheel()
//			.isAtVelocities(
//				FlywheelState.PASSING.getRightVelocity(),
//				FlywheelState.PASSING.getLeftVelocity(),
//				Tolerances.FLYWHEEL_VELOCITY_PER_SECOND
//			);

		return false;// isFlywheelReady && isPivotReady;
	}

	private boolean isReadyToShootInterpolation() {
		Translation2d robotTranslation2d = null;// robot.getPoseEstimator().getEstimatedPose().getTranslation();

		double metersFromSpeaker = 0;// Field.getSpeaker().toTranslation2d().getDistance(robotTranslation2d);
//		boolean isPivotReady = robot.getPivot()
//			.isAtPosition(Rotation2d.fromRadians(PivotInterpolationMap.METERS_TO_RADIANS.get(metersFromSpeaker)), Tolerances.PIVOT_POSITION);
//
//		boolean isFlywheelReady = robot.getFlywheel()
//			.isAtVelocities(
//				FlywheelState.PRE_SPEAKER.getRightVelocity(),
//				FlywheelState.PRE_SPEAKER.getLeftVelocity(),
//				Tolerances.FLYWHEEL_VELOCITY_PER_SECOND
//			);

		Rotation2d angleToSpeaker = new Rotation2d();// PoseMath.getRelativeTranslation(robotTranslation2d,
														// Field.getSpeaker().toTranslation2d()).getAngle();
		boolean isSwerveReady = swerve.isAtHeading(angleToSpeaker, Tolerances.SWERVE_HEADING, Tolerances.ROTATION_VELOCITY_DEADBAND);

		return false;// isFlywheelReady && isPivotReady && isSwerveReady;
	}

	private Command setCurrentStateName(RobotState state) {
		return new InstantCommand(() -> currentState = state);
	}

	public Command setState(RobotState state) {
		return robotCommandGenerator.makeRobotCommand(switch (state) {
			case IDLE -> idle();
			case INTAKE -> intake();
			case INTAKE_WITH_FLYWHEEL -> intakeWithFlywheel();
			case FEEDER_INTAKE -> feederIntake();
			case ARM_INTAKE -> armIntake();
			case PRE_SPEAKER -> preSpeaker();
			case SPEAKER -> speaker();
			case PRE_AMP -> preAMP();
			case AMP -> amp();
			case ARM_UP -> armUp();
			case SHOOT_L2 -> shootL2();
			case TRANSFER_SHOOTER_TO_ARM -> transferShooterToArm();
			case TRANSFER_ARM_TO_SHOOTER -> transferArmToShooter();
			case INTAKE_OUTTAKE -> intakeOuttake();
			case ARM_OUTTAKE -> armOuttake();
			case PASSING -> passing();
			case FEED -> feed();
			case ALIGN_REEF -> alignReef();
			case PRE_SCORE_REEF -> preReef();
		}, state);
	}

	private Command preReef() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					wristStateHandler.setState(WristState.IN_ARM),
					rollerStateHandler.setState(RollerState.STOP)
				).withTimeout(0.1), // .until(() -> swerve.isAtHeading(Field.getAngleToAmp(), Tolerances.SWERVE_HEADING,
									// Tolerances.ROTATION_VELOCITY_DEADBAND)),
				new ParallelCommandGroup(
					elbowStateHandler.setState(ElbowState.PRE_AMP),
					funnelStateHandler.setState(FunnelState.RELEASE_FOR_ARM),
					intakeStateHandler.setState(IntakeState.RELEASE_FOR_ARM),
					wristStateHandler.setState(WristState.IN_ARM)
				)
			),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH)),
			pivotStateHandler.setState(PivotState.IDLE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT)
		);
	}

	private Command alignReef() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.MANUAL),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.MANUAL),
			pivotStateHandler.setState(PivotState.IDLE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			elbowStateHandler.setState(ElbowState.IDLE),
			wristStateHandler.setState(WristState.DEFAULT),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.REEF))
		);
	}

	private Command feed() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.FEEDER),
					funnelStateHandler.setState(FunnelState.STOP),
					intakeStateHandler.setState(IntakeState.STOP),
					flywheelStateHandler.setState(FlywheelState.DEFAULT)
				), // .until(() -> robot.getPivot().isAtPosition(PivotState.FEEDER.getTargetPosition(), Tolerances.PIVOT_POSITION)),
				new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.FEEDER),
					funnelStateHandler.setState(FunnelState.OUTTAKE),
					intakeStateHandler.setState(IntakeState.OUTTAKE),
					flywheelStateHandler.setState(FlywheelState.DEFAULT)
				).until(this::isObjectInFunnel),
				new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.FEEDER),
					funnelStateHandler.setState(FunnelState.OUTTAKE),
					intakeStateHandler.setState(IntakeState.OUTTAKE),
					flywheelStateHandler.setState(FlywheelState.DEFAULT)
				).until(() -> !isObjectInFunnel()),
				new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.FEEDER),
					funnelStateHandler.setState(FunnelState.SLOW_INTAKE),
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL),
					flywheelStateHandler.setState(FlywheelState.DEFAULT)
				).until(this::isObjectInFunnel)
			),
			rollerStateHandler.setState(RollerState.STOP),
			wristStateHandler.setState(WristState.DEFAULT),
			elbowStateHandler.setState(ElbowState.IDLE),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.FEEDER))
		);
	}

	//@formatter:off
	private Command idle() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.MANUAL),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.MANUAL),
			pivotStateHandler.setState(PivotState.IDLE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			elbowStateHandler.setState(ElbowState.IDLE),
			wristStateHandler.setState(WristState.DEFAULT),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command shootL2() {
		return new ParallelDeadlineGroup(
				new SequentialCommandGroup(
					new ParallelCommandGroup(
						funnelStateHandler.setState(FunnelState.STOP),
						wristStateHandler.setState(WristState.IN_ARM),
						rollerStateHandler.setState(RollerState.STOP)
				).withTimeout(0.1),//.until(() -> swerve.isAtHeading(Field.getAngleToAmp(), Tolerances.SWERVE_HEADING, Tolerances.ROTATION_VELOCITY_DEADBAND)),
				new ParallelCommandGroup(
					elbowStateHandler.setState(ElbowState.PRE_AMP),
					funnelStateHandler.setState(FunnelState.RELEASE_FOR_ARM),
					intakeStateHandler.setState(IntakeState.RELEASE_FOR_ARM),
					wristStateHandler.setState(WristState.PRE_TRAP)
				),//.until(() -> robot.getElbow().isAtAngle(ElbowState.PRE_AMP.getTargetPosition(), Tolerances.ELBOW_POSITION)),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					intakeStateHandler.setState(IntakeState.STOP),
					rollerStateHandler.setState(RollerState.ROLL_OUT)
				).withTimeout(Timeouts.AMP_RELEASE_SECONDS)//.until(() -> !isObjectInRoller())
			),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH)),
			pivotStateHandler.setState(PivotState.IDLE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT)
		);
	}

	private Command intake() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE),
					rollerStateHandler.setState(RollerState.ROLL_IN),
					funnelStateHandler.setState(FunnelState.STOP)
				).until(this::isObjectInIntake),
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL),
					funnelStateHandler.setState(FunnelState.INTAKE),
					rollerStateHandler.setState(RollerState.ROLL_IN)
				).until(this::isObjectInFunnel)
			),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			pivotStateHandler.setState(PivotState.IDLE),
			elbowStateHandler.setState(ElbowState.INTAKE),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command intakeWithFlywheel() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE),
					rollerStateHandler.setState(RollerState.ROLL_IN),
					funnelStateHandler.setState(FunnelState.STOP)
				).until(this::isObjectInIntake),
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL),
					funnelStateHandler.setState(FunnelState.INTAKE),
					rollerStateHandler.setState(RollerState.ROLL_IN)
				).until(this::isObjectInFunnel)
			),
			flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
			pivotStateHandler.setState(PivotState.IDLE),
			elbowStateHandler.setState(ElbowState.INTAKE),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command feederIntake() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.FEEDER),
					funnelStateHandler.setState(FunnelState.STOP),
					intakeStateHandler.setState(IntakeState.STOP),
					flywheelStateHandler.setState(FlywheelState.FEEDER)
				),//.until(() -> robot.getPivot().isAtPosition(PivotState.FEEDER.getTargetPosition(), Tolerances.PIVOT_POSITION)),
				new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.FEEDER),
					funnelStateHandler.setState(FunnelState.OUTTAKE),
					intakeStateHandler.setState(IntakeState.OUTTAKE),
					flywheelStateHandler.setState(FlywheelState.FEEDER)
				).until(this::isObjectInFunnel),
				new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.FEEDER),
					funnelStateHandler.setState(FunnelState.OUTTAKE),
					intakeStateHandler.setState(IntakeState.OUTTAKE),
					flywheelStateHandler.setState(FlywheelState.FEEDER)
				).until(() -> !isObjectInFunnel()),
				new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.FEEDER),
					funnelStateHandler.setState(FunnelState.SLOW_INTAKE),
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL),
					flywheelStateHandler.setState(FlywheelState.FEEDER)
				).until(this::isObjectInFunnel)
			),
			rollerStateHandler.setState(RollerState.STOP),
			wristStateHandler.setState(WristState.DEFAULT),
			elbowStateHandler.setState(ElbowState.IDLE),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command armIntake() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE),
					rollerStateHandler.setState(RollerState.ROLL_IN),
					wristStateHandler.setState(WristState.ARM_INTAKE)
				).until(this::isObjectInIntake),
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_ARM),
					rollerStateHandler.setState(RollerState.ROLL_IN),
					funnelStateHandler.setState(FunnelState.SLOW_INTAKE),
					wristStateHandler.setState(WristState.ARM_INTAKE)
				).withTimeout(Timeouts.INTAKE_ROLLER_SECONDS),//.until(this::isObjectInRoller)
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_ARM),
					rollerStateHandler.setState(RollerState.STOP),
					funnelStateHandler.setState(FunnelState.SLOW_INTAKE),
					wristStateHandler.setState(WristState.DEFAULT)
				)//.withTimeout(Timeouts.WRIST_TO_POSITION_SECONDS)
				 //.until(() -> robot.getWrist().isAtPosition(WristState.DEFAULT.getPosition(), Tolerances.WRIST_POSITION))
			),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			pivotStateHandler.setState(PivotState.ARM_INTAKE),
			elbowStateHandler.setState(ElbowState.ARM_INTAKE),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command preSpeaker() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.STOP),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.MANUAL),
			pivotStateHandler.setState(PivotState.INTERPOLATE),
			flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
			elbowStateHandler.setState(ElbowState.IDLE),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command speaker() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					intakeStateHandler.setState(IntakeState.STOP)
				).until(this::isReadyToShootInterpolation),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.SHOOT),
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL)
				).until(() -> !isObjectInFunnel())
			),
			rollerStateHandler.setState(RollerState.STOP),
			pivotStateHandler.setState(PivotState.INTERPOLATE),
			flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
			elbowStateHandler.setState(ElbowState.INTAKE),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command preAMP() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP)
				),//.until(() -> swerve.isAtHeading(Field.getAngleToAmp(), Tolerances.SWERVE_HEADING, Tolerances.ROTATION_VELOCITY_DEADBAND)),
				new ParallelCommandGroup(
					elbowStateHandler.setState(ElbowState.PRE_AMP),
					funnelStateHandler.setState(FunnelState.RELEASE_FOR_ARM),
					intakeStateHandler.setState(IntakeState.RELEASE_FOR_ARM)
				),//.until(() -> robot.getElbow().isAtAngle(ElbowState.PRE_AMP.getTargetPosition(), Tolerances.ELBOW_POSITION)),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					intakeStateHandler.setState(IntakeState.STOP)
				)
			),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE),
			rollerStateHandler.setState(RollerState.STOP),
			pivotStateHandler.setState(PivotState.IDLE),
			wristStateHandler.setState(WristState.IN_ARM),
			flywheelStateHandler.setState(FlywheelState.DEFAULT)
		);
	}

	private Command amp() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					rollerStateHandler.setState(RollerState.STOP)
				).withTimeout(0.1),//.until(() -> swerve.isAtHeading(Field.getAngleToAmp(), Tolerances.SWERVE_HEADING, Tolerances.ROTATION_VELOCITY_DEADBAND)),
				new ParallelCommandGroup(
					elbowStateHandler.setState(ElbowState.PRE_AMP),
					funnelStateHandler.setState(FunnelState.RELEASE_FOR_ARM),
					intakeStateHandler.setState(IntakeState.RELEASE_FOR_ARM)
				),//.until(() -> robot.getElbow().isAtAngle(ElbowState.PRE_AMP.getTargetPosition(), Tolerances.ELBOW_POSITION)),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					intakeStateHandler.setState(IntakeState.STOP),
					rollerStateHandler.setState(RollerState.ROLL_OUT)
				).withTimeout(Timeouts.AMP_RELEASE_SECONDS)//.until(() -> !isObjectInRoller())
			),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE),
			pivotStateHandler.setState(PivotState.IDLE),
			wristStateHandler.setState(WristState.IN_ARM),
			flywheelStateHandler.setState(FlywheelState.DEFAULT)
		);
	}

	private Command armUp() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.MANUAL),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.MANUAL),
			pivotStateHandler.setState(PivotState.IDLE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			elbowStateHandler.setState(ElbowState.PRE_AMP),
			wristStateHandler.setState(WristState.DEFAULT),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command transferShooterToArm() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.TRANSFER),
					elbowStateHandler.setState(ElbowState.TRANSFER),
					rollerStateHandler.setState(RollerState.STOP),
					funnelStateHandler.setState(FunnelState.STOP)
				).until(this::isReadyToTransfer),
				new ParallelCommandGroup(
					rollerStateHandler.setState(RollerState.ROLL_IN),
					funnelStateHandler.setState(FunnelState.TRANSFER_TO_ARM)
				).withTimeout(Timeouts.TRANSFER_SHOOTER_ARM_SECONDS),//.until(this::isObjectInRoller),
				new ParallelDeadlineGroup(
					rollerStateHandler.setState(RollerState.AFTER_INTAKE),
					funnelStateHandler.setState(FunnelState.TRANSFER_TO_ARM)
				).withTimeout(Timeouts.INTAKE_ARM_1_ROTATION_SECONDS)
			),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			intakeStateHandler.setState(IntakeState.STOP),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command transferArmToShooter() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.TRANSFER),
					elbowStateHandler.setState(ElbowState.TRANSFER),
					rollerStateHandler.setState(RollerState.STOP),
					intakeStateHandler.setState(IntakeState.STOP),
					funnelStateHandler.setState(FunnelState.STOP)
				).until(this::isReadyToTransfer),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.INTAKE),
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL),
					rollerStateHandler.setState(RollerState.ROLL_OUT)
				).until(this::isObjectInFunnel)
			),
			wristStateHandler.setState(WristState.IN_ARM),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command intakeOuttake() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.ROLL_OUT),
			intakeStateHandler.setState(IntakeState.OUTTAKE),
			funnelStateHandler.setState(FunnelState.OUTTAKE),
			pivotStateHandler.setState(PivotState.INTAKE),
			elbowStateHandler.setState(ElbowState.MANUAL),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command armOuttake() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.FAST_ROLL_IN),
			elbowStateHandler.setState(ElbowState.ARM_INTAKE),
			wristStateHandler.setState(WristState.DEFAULT),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.SLOW_OUTTAKE),
			pivotStateHandler.setState(PivotState.INTAKE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command passing() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					intakeStateHandler.setState(IntakeState.STOP)
				).until(this::isReadyToPass),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.SHOOT),
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL)
				).until(() -> !isObjectInFunnel())
			),
			rollerStateHandler.setState(RollerState.STOP),
			pivotStateHandler.setState(PivotState.PASSING),
			flywheelStateHandler.setState(FlywheelState.PASSING),
			elbowStateHandler.setState(ElbowState.INTAKE),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
		);
	}
	//@formatter:on

}
