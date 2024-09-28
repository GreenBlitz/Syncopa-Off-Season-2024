package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.constants.Field;
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
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import org.littletonrobotics.junction.Logger;

public class Supersturcture {
	
	private final Robot robot;
	
	private final Swerve swerve;
	private final ElbowStateHandler elbowStateHandler;
	private final FlywheelStateHandler flywheelStateHandler;
	private final FunnelStateHandler funnelStateHandler;
	private final IntakeStateHandler intakeStateHandler;
	private final PivotStateHandler pivotStateHandler;
	private final RollerStateHandler rollerStateHandler;
	
	private RobotState currentState;
	
	public Supersturcture(Robot robot) {
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.elbowStateHandler = new ElbowStateHandler(robot.getElbow());
		this.flywheelStateHandler = new FlywheelStateHandler(robot.getFlywheel());
		this.funnelStateHandler = new FunnelStateHandler(robot.getFunnel());
		this.intakeStateHandler = new IntakeStateHandler(robot.getIntake());
		this.pivotStateHandler = new PivotStateHandler(robot.getPivot());
		this.rollerStateHandler = new RollerStateHandler(robot.getRoller());
	}
	
	public RobotState getCurrentState() {
		return currentState;
	}
	
	public void logStatus() {
		Logger.recordOutput("CurrentState", currentState);
	}
	
	private boolean isObjectInRoller() {
		return robot.getRoller().isObjectIn();
	}
	
	private boolean isObjectInIntake() {
		return robot.getIntake().isObjectIn();
	}
	
	private boolean isObjectInFunnel() {
		return robot.getFunnel().isObjectIn();
	}
	
	private boolean isReadyToShoot() {
		boolean isPivotReady = robot.getPivot().isAtPosition(PivotState.PRE_SPEAKER.getTargetPosition(), Tolerances.PIVOT_POSITION_TOLERANCE);
		
		boolean isFlywheelReady = robot.getFlywheel()
				.isAtVelocities(
						FlywheelState.PRE_SPEAKER.getRightVelocity(),
						FlywheelState.PRE_SPEAKER.getLeftVelocity(),
						Tolerances.FLYWHEEL_VELOCITY_PER_SECOND_TOLERANCE
				);
		
		return isFlywheelReady && isPivotReady;
	}
	
	
	public Command setState(RobotState state) {
		this.currentState = state;
		return switch (state) {
			case IDLE -> idle();
			case INTAKE -> intake();
			case PRE_SPEAKER -> preSpeaker();
			case SPEAKER -> speaker();
			case PRE_AMP -> preAMP();
			case AMP -> amp();
			case SHOOTER_OUTTAKE -> shooterOuttake();
			case TRANSFER_SHOOTER_TO_ARM -> transferShooterToArm();
			case TRANSFER_ARM_TO_SHOOTER -> transferArmToShooter();
			case INTAKE_OUTTAKE -> null;
		};
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
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command intake() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE),
					rollerStateHandler.setState(RollerState.ROLL_IN)
				).until(this::isObjectInIntake),
				new ParallelDeadlineGroup(
					new WaitCommand(0.475),
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL),
					funnelStateHandler.setState(FunnelState.INTAKE),
					rollerStateHandler.setState(RollerState.ROLL_IN)
				)
//						new ParallelDeadlineGroup(
//								new WaitCommand(0.5),
//								funnelStateHandler.setState(Fun)
//						)
			),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			pivotStateHandler.setState(PivotState.IDLE),
			elbowStateHandler.setState(ElbowState.INTAKE),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE))
		);
	}

	private Command preSpeaker() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.STOP),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.STOP),
			pivotStateHandler.setState(PivotState.PRE_SPEAKER),
			flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
			elbowStateHandler.setState(ElbowState.IDLE),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
		);
	}

	private Command speaker() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				funnelStateHandler.setState(FunnelState.STOP).until(this::isReadyToShoot),
				funnelStateHandler.setState(FunnelState.SHOOT).withTimeout(3)//.until(() -> !isObjectInFunnel())
			),
			intakeStateHandler.setState(IntakeState.STOP),
			rollerStateHandler.setState(RollerState.STOP),
			pivotStateHandler.setState(PivotState.PRE_SPEAKER),
			flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
			elbowStateHandler.setState(ElbowState.IDLE),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
		).andThen(idle());
	}

	private Command preAMP() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
					funnelStateHandler.setState(FunnelState.STOP)
				).until(() -> swerve.isAtHeading(Field.getAngleToAmp())),
				new ParallelDeadlineGroup(
					elbowStateHandler.setState(ElbowState.PRE_AMP),
					funnelStateHandler.setState(FunnelState.RELEASE_FOR_ARM)
				),
				funnelStateHandler.setState(FunnelState.STOP)
			),
			rollerStateHandler.setState(RollerState.STOP),
			intakeStateHandler.setState(IntakeState.STOP),
			pivotStateHandler.setState(PivotState.IDLE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT)
		);
	}

	private Command amp() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelDeadlineGroup(
					swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
					funnelStateHandler.setState(FunnelState.STOP),
					rollerStateHandler.setState(RollerState.STOP)
				).until(() -> swerve.isAtHeading(Field.getAngleToAmp())),
				new ParallelDeadlineGroup(
					elbowStateHandler.setState(ElbowState.PRE_AMP),
					funnelStateHandler.setState(FunnelState.RELEASE_FOR_ARM)
				),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					rollerStateHandler.setState(RollerState.ROLL_OUT)
				).withTimeout(3)//.until(() -> !isObjectInRoller())
			),
			intakeStateHandler.setState(IntakeState.STOP),
			pivotStateHandler.setState(PivotState.IDLE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT)
		);
	}

	private Command shooterOuttake() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.ROLL_OUT),
			intakeStateHandler.setState(IntakeState.OUTTAKE),
			funnelStateHandler.setState(FunnelState.OUTTAKE),
			pivotStateHandler.setState(PivotState.INTAKE),
			elbowStateHandler.setState(ElbowState.MANUAL),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command transferShooterToArm() {
		return new SequentialCommandGroup(
			new ParallelCommandGroup(
				pivotStateHandler.setState(PivotState.TRANSFER),
				elbowStateHandler.setState(ElbowState.TRANSFER)
			),
			new ParallelDeadlineGroup(
				rollerStateHandler.setState(RollerState.INTAKE),
				funnelStateHandler.setState(FunnelState.TRANSFER_TO_ARM)
			)
		);
	}

	private Command transferArmToShooter() {
		return new SequentialCommandGroup(
			new ParallelCommandGroup(
				pivotStateHandler.setState(PivotState.TRANSFER),
				elbowStateHandler.setState(ElbowState.TRANSFER)
			),
			new ParallelCommandGroup(
				funnelStateHandler.setState(FunnelState.TRANSFER_TO_SHOOTER),
				rollerStateHandler.setState(RollerState.ROLL_OUT)
			).withTimeout(0.7)//.until(this::isObjectInFunnel)
		);
	}
	//@formatter:on
	
}
