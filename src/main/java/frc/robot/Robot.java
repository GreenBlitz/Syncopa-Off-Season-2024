// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.CodeCode;
import frc.constants.field.Field;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelConstants;
import frc.robot.subsystems.funnel.factory.FunnelFactory;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.factory.IntakeFactory;
import frc.robot.subsystems.elbow.Elbow;
import frc.robot.subsystems.elbow.ElbowConstants;
import frc.robot.subsystems.elbow.factory.ElbowFactory;
import frc.robot.subsystems.flywheel.FlyWheelConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.factory.FlywheelFactory;
import frc.robot.subsystems.lifter.Lifter;
import frc.robot.subsystems.lifter.LifterConstants;
import frc.robot.subsystems.lifter.factory.LifterFactory;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.factory.PivotFactory;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.factory.RollerFactory;
import frc.robot.subsystems.solenoid.Solenoid;
import frc.robot.subsystems.solenoid.SolenoidConstants;
import frc.robot.subsystems.solenoid.factory.SolenoidFactory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.constants.RobotHeadingEstimatorConstants;
import frc.robot.autonomous.AutonomousConstants;
import frc.constants.VisionConstants;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.poseestimator.helpers.RobotHeadingEstimator;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.factory.WristFactory;
import frc.robot.superstructure.StatesMotionPlanner;
import frc.utils.brakestate.BrakeStateManager;
import frc.utils.auto.AutonomousChooser;
import frc.robot.vision.data.HeadingData;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSources;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.battery.BatteryUtils;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Swerve swerve;
	private final Solenoid solenoid;
	private final Funnel funnel;
	private final Intake intake;
	private final Elbow elbow;
	private final Flywheel flywheel;
	private final Pivot pivot;
	private final Lifter lifter;
	private final Roller roller;
	private final Wrist wrist;

	private final Superstructure superstructureFunny;
	private final StatesMotionPlanner statesMotionPlanner;
	private final IPoseEstimator poseEstimator;
	private final MultiAprilTagVisionSources aprilTagVisionSources;
	private final frc.robot.superstructure.Superstructure superstructureRobot;
	private RobotHeadingEstimator headingEstimator = null;

	private AutonomousChooser autonomousChooser;

	public Robot() {
		BatteryUtils.scheduleLimiter();
		IGyro gyro = GyroFactory.createGyro(RobotConstants.SUBSYSTEM_LOG_PREFIX + "Swerve/");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOG_PREFIX + "Swerve/"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOG_PREFIX + "Swerve/"),
			gyro,
			GyroFactory.createSignals(gyro)
		);

		this.solenoid = new Solenoid(SolenoidFactory.create(SolenoidConstants.LOG_PATH));
		this.intake = new Intake(IntakeFactory.create(IntakeConstants.LOG_PATH));
		this.flywheel = new Flywheel(FlywheelFactory.create(FlyWheelConstants.LOG_PATH));
		this.pivot = new Pivot(PivotFactory.create(PivotConstants.LOG_PATH));
		BrakeStateManager.add(() -> pivot.setBrake(true), () -> pivot.setBrake(false));
		this.elbow = new Elbow(ElbowFactory.create(ElbowConstants.LOG_PATH));
		BrakeStateManager.add(() -> elbow.setBrake(true), () -> elbow.setBrake(false));
		this.funnel = new Funnel(FunnelFactory.create(FunnelConstants.LOG_PATH));
		this.lifter = new Lifter(LifterFactory.create(LifterConstants.LOG_PATH));
		BrakeStateManager.add(() -> lifter.setBrake(true), () -> lifter.setBrake(false));
		this.roller = new Roller(RollerFactory.create(RollerConstants.LOG_PATH));
		BrakeStateManager.add(() -> roller.setBrake(true), () -> roller.setBrake(false));
		this.wrist = new Wrist(WristFactory.create(WristConstants.LOG_PATH));
		BrakeStateManager.add(() -> wrist.setBrake(true), () -> wrist.setBrake(false));

		this.poseEstimator = new WPILibPoseEstimatorWrapper(
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH,
			swerve.getKinematics(),
			swerve.getAllOdometryObservations()[0].wheelPositions(),
			WPILibPoseEstimatorConstants.INITIAL_GYRO_ANGLE
		);

//		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getEstimatedPose);
		swerve.getStateHandler().setReedTranslationSupplier(() -> Optional.of(Field.getMiddleOfReefSide(CodeCode.reefSide).getTranslation()));
		swerve.getStateHandler().setBranchTranslationSupplier(() -> Optional.of(Field.branchCool(CodeCode.reefSide, CodeCode.leftBrnach)));
		swerve.getStateHandler()
			.setFeederTranslationSupplier(() -> Optional.of(Field.getMiddleOfCoralStation(CodeCode.coralStationPosition).getTranslation()));

		headingEstimator = new RobotHeadingEstimator(swerve.getGyroAbsoluteYaw(), swerve.getGyroAbsoluteYaw(), 0.0001);

		this.aprilTagVisionSources = new MultiAprilTagVisionSources(
			VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
			() -> (headingEstimator.getEstimatedHeading()),
			() -> Rotation2d.fromDegrees(0),
			VisionConstants.DEFAULT_VISION_POSEESTIMATING_SOURCES
		);
		this.superstructureFunny = new Superstructure(swerve, poseEstimator);
		this.superstructureRobot = new frc.robot.superstructure.Superstructure("Superstructure/", this);
		this.statesMotionPlanner = new StatesMotionPlanner(superstructureRobot);

		configPathPlanner();
	}

	public void periodic() {
		swerve.update();
		headingEstimator.updateGyroAngle(new HeadingData(swerve.getGyroAbsoluteYaw(), TimeUtils.getCurrentTimeSeconds()));
		List<TimedValue<Rotation2d>> headingAndTime = aprilTagVisionSources.getRawRobotHeadings();
		if (!headingAndTime.isEmpty()) {
			Logger.recordOutput("Robot Heading", headingAndTime.get(0).value());
			headingEstimator.updateVisionIfNotCalibrated(
				new HeadingData(headingAndTime.get(0).value(), headingAndTime.get(0).timestamp()),
				RobotHeadingEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATION,
				0.001
			);
//			headingEstimator.updateVisionHeading(headingAndTime.get(0).value(), headingAndTime.get(0).timestamp());
//			headingEstimator.updateVisionHeading(headingAndTime.get(0).getFirst(), TimeUtils.getCurrentTimeSeconds());
		}
		poseEstimator.updateOdometry(swerve.getAllOdometryObservations());
		poseEstimator.updateVision(aprilTagVisionSources.getUnfilteredVisionData());

		superstructureFunny.periodic();
		superstructureRobot.periodic();
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	private void configPathPlanner() {
		// Register commands..
//		PathPlannerUtils.registerCommand(RobotState.INTAKE_WITH_FLYWHEEL.name(), superstructure.setState(RobotState.INTAKE_WITH_FLYWHEEL));
//		PathPlannerUtils.registerCommand(RobotState.PRE_SPEAKER.name(), superstructure.setState(RobotState.PRE_SPEAKER));
//		PathPlannerUtils.registerCommand(RobotState.SPEAKER.name(), superstructure.setState(RobotState.SPEAKER));

		swerve.configPathPlanner(
			poseEstimator::getEstimatedPose,
			poseEstimator::resetPose,
			PathPlannerUtils.getGuiRobotConfig().orElse(AutonomousConstants.SYNCOPA_ROBOT_CONFIG)
		);
//		autonomousChooser = new AutonomousChooser("Autonomous Chooser");
		superstructureRobot.periodic();
		aprilTagVisionSources.log();
		Logger.recordOutput("Robot Heading By Estimator", new Pose2d(new Translation2d(0, 0), headingEstimator.getEstimatedHeading()));
		CommandScheduler.getInstance().run(); // Should be last
	}


	public Command getAutonomousCommand() {
		return autonomousChooser.getChosenValue();
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

	public Solenoid getSolenoid() {
		return solenoid;
	}

	public Funnel getFunnel() {
		return funnel;
	}

	public Intake getIntake() {
		return intake;
	}

	public Elbow getElbow() {
		return elbow;
	}

	public Flywheel getFlywheel() {
		return flywheel;
	}

	public Pivot getPivot() {
		return pivot;
	}

	public Lifter getLifter() {
		return lifter;
	}

	public Roller getRoller() {
		return roller;
	}

	public Wrist getWrist() {
		return wrist;
	}

	public Superstructure getSuperstructureFunny() {
		return superstructureFunny;
	}

	public frc.robot.superstructure.Superstructure getSuperstructureRobot() {
		return superstructureRobot;
	}

	public StatesMotionPlanner getStatesMotionPlanner() {
		return statesMotionPlanner;
	}

	public IPoseEstimator[] getPoseEstimators() {
		return new IPoseEstimator[] {poseEstimator};
	}

	public MultiAprilTagVisionSources[] getAprilTagVisionSources() {
		return new MultiAprilTagVisionSources[] {aprilTagVisionSources};
	}

	public RobotHeadingEstimator getHeadingEstimator() {
		return headingEstimator;
	}

}
