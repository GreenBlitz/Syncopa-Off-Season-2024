// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Field;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.poseestimator.GBPoseEstimator;
import frc.robot.poseestimator.PoseEstimatorConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import frc.robot.vision.limelights.LimeLightConstants;
import frc.robot.vision.limelights.LimelightFilterer;
import frc.robot.vision.limelights.LimelightFiltererConfig;
import frc.robot.vision.limelights.MultiLimelights;
import frc.utils.auto.AutonomousChooser;
import frc.utils.auto.PathPlannerUtils;

import java.util.Arrays;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private AutonomousChooser autonomousChooser;

	private final Swerve swerve;
	private final GBPoseEstimator poseEstimator;
	private final LimelightFilterer limelightFilterer;
	private final MultiLimelights multiLimelights;

	private BrushlessSparkMAXMotor motor;

	public Robot() {
		SparkMaxWrapper wrapper = new SparkMaxWrapper(new SparkMaxDeviceID(11, CANSparkLowLevel.MotorType.kBrushless));
		motor = new BrushlessSparkMAXMotor("intake", wrapper, new SysIdRoutine.Config());

		this.swerve = new Swerve(
			SwerveConstantsFactory.create(SwerveType.SWERVE),
			ModulesFactory.create(SwerveType.SWERVE),
			GyroFactory.create(SwerveType.SWERVE)
		);
		swerve.updateStatus();

		multiLimelights = new MultiLimelights(LimeLightConstants.LIMELIGHT_NAMES, "limelightsHardware");
		limelightFilterer = new LimelightFilterer(
			new LimelightFiltererConfig("limelightfilterer", Field.APRIL_TAGS_AVERAGE_HEIGHT_METERS),
			multiLimelights
		);
		this.poseEstimator = new GBPoseEstimator(
			swerve::setHeading,
			"PoseEstimator/",
			limelightFilterer,
			swerve.getConstants().kinematics(),
			swerve.getModules().getWheelsPositions(0),
			swerve.getAbsoluteHeading(),
			PoseEstimatorConstants.DEFAULT_ODOMETRY_STANDARD_DEVIATIONS,
			new Pose2d()
		);

		swerve.configPathPlanner(poseEstimator::getEstimatedPose, pose2d -> {});
		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.setStateHelper(new SwerveStateHelper(() -> Optional.of(poseEstimator.getEstimatedPose()), Optional::empty, swerve));

		configPathPlanner();
		configureBindings();
	}

	public void periodic() {
		swerve.updateStatus();
		poseEstimator.updatePoseEstimator(Arrays.asList(swerve.getAllOdometryObservations()), limelightFilterer.getFilteredVisionObservations());
	}

	private void configPathPlanner() {
		// Register commands...
		PathPlannerUtils
			.registerCommand("Intake", new FunctionalCommand(() -> {}, () -> motor.setPower(0.6), interrupted -> motor.stop(), () -> false));
		PathPlannerUtils.registerCommand(
			"Intake",
			new FunctionalCommand(() -> {}, () -> motor.setPower(-0.6), interrupted -> motor.stop(), () -> false).withTimeout(10)
		);
		swerve.configPathPlanner(poseEstimator::getEstimatedPose, poseEstimator::resetPose);
		autonomousChooser = new AutonomousChooser("Autonomous Chooser");
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings(this);
	}


	public Command getAutonomousCommand() {
		return autonomousChooser.getChosenValue();
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public GBPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}
}
