package frc.robot.poseestimator.limelights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class Limelight extends GBSubsystem {

	private NetworkTableEntry robotPoseEntry, idEntry, tagPoseEntry;
	private String name;
	private double[] poseArray;

	public Limelight(String name, String hardwareLogPath) {
		super(hardwareLogPath + VisionConstants.LIMELIGHT_LOGPATH_PREFIX + name + "/");

		this.name = name;
		this.robotPoseEntry = getLimelightNetworkTableEntry("botpose_wpiblue");
		this.tagPoseEntry = getLimelightNetworkTableEntry("targetpose_cameraspace");
		this.idEntry = getLimelightNetworkTableEntry("tid");


		System.out.println(name + " exists");
		Logger.recordOutput(name + "Exists");
	}

	public Optional<Pair<Pose2d, Double>> getUpdatedPose2DEstimation() {
		int id = (int) idEntry.getInteger(-1);
		if (id == -1) {
			return Optional.empty();
		}

		poseArray = robotPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);

		double processingLatencySeconds = poseArray[LimelightEntryValue.TOTAL_LATENCY.getIndex()] / 1000;
		double timestamp = Timer.getFPGATimestamp() - processingLatencySeconds;

		Logger.recordOutput(super.getLogPath() + timestamp, poseArray);

		Pose2d robotPose = new Pose2d(
			poseArray[LimelightEntryValue.X_AXIS.getIndex()],
			poseArray[LimelightEntryValue.Y_AXIS.getIndex()],
			Rotation2d.fromDegrees(poseArray[LimelightEntryValue.PITCH_ANGLE.getIndex()])
		);
		Logger.recordOutput(super.getLogPath() + "test", robotPose);
		return Optional.of(new Pair<>(robotPose, timestamp));
	}

	public double getAprilTagHeight() {
		return poseArray[LimelightEntryValue.Y_AXIS.getIndex()];
	}

	public double getDistanceFromAprilTag() {
		return poseArray[LimelightEntryValue.Z_AXIS.getIndex()];
	}

	private NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
		return NetworkTableInstance.getDefault().getTable(name).getEntry(entryName);
	}

	@Override
	protected void subsystemPeriodic() {}


}
