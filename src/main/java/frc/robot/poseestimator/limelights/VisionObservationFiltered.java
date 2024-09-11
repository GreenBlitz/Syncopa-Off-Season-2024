package frc.robot.poseestimator.limelights;

import frc.robot.constants.Field;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;

public class VisionObservationFiltered extends GBSubsystem {

    private LimelightRawData limelightHardware;
    private FilteredLimelightsConfig config;

    public VisionObservationFiltered(FilteredLimelightsConfig config) {
        super(config.logPath());

        System.out.println("filteredCreated");
        this.limelightHardware = new LimelightRawData(config.limelightsNames(), config.hardwareLogPath());
        this.config = config;
    }

    public List<VisionObservation> getFilteredVisionObservations() {
        ArrayList<VisionObservation> estimates = new ArrayList<>();

        for (LimelightData limelightData : limelightHardware.getAllAvailableLimelightData()) {
            System.out.println(super.getLogPath() + limelightData.timeStamp() + " pose " + limelightData.EstimatedPosition());
            Logger.recordOutput(super.getLogPath() + limelightData.timeStamp(), limelightData.AprilTagHeight());
            if (!filterOutLimelightData(limelightData)) {
                double standardDeviation = getDynamicStandardDeviations(limelightData);
                double[] standardDeviations = new double[]{standardDeviation};

                estimates
                        .add(new VisionObservation(limelightData.EstimatedPosition(), standardDeviations, limelightData.timeStamp()));
            }
        }

        return estimates;
    }

    private boolean isLimelightedOutputInTolerance(LimelightData limelightData) {
        return true;
//		Pose2d currentPoseObservation = NetworkTables...;

//		Pose2d limelightPosition = limelightData.EstimatedPosition();
//		Transform2d transformDifference = limelightPosition.minus(currentPoseObservation);
//		Rotation2d rotationDifference = limelightPosition.getRotation().minus(currentPoseObservation.getRotation());
//
//		return transformDifference.getTranslation().getNorm() <= config.positionNormTolerance()
//			&& rotationDifference.getDegrees() <= config.rotationTolerance().getDegrees();
    }

    private boolean isAprilTagInProperHeight(LimelightData limelightData) {
        boolean aprilTagHeightConfidence = Math.abs(limelightData.AprilTagHeight() - Field.APRIL_TAG_HEIGHT_METERS)
                < VisionConstants.APRIL_TAG_HEIGHT_TOLERANCE_METERS;
        return aprilTagHeightConfidence;
    }

    private boolean filterOutLimelightData(LimelightData limelightData) {
        return !(isAprilTagInProperHeight(limelightData) && isLimelightedOutputInTolerance(limelightData));
    }

    public void logEstimatedPositions() {
        List<VisionObservation> observations = getFilteredVisionObservations();

        for (int i = 0; i < observations.size(); i++) {
            System.out.println("est" + i);

            Logger.recordOutput(
                    super.getLogPath() + VisionConstants.ESTIMATION_LOGPATH_PREFIX + i,
                    observations.get(i).visionPose()
            );
        }
    }

    private double getDynamicStandardDeviations(LimelightData limelightData) {
        return limelightData.DistanceFromAprilTag() / VisionConstants.APRIL_TAG_DiSTANCE_TO_STANDARD_DEVIATIONS_FACTOR;
    }

    public VisionObservation getFirstAvailableTarget() {
        return getFilteredVisionObservations().get(0);
    }

    @Override
    protected void subsystemPeriodic() {
    }

}
