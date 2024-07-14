package frc.robot.subsystems.swerve.gyro.factory;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2GyroConfigObject;
import frc.robot.constants.IDs;

public class Pigeon2GyroContainer {

    private static final Pigeon2Configuration PIGEON_2_CONFIGURATION = new Pigeon2Configuration();
    static {
        PIGEON_2_CONFIGURATION.MountPose.MountPoseRoll = 180;
        PIGEON_2_CONFIGURATION.MountPose.MountPosePitch = 0;
        PIGEON_2_CONFIGURATION.MountPose.MountPoseYaw = 0;
    }

    protected static final Pigeon2GyroConfigObject pigeon2GyroConfigObject = new Pigeon2GyroConfigObject(
            IDs.PIGEON_2_DEVICE_ID,
            PIGEON_2_CONFIGURATION
    );

}
