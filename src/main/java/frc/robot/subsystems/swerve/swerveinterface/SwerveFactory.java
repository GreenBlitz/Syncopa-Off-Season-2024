package frc.robot.subsystems.swerve.swerveinterface;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.swerve.mk4iswerve.MK4ISwerve;
import frc.robot.subsystems.swerve.replayswerve.ReplaySwerve;
import frc.robot.subsystems.swerve.simulationswerve.SimulationSwerve;
import frc.utils.RobotTypeUtils;

public class SwerveFactory {

    public static ISwerve createSwerve() {
        return switch (RobotConstants.ROBOT_TYPE) {
            case REAL -> new MK4ISwerve();
            case REPLAY -> new ReplaySwerve();
            case SIMULATION -> new SimulationSwerve();
        };
    }

}
