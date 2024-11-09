package frc.robot.subsystems.elbow.factory;

import frc.robot.Robot;
import frc.robot.subsystems.jointSubsystem.Joint;

public class ElbowFactory {

	public static Joint create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealElbowConstants.generateJointElbow(logPath);
			case SIMULATION -> null;
		};
	}

}
