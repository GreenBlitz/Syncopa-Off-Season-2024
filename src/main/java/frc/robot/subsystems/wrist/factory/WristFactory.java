package frc.robot.subsystems.wrist.factory;

import frc.robot.Robot;
import frc.robot.subsystems.jointSubsystem.Joint;

public class WristFactory {

	public static Joint create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealWristConstants.generateJointWrist(logPath);
			case SIMULATION -> null;
		};
	}

}
