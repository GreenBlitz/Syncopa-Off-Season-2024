package frc.robot.subsystems.pivot.factory;

import frc.robot.Robot;
import frc.robot.subsystems.jointSubsystem.Joint;
import frc.robot.subsystems.pivot.PivotStuff;

public class PivotFactory {

	public static Joint create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealPivotConstants.generateJointPivot(logPath);
			case SIMULATION -> null;
		};
	}

}
