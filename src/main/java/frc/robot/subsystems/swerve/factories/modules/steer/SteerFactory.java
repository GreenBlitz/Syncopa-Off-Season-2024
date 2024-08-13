package frc.robot.subsystems.swerve.factories.modules.steer;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.steer.ISteer;
import frc.robot.subsystems.swerve.modules.steer.replay.EmptySteer;
import frc.robot.subsystems.swerve.modules.steer.simulation.SimulationSteer;
import frc.robot.subsystems.swerve.modules.steer.talonfx.TalonFXSteer;

public class SteerFactory {

	public static ISteer create(SwerveName swerveName, ModuleUtils.ModulePosition modulePosition) {
		return switch (swerveName) {
			case SWERVE -> createSwerveSteer(modulePosition);
		};
	}

	private static ISteer createSwerveSteer(ModuleUtils.ModulePosition modulePosition) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> switch (modulePosition) {
				case FRONT_LEFT ->
					new TalonFXSteer(
						SteerRealConstants.FRONT_LEFT_CONSTANTS(SwerveName.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION)
					);
				case FRONT_RIGHT ->
					new TalonFXSteer(
						SteerRealConstants.FRONT_RIGHT_CONSTANTS(SwerveName.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION)
					);
				// @formatter:off
				case BACK_LEFT ->
					new TalonFXSteer(
						SteerRealConstants.BACK_LEFT_CONSTANTS(SwerveName.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION)
					);
				// @formatter:on
				case BACK_RIGHT ->
					new TalonFXSteer(
						SteerRealConstants.BACK_RIGHT_CONSTANTS(SwerveName.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION)
					);
			};
			case SIMULATION -> new SimulationSteer(SteerSimulationConstants.getConstants());
			case REPLAY -> new EmptySteer();
		};
	}

}
