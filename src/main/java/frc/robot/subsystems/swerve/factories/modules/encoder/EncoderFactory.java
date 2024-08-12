package frc.robot.subsystems.swerve.factories.modules.encoder;

import frc.robot.Robot;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.encoder.IEncoder;
import frc.robot.subsystems.swerve.modules.encoder.cancoder.CancoderEncoder;
import frc.robot.subsystems.swerve.modules.encoder.simulation.EmptyEncoder;

public class EncoderFactory {

	public static IEncoder create(SwerveName swerveName, ModuleUtils.ModulePosition modulePosition) {
		return switch (swerveName) {
			case SWERVE -> createSwerveEncoder(modulePosition);
		};
	}

	private static IEncoder createSwerveEncoder(ModuleUtils.ModulePosition modulePosition) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> switch (modulePosition) {
				case FRONT_LEFT -> new CancoderEncoder(IDs.CANCodersIDs.FRONT_LEFT_ENCODER, EncoderRealConstants.ENCODER_CONFIG, SwerveName.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION + modulePosition + "/");
				case FRONT_RIGHT -> new CancoderEncoder(IDs.CANCodersIDs.FRONT_RIGHT_ENCODER, EncoderRealConstants.ENCODER_CONFIG, SwerveName.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION + modulePosition + "/");
				case BACK_LEFT -> new CancoderEncoder(IDs.CANCodersIDs.BACK_LEFT_ENCODER, EncoderRealConstants.ENCODER_CONFIG, SwerveName.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION + modulePosition + "/");
				case BACK_RIGHT -> new CancoderEncoder(IDs.CANCodersIDs.BACK_RIGHT_ENCODER, EncoderRealConstants.ENCODER_CONFIG, SwerveName.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION + modulePosition + "/");
			};
			case SIMULATION, REPLAY -> new EmptyEncoder();
		};
	}

}
