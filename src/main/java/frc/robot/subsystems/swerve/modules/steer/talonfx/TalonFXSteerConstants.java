package frc.robot.subsystems.swerve.modules.steer.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXSteerConstants {

	protected static final int NO_ENCODER_ID = -1;

	private final TalonFXWrapper motor;
	private final TalonFXSteerSignals signals;
	private final boolean enableFOC;
	private final SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo;

	public TalonFXSteerConstants(
		CTREDeviceID steerMotorID,
		boolean inverted,
		TalonFXConfiguration configuration,
		boolean enableFOC,
		SysIdRoutine.Config sysIdConfig
	) {
		this(steerMotorID, inverted, NO_ENCODER_ID, configuration, enableFOC, sysIdConfig);
	}

	public TalonFXSteerConstants(
		CTREDeviceID steerMotorID,
		boolean inverted,
		int encoderID,
		TalonFXConfiguration configuration,
		boolean enableFOC,
		SysIdRoutine.Config sysIdConfig
	) {
		TalonFXSteerConfigObject steerConfigObject = new TalonFXSteerConfigObject(steerMotorID, inverted, encoderID, configuration);
		this.motor = steerConfigObject.getMotor();
		this.signals = steerConfigObject.getSignals();
		this.enableFOC = enableFOC;
		this.sysIdConfigInfo = new SysIdCalibrator.SysIdConfigInfo(sysIdConfig, true);
	}


	protected TalonFXWrapper getMotor() {
		return motor;
	}

	protected TalonFXSteerSignals getSignals() {
		return signals;
	}

	protected boolean getEnableFOC() {
		return enableFOC;
	}

	protected SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return sysIdConfigInfo;
	}

}
