package frc.robot.subsystems.swerve.factories.modules.steer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.modules.steer.talonfx.TalonFXSteerConstants;

class RealSteerConstants {

    private static final boolean ENABLE_FOC = true;

    private static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration();
    static {
        MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 30;
        MOTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

        MOTOR_CONFIG.Feedback.RotorToSensorRatio = 150.0 / 7.0;
        MOTOR_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        MOTOR_CONFIG.Slot0.kS = 0.19648;
        MOTOR_CONFIG.Slot0.kV = 2.5763;
        MOTOR_CONFIG.Slot0.kA = 0.50361;
        MOTOR_CONFIG.Slot0.kP = 88;
        MOTOR_CONFIG.Slot0.kI = 0;
        MOTOR_CONFIG.Slot0.kD = 1.5;
        MOTOR_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;
    }

    protected static final TalonFXSteerConstants FRONT_LEFT_CONSTANTS = new TalonFXSteerConstants(
            IDs.TalonFXIDs.FRONT_LEFT_STEER_MOTOR,
            true,
            IDs.CANCodersIDs.FRONT_LEFT_ENCODER.ID(),
            MOTOR_CONFIG,
            ENABLE_FOC
    );

    protected static final TalonFXSteerConstants FRONT_RIGHT_CONSTANTS = new TalonFXSteerConstants(
            IDs.TalonFXIDs.FRONT_RIGHT_STEER_MOTOR,
            true,
            IDs.CANCodersIDs.FRONT_RIGHT_ENCODER.ID(),
            MOTOR_CONFIG,
            ENABLE_FOC
    );

    protected static final TalonFXSteerConstants BACK_LEFT_CONSTANTS = new TalonFXSteerConstants(
            IDs.TalonFXIDs.BACK_LEFT_STEER_MOTOR,
            false,
            IDs.CANCodersIDs.BACK_LEFT_ENCODER.ID(),
            MOTOR_CONFIG,
            ENABLE_FOC
    );

    protected static final TalonFXSteerConstants BACK_RIGHT_CONSTANTS = new TalonFXSteerConstants(
            IDs.TalonFXIDs.BACK_RIGHT_STEER_MOTOR,
            true,
            IDs.CANCodersIDs.BACK_RIGHT_ENCODER.ID(),
            MOTOR_CONFIG,
            ENABLE_FOC
    );

}
