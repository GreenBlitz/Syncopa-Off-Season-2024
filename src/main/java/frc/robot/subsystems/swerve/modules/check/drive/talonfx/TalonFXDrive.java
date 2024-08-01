package frc.robot.subsystems.swerve.modules.check.drive.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.check.drive.DriveMotorInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.check.drive.IDrive;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsContainer;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;
import frc.utils.devicewrappers.TalonFXWrapper;

import java.util.Queue;

public class TalonFXDrive implements IDrive {

    private final TalonFXWrapper driveMotor;
    private final TalonFXDriveSignals signals;

    private final VelocityVoltage velocityVoltageRequest;
    private final VoltageOut voltageRequest;

    private final Queue<Double> drivePositionQueue;

    public TalonFXDrive(TalonFXDriveConstants constants) {
        this.driveMotor = constants.getDriveMotor();
        this.signals = constants.getSignals();

        this.velocityVoltageRequest = new VelocityVoltage(0).withEnableFOC(constants.getEnableFOC());
        this.voltageRequest = new VoltageOut(0).withEnableFOC(constants.getEnableFOC());

        this.drivePositionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(
                driveMotor,
                signals.positionSignal(),
                signals.velocitySignal()
        );

        driveMotor.setPosition(0);
    }


    @Override
    public void setBrake(boolean brake) {
        NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.setNeutralMode(neutralModeValue);
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
    }

    @Override
    public void runMotorByVoltage(double voltage) {
        driveMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setTargetClosedLoopVelocity(Rotation2d velocityPerSecond) {
        driveMotor.setControl(velocityVoltageRequest.withVelocity(velocityPerSecond.getRotations()));
    }

    @Override
    public void updateInputs(ModuleInputsContainer inputs) {
        DriveMotorInputsAutoLogged driveInputs = inputs.getDriveMotorInputs();
        driveInputs.isConnected = BaseStatusSignal.refreshAll(
                signals.positionSignal(),
                signals.velocitySignal(),
                signals.accelerationSignal(),
                signals.voltageSignal(),
                signals.statorCurrentSignal()
        ).isOK();
        driveInputs.angle = Rotation2d.fromRotations(driveMotor.getLatencyCompensatedPosition());
        driveInputs.velocity = Rotation2d.fromRotations(driveMotor.getLatencyCompensatedVelocity());
        driveInputs.acceleration = Rotation2d.fromRotations(signals.accelerationSignal().getValue());
        driveInputs.current = signals.statorCurrentSignal().getValue();
        driveInputs.voltage = signals.voltageSignal().getValue();
        driveInputs.angleOdometrySamples = drivePositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        drivePositionQueue.clear();
    }

}
