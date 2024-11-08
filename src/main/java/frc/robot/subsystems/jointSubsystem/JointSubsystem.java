package frc.robot.subsystems.jointSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class JointSubsystem extends GBSubsystem {

	private final ControllableMotor motor;
	private final JointCommandsBuilder commandsBuilder;
	private final InputSignal<Rotation2d> positionSignal;
	private final InputSignal[] otherSignals;
	private final IRequest<Rotation2d> positionRequest;

	public JointSubsystem(
		String logPath,
		ControllableMotor motor,
		IRequest<Rotation2d> positionRequest,
		InputSignal<Rotation2d> positionSignal,
		InputSignal... otherSignals
	) {
		super(logPath);
		this.motor = motor;
		this.commandsBuilder = new JointCommandsBuilder(this);
		this.positionRequest = positionRequest;
		this.positionSignal = positionSignal;
		this.otherSignals = otherSignals;
	}

	public JointCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

	private void updateInputs() {
		for (InputSignal<?> signal : otherSignals) {
			motor.updateSignals(signal);
		}
		motor.updateSignals(positionSignal);
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void stayInPlace() {
		setTargetAngle(positionSignal.getLatestValue());
	}

	protected void setTargetAngle(Rotation2d angle) {
		motor.applyAngleRequest(positionRequest.withSetPoint(angle));
	}

	public boolean isAtAngle(Rotation2d angle, Rotation2d tolerance) {
		return MathUtil.isNear(angle.getDegrees(), positionSignal.getLatestValue().getDegrees(), tolerance.getDegrees());
	}

}
