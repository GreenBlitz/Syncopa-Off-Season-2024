package frc.robot.subsystems.jointSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.subsystems.GBSubsystem;

import java.util.Arrays;

public class Joint extends GBSubsystem {

	private final ControllableMotor motor;
	private final JointCommandsBuilder commandsBuilder;
	private final InputSignal<Rotation2d> positionSignal;
	private final InputSignal[] otherSignals;
	private final IRequest<Rotation2d> positionRequest;

	public Joint(
		String logPath,
		ControllableMotor motor,
		IRequest<Rotation2d> positionRequest,
		InputSignal<Rotation2d> positionSignal,
		InputSignal... otherSignals
	) {
		super(logPath);
		this.motor = motor;
		this.positionRequest = positionRequest;
		this.positionSignal = positionSignal;
		this.otherSignals = otherSignals;
		this.commandsBuilder = new JointCommandsBuilder(this);
	}

	public JointCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs(positionSignal);
		updateInputs(otherSignals);
	}

	private void updateInputs(InputSignal... signals) {
		motor.updateSignals(signals);
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	protected void stop(){
		motor.stop();
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void stayInPlace() {
		setTargetPosition(positionSignal.getLatestValue());
	}

	protected void setTargetPosition(Rotation2d position) {
		motor.applyAngleRequest(positionRequest.withSetPoint(position));
	}

	public boolean isAtPosition(Rotation2d position, Rotation2d tolerance) {
		return MathUtil.isNear(position.getDegrees(), positionSignal.getLatestValue().getDegrees(), tolerance.getDegrees());
	}

}
