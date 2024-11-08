package frc.robot.subsystems.rollerSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class RollerSubsystem extends GBSubsystem {

	private final IMotor motor;
	private final RollerSubsystemCommandsBuilder commandsBuilder;
	private final InputSignal<Rotation2d> positionSignal;
	private final InputSignal[] otherSignals;

	public RollerSubsystem(String logPath, IMotor motor, InputSignal<Rotation2d> positionSignal, InputSignal... otherSignals) {
		super(logPath);
		this.motor = motor;
		this.positionSignal = positionSignal;
		this.otherSignals = otherSignals;
		this.commandsBuilder = new RollerSubsystemCommandsBuilder(this);
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs(positionSignal);
		updateInputs(otherSignals);
	}

	private void updateInputs(InputSignal... signals) {
		motor.updateSignals(signals);
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void stop() {
		motor.stop();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public Rotation2d getPosition() {
		return positionSignal.getLatestValue();
	}

}
