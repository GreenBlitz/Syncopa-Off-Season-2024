package frc.robot.subsystems.lifter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

public class Lifter extends GBSubsystem {

	private final ControllableMotor motor;
	private final LifterComponents lifterComponents;
	private final LifterCommandsBuilder lifterCommandsBuilder;

	public Lifter(LifterComponents lifterComponents) {
		super(lifterComponents.logPath());
		this.motor = lifterComponents.motor();
		this.lifterComponents = lifterComponents;
		this.lifterCommandsBuilder = new LifterCommandsBuilder(this);
		motor.resetPosition(new Rotation2d());

		updateInputs();
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void stop() {
		motor.stop();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public boolean isHigher(double expectedPositionMeters) {
		return expectedPositionMeters < convertToMeters(lifterComponents.positionSignal().getLatestValue());
	}

	public boolean isLower(double expectedPositionMeters) {
		return !isHigher(expectedPositionMeters);
	}

	public LifterComponents getLifterComponents() {
		return lifterComponents;
	}

	public LifterCommandsBuilder getCommandsBuilder() {
		return lifterCommandsBuilder;
	}

	@Override
	protected void subsystemPeriodic() {
		if (LifterConstants.MINIMUM_ACHIVEABLE_POSITION_METERS > convertToMeters(lifterComponents.positionSignal().getLatestValue())) {
			motor.resetPosition(convertFromMeters(LifterConstants.MINIMUM_ACHIVEABLE_POSITION_METERS));
		}

		updateInputs();
	}

	private void updateInputs() {
		motor.updateSignals(lifterComponents.positionSignal());
		motor.updateSignals(lifterComponents.otherSignals());

		Logger.recordOutput("lifter position in meters", convertToMeters(lifterComponents.positionSignal().getLatestValue()));
	}

	private double convertToMeters(Rotation2d motorPosition) {
		return Conversions.angleToDistance(motorPosition, lifterComponents.drumRadius());
	}

	private Rotation2d convertFromMeters(double mechanismPosition) {
		return Conversions.distanceToAngle(mechanismPosition, lifterComponents.drumRadius());
	}

}

