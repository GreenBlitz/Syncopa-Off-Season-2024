package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;

public class IntakeCommandsBuilder {

	private final Intake intake;

	public IntakeCommandsBuilder(Intake intake) {
		this.intake = intake;
	}

	//@formatter:off
	public Command setPower(double power) {
		return new FunctionalCommand(
				() -> {},
				() -> intake.setPower(power),
				interrupted -> intake.stop(),
				() -> false,
				intake
		).withName("Set power to: " + power);
	}

	public Command setPower(DoubleSupplier power) {
		return new FunctionalCommand(
				() -> {},
				() -> intake.setPower(power.getAsDouble()),
				interrupted -> intake.stop(),
				() -> false,
				intake
		).withName("Set power by supplier");
	}

	public Command stop() {
		return new RunCommand(intake::stop, intake).withName("Stop");
	}
	//@formatter:on

}
