package frc.robot.subsystems.rollerSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;

public class RollerSubsystemCommandsBuilder {

	private final RollerSubsystem rollerSubsystem;

	public RollerSubsystemCommandsBuilder(RollerSubsystem rollerSubsystem) {
		this.rollerSubsystem = rollerSubsystem;
	}

	public Command setPower(double power) {
		return new RunCommand(() -> rollerSubsystem.setPower(power), rollerSubsystem).withName("Set power: " + power);
	}

	public Command setPower(DoubleSupplier doubleSupplier) {
		return new RunCommand(() -> rollerSubsystem.setPower(doubleSupplier.getAsDouble()), rollerSubsystem).withName("Set power by supplier");
	}

	public Command stop() {
		return new RunCommand(rollerSubsystem::stop, rollerSubsystem).withName("Stop");
	}

	public Command rollRotations(Rotation2d rotations, double power) {
		Rotation2d startingPosition = rollerSubsystem.getPosition();
		return new FunctionalCommand(
			() -> {},
			() -> rollerSubsystem.setPower(power),
			interrupted -> rollerSubsystem.stop(),
			() -> Math.abs(rollerSubsystem.getPosition().getRotations() - startingPosition.getRotations()) > rotations.getRotations(),
			rollerSubsystem
		).withName("Rotate rotations: " + rotations);
	}

}
