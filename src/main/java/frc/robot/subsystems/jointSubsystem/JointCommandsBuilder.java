package frc.robot.subsystems.jointSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;

public class JointCommandsBuilder {

	private final JointSubsystem jointSubsystem;

	public JointCommandsBuilder(JointSubsystem jointSubsystem) {
		this.jointSubsystem = jointSubsystem;
	}

	public Command setPower(double power) {
		return new RunCommand(() -> jointSubsystem.setPower(power), jointSubsystem).withName("Set power");
	}

	public Command setPower(DoubleSupplier doubleSupplier) {
		return new RunCommand(() -> jointSubsystem.setPower(doubleSupplier.getAsDouble()), jointSubsystem).withName("Set power by supplier");
	}

	public Command stayInPlace() {
		return new RunCommand(jointSubsystem::stayInPlace, jointSubsystem).withName("Stay in place");
	}

}
