package frc.robot.subsystems.jointSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class JointCommandsBuilder {

	private final Joint joint;

	public JointCommandsBuilder(Joint joint) {
		this.joint = joint;
	}

	public Command setPower(double power) {
		return new RunCommand(() -> joint.setPower(power), joint).withName("Set power: " + power);
	}

	public Command setPower(DoubleSupplier doubleSupplier) {
		return new RunCommand(() -> joint.setPower(doubleSupplier.getAsDouble()), joint).withName("Set power by supplier");
	}

	public Command stop(){
		return new RunCommand(joint::stop).withName("Stop");
	}

	public Command moveToPosition(Rotation2d position){
		return new RunCommand(() -> joint.setTargetPosition(position), joint).withName("Move to position: " + position);
	}

	public Command moveToPosition(Supplier<Rotation2d> positionSupplier){
		return new RunCommand(() -> joint.setTargetPosition(positionSupplier.get()), joint).withName("Move to position by supplier");
	}

	public Command stayInPlace() {
		return new RunCommand(joint::stayInPlace, joint).withName("Stay in place");
	}

}
