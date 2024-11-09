package frc.robot.subsystems.elbow;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.jointSubsystem.Joint;

public class ElbowStateHandler {

	private final Joint elbow;

	public ElbowStateHandler(Joint elbow) {
		this.elbow = elbow;
	}

	public Command setState(ElbowState elbowState) {
		if (elbowState == ElbowState.MANUAL) {
			return new InstantCommand();
		}
		return elbow.getCommandsBuilder().moveToPosition(elbowState.getTargetPosition());
	}

}
