package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.jointSubsystem.Joint;

public class WristStateHandler {

	private final Joint wrist;

	public WristStateHandler(Joint wrist) {
		this.wrist = wrist;
	}

	public Command setState(WristState state) {
		return wrist.getCommandsBuilder().moveToPosition(state.getPosition());
	}

}
