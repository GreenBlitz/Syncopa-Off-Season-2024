package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WristStateHandler {

	private final Wrist wrist;

	public WristStateHandler(Wrist wrist) {
		this.wrist = wrist;
	}

	public Command setState(WristState state) {
		return new InstantCommand();
//		return wrist.getCommandsBuilder().moveToPosition(state.getPosition());
	}

}
