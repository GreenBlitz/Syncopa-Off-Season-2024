package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.joysticks.SmartJoystick;

public class EndBehaviorManager {

	private final Superstructure superstructure;

	public EndBehaviorManager(Superstructure superstructure) {
		this.superstructure = superstructure;
	}

	public Command endState(RobotState state, SmartJoystick joystick) {
		return switch (state) {
			case FEED, ALIGN_REEF, INTAKE, ARM_INTAKE, SPEAKER, TRANSFER_SHOOTER_TO_ARM, TRANSFER_ARM_TO_SHOOTER, PASSING, FEEDER_INTAKE ->
				superstructure.setState(RobotState.IDLE, joystick);
			case INTAKE_WITH_FLYWHEEL -> superstructure.setState(RobotState.PRE_SPEAKER, joystick);
			case PRE_SCORE_REEF, AMP, SHOOT_L2 -> superstructure.setState(RobotState.ARM_UP, joystick);
			case INTAKE_OUTTAKE, ARM_OUTTAKE, PRE_AMP, PRE_SPEAKER, IDLE, ARM_UP -> superstructure.setState(state, joystick);
		};
	}

}
