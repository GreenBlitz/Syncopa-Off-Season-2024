package frc;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.constants.field.enums.ReefSide;
import frc.robot.Robot;
import frc.joysticks.keyboard.KeyboardController;

public class KeyboardBindings {

	private static final KeyboardController KEYBOARD_CONTROLLER = new KeyboardController();

	public static void configureBindings(Robot robot) {
		KeyboardController usedKeyboard = KEYBOARD_CONTROLLER;
		// bindings...

		usedKeyboard.A.onTrue(new InstantCommand(() -> CodeCode.reefSide = ReefSide.A));
		usedKeyboard.W.onTrue(new InstantCommand(() -> CodeCode.reefSide = ReefSide.B));
		usedKeyboard.E.onTrue(new InstantCommand(() -> CodeCode.reefSide = ReefSide.C));
		usedKeyboard.D.onTrue(new InstantCommand(() -> CodeCode.reefSide = ReefSide.D));
		usedKeyboard.X.onTrue(new InstantCommand(() -> CodeCode.reefSide = ReefSide.E));
		usedKeyboard.Z.onTrue(new InstantCommand(() -> CodeCode.reefSide = ReefSide.F));
	}

}
