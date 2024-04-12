package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.commands.InitExecuteCommand;

import java.util.function.DoubleSupplier;

import static frc.robot.RobotContainer.SWERVE;

public class SwerveCommands {

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed
     * open mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getOpenLoopFieldRelativeDriveCommand(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier
    ) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(false),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
    }
}
