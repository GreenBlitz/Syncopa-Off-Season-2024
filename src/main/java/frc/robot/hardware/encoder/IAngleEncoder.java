package frc.robot.hardware.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.IDevice;

public interface IAngleEncoder extends IDevice {

	void setPosition(Rotation2d position);

}
