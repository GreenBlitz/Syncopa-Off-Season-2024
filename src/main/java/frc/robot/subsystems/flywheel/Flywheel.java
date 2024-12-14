package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class Flywheel extends GBSubsystem {

	private final ControllableMotor rightMotor;
	private final ControllableMotor leftMotor;
	private final IRequest<Rotation2d> rightFlywheelVelocityRequest;
	private final IRequest<Rotation2d> leftFlywheelVelocityRequest;
	private final FlywheelStuff flywheelStuff;

	private final FlywheelCommandsBuilder commandsBuilder;
	private final SysIdCalibrator rightSysidCalibrator;
	private final SysIdCalibrator leftSysidCalibrator;

//	private final MechanismLigament2d LflywheelLigament2d;
//	private final MechanismLigament2d RflywheelLigament2d;

	public Flywheel(FlywheelStuff flywheelStuff) {
		super(flywheelStuff.logPath());
		this.rightMotor = flywheelStuff.rightFlywheel();
		this.leftMotor = flywheelStuff.leftFlywheel();
		this.rightFlywheelVelocityRequest = flywheelStuff.rightFlywheelVelocityRequest();
		this.leftFlywheelVelocityRequest = flywheelStuff.leftFlywheelVelocityRequest();
		this.flywheelStuff = flywheelStuff;
		this.commandsBuilder = new FlywheelCommandsBuilder(this);
		this.rightSysidCalibrator = new SysIdCalibrator(rightMotor.getSysidConfigInfo(), this, voltage -> setVoltages(voltage, 0));
		this.leftSysidCalibrator = new SysIdCalibrator(leftMotor.getSysidConfigInfo(), this, voltage -> setVoltages(0, voltage));

//		this.LflywheelLigament2d = new MechanismLigament2d("Lflywheel", 4.4, flywheelStuff.leftPosition().getLatestValue().getDegrees(), 5, new Color8Bit(Color.kRed));
//		this.RflywheelLigament2d = new MechanismLigament2d("Rflywheel", 4.4, flywheelStuff.rightPosition().getLatestValue().getDegrees(), 5, new Color8Bit(Color.kOrange));
//
//		Robot.mechanism2d.getRoot("FLYWHEEL", 10, 5).append(LflywheelLigament2d);
//		Robot.mechanism2d.getRoot("FLYWHEEL", 10, 5).append(RflywheelLigament2d);

		updateInputs();
	}

	public FlywheelCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public SysIdCalibrator getRightSysidCalibrator() {
		return rightSysidCalibrator;
	}

	public SysIdCalibrator getLeftSysidCalibrator() {
		return leftSysidCalibrator;
	}

	private void updateInputs() {
		rightMotor.updateInputs(flywheelStuff.rightSignals());
		rightMotor.updateInputs(flywheelStuff.rightVelocitySignal());
//		rightMotor.updateInputs(flywheelStuff.rightPosition());
//		RflywheelLigament2d.setAngle(flywheelStuff.rightPosition().getLatestValue().getDegrees());
		rightMotor.updateSimulation();
		leftMotor.updateInputs(flywheelStuff.leftSignals());
		leftMotor.updateInputs(flywheelStuff.leftVelocitySignal());
//		leftMotor.updateInputs(flywheelStuff.leftPosition());
//		LflywheelLigament2d.setAngle(flywheelStuff.leftPosition().getLatestValue().getDegrees());
		leftMotor.updateSimulation();
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

	protected void stop() {
		rightMotor.stop();
		leftMotor.stop();
	}

	protected void setPowers(double rightPower, double leftPower) {
		rightMotor.setPower(rightPower);
		leftMotor.setPower(leftPower);
	}

	protected void setVoltages(double rightVoltage, double leftVoltage) {
		rightMotor.applyRequest(flywheelStuff.rightVoltageRequest().withSetPoint(rightVoltage));
		leftMotor.applyRequest(flywheelStuff.leftVoltageRequest().withSetPoint(leftVoltage));
	}

	protected void setTargetVelocities(Rotation2d rightFlywheelVelocity, Rotation2d leftFlywheelVelocity) {
		rightMotor.applyRequest(rightFlywheelVelocityRequest.withSetPoint(rightFlywheelVelocity));
		leftMotor.applyRequest(leftFlywheelVelocityRequest.withSetPoint(leftFlywheelVelocity));
	}

	//@formatter:off
	public boolean isAtVelocities(Rotation2d rightFlywheelTargetVelocity, Rotation2d leftFlywheelTargetVelocity, Rotation2d velocityPerSecondTolerance) {
		return isAtVelocities(rightFlywheelTargetVelocity, leftFlywheelTargetVelocity, velocityPerSecondTolerance, velocityPerSecondTolerance);
	}
	//@formatter:on

	public boolean isAtVelocities(
		Rotation2d rightFlywheelTargetVelocity,
		Rotation2d leftFlywheelTargetVelocity,
		Rotation2d rightVelocityPerSecondTolerance,
		Rotation2d leftVelocityPerSecondTolerance
	) {
		boolean rightFlyWheelAtVelocity = MathUtil.isNear(
			rightFlywheelTargetVelocity.getRotations(),
			flywheelStuff.rightVelocitySignal().getLatestValue().getRotations(),
			rightVelocityPerSecondTolerance.getRotations()
		);
		boolean leftFlyWheelAtVelocity = MathUtil.isNear(
			leftFlywheelTargetVelocity.getRotations(),
			flywheelStuff.leftVelocitySignal().getLatestValue().getRotations(),
			leftVelocityPerSecondTolerance.getRotations()
		);
		return rightFlyWheelAtVelocity && leftFlyWheelAtVelocity;
	}

}
