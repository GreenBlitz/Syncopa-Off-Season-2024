// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.proto.Pose2dProto;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.hardware.phoenix6.BusChain;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtils;
import frc.utils.battery.BatteryUtils;
import frc.utils.time.TimeUtils;
import frc.utils.logger.LoggerFactory;
import org.littletonrobotics.junction.LoggedRobot;
import frc.utils.brakestate.BrakeStateManager;
import org.littletonrobotics.junction.Logger;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in
 * the project.
 */
public class RobotManager extends LoggedRobot {

	private int roborioCycles;

	private Command autonomousCommand;
	private Robot robot;

	@Override
	public void robotInit() {
		LoggerFactory.initializeLogger();
		PathPlannerUtils.startPathfinder();
		BatteryUtils.scheduleLimiter();
		this.roborioCycles = 0;

		Robot.mechanism2d.getRoot("ELBOW_ROOT", 10, 0);
		Logger.recordOutput("target", Rotation2d.fromDegrees(50));
		Pose2d pose2d = new Pose2d(2, 2, new Rotation2d());
		Pose2d pose2d1 = new Pose2d();
		Translation2d translation2d = new Translation2d();
//		Robot.mechanism2d.getRoot("ROBOT", 2,2).append(robot.getElbow().);


		this.robot = new Robot();
	}

	@Override
	public void testInit() {
		robot.getElbow().getCommandsBuilder().moveToAngle(Rotation2d.fromDegrees(-30)).schedule();
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtils.isMatch()) {
			BrakeStateManager.coast();
		}
	}

	@Override
	public void disabledExit() {
		if (!DriverStationUtils.isMatch()) {
			BrakeStateManager.brake();
		}
	}

	@Override
	public void autonomousInit() {
		this.autonomousCommand = robot.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
		robot.getElbow().getCommandsBuilder().moveToAngle(Rotation2d.fromDegrees(-70)).schedule();
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
//		robot.getElbow().getCommandsBuilder().moveToAngle(Rotation2d.fromDegrees(10)).schedule();
		robot.getElbow().getCommandsBuilder().moveToAngle(Rotation2d.fromDegrees(60)).schedule();
//		robot.getElbow().getCommandsBuilder().setPower(() -> 1).schedule();
//		robot.getFlywheel().getCommandsBuilder().setVelocities(Rotation2d.fromRotations(10), Rotation2d.fromRotations(10)).schedule();
	}

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
		CommandScheduler.getInstance().run();
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		AlertManager.reportAlerts();

		Logger.recordOutput("PivotPose", robot.getPivot().getSimulationPivotPosition3d());
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtils.updateCycleTime(roborioCycles);
	}

}
