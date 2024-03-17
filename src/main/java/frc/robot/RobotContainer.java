// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeStartWithSensorCmd;
import frc.robot.commands.ShooterPositionCmd;
import frc.robot.commands.ShooterStartWithSensorCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TransferContinueWithDriverCmd;
import frc.robot.commands.TransferStartWithSensorCmd;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterPosition;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class RobotContainer {
  //public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  public final ShooterSubsystem shooterSubsysPtem = new ShooterSubsystem();
  private final TransferSubsystem transferSubsystem = new TransferSubsystem();
  private final Intake intake = new Intake();
  private final ShooterPosition shooterPosition = new ShooterPosition();

  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  private final JoystickButton stopSwerve = new JoystickButton(driverJoytick,
      Constants.JoystickControlConstans.StopSwerveButton);
  private final JoystickButton zeroGyro = new JoystickButton(driverJoytick,
      Constants.JoystickControlConstans.ZeroGyroButton);

  private final JoystickButton shootButton = new JoystickButton(driverJoytick,
      Constants.JoystickControlConstans.ShootButton);

  DigitalInput sensorShooter = new DigitalInput(0); // Limit switch on DIO 3
  Trigger intakeTriger = new Trigger(sensorShooter::get);

  private IntakeStartWithSensorCmd intakeCommand = new IntakeStartWithSensorCmd(intake, shooterSubsysPtem,
      transferSubsystem);

  DigitalInput sensorTransfer = new DigitalInput(1); // Limit switch on DIO 3
  Trigger shooterTriger = new Trigger(sensorTransfer::get);

  public ShooterStartWithSensorCmd shooterCommand = new ShooterStartWithSensorCmd(shooterSubsysPtem, transferSubsystem);

  DigitalInput sensorIntake = new DigitalInput(2); // Limit switch on DIO 3
  Trigger transferTriger = new Trigger(sensorTransfer::get);

  private TransferStartWithSensorCmd transferCommand = new TransferStartWithSensorCmd(transferSubsystem);

  private TransferContinueWithDriverCmd transferContinueCommand = new TransferContinueWithDriverCmd(transferSubsystem);

  private ShooterPositionCmd shooterPositionCmd = new ShooterPositionCmd(shooterPosition);

  public RobotContainer() {
    // SILMEYIN SAKIN
    Preferences.initDouble("swerveX", 0);
    Preferences.initDouble("swerveY", 0);
    Preferences.initDouble("swerveRot", 0);

    Preferences.initDouble("swerveP", 0.25);
    Preferences.initDouble("swerveI", 0.03);
    Preferences.initDouble("swerveD", 0.01);
    // SILMEYIN SAKIN

   /* swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx))); */

    // configureSensorBindings();

    configureButtonBindings();
  }

  private void configureSensorBindings() {
    /*
     * intakeTriger.toggleOnFalse(intakeCommand);
     * shooterTriger.toggleOnTrue(shooterCommand);
     * transferTriger.toggleOnTrue(transferCommand);
     */
  }

  private void configureButtonBindings() {

   /* zeroGyro.onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    stopSwerve.onTrue(new InstantCommand(() -> swerveSubsystem.stopModules()));*/

    shootButton.onTrue(shooterPositionCmd);
  }

  public Command getAutonomousCommand() {
  /*  // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, 0),
            new Translation2d(1, -1)),
        new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> swerveSubsystem.stopModules()));*/
        return null;
  }

  public Joystick getDriverJoytick() {
    return driverJoytick;
  }

  public JoystickButton getStopSwerve() {
    return stopSwerve;
  }

  public JoystickButton getZeroGyro() {
    return zeroGyro;
  }
}
