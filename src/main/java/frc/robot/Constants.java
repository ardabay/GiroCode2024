// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
        public static final class ModuleConstants {
                public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
                public static final double kDriveMotorGearRatio = 1 / 8.14;
                public static final double kTurningMotorGearRatio = 1 / 18.0;
                public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI
                                * kWheelDiameterMeters;
                public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
                public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
                public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
                public static final double kPTurning = 0.22;
        }

        public static final class DriveConstants {

                public static final double kTrackWidth = Units.inchesToMeters(27.74);
                // Distance between right and left wheels
                public static final double kWheelBase = Units.inchesToMeters(31.49);
                // Distance between front and back wheels
                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2));

                public static final int kFrontLeftDriveMotorPort = 2;
                public static final int kBackLeftDriveMotorPort = 13;
                public static final int kFrontRightDriveMotorPort = 10;
                public static final int kBackRightDriveMotorPort = 12;

                public static final int kFrontLeftTurningMotorPort = 7;
                public static final int kBackLeftTurningMotorPort = 14;
                public static final int kFrontRightTurningMotorPort = 9;
                public static final int kBackRightTurningMotorPort = 11;

                public static final boolean kFrontLeftTurningEncoderReversed = true;
                public static final boolean kBackLeftTurningEncoderReversed = true;
                public static final boolean kFrontRightTurningEncoderReversed = true;
                public static final boolean kBackRightTurningEncoderReversed = true;

                public static final boolean kFrontLeftDriveEncoderReversed = false;
                public static final boolean kBackLeftDriveEncoderReversed = false;
                public static final boolean kFrontRightDriveEncoderReversed = false;
                public static final boolean kBackRightDriveEncoderReversed = false;

                public static final int kFrontLeftDriveAbsoluteEncoderPort = 18;
                public static final int kBackLeftDriveAbsoluteEncoderPort = 15;
                public static final int kFrontRightDriveAbsoluteEncoderPort = 17;
                public static final int kBackRightDriveAbsoluteEncoderPort = 16;

                public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
                public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
                public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
                public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

                public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
                public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
                public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.0;
                public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;

                public static final double kPhysicalMaxSpeedMetersPerSecond = 3.70945671;
                public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

                public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
                public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
                public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
                public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        }

        public static final class AutoConstants {
                public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond
                                / 4;
                public static final double kMaxAngularSpeedRadiansPerSecond = //
                                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3;
                public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
                public static final double kPXController = 1.5;
                public static final double kPYController = 1.5;
                public static final double kPThetaController = 3;

                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                                new TrapezoidProfile.Constraints(
                                                kMaxAngularSpeedRadiansPerSecond,
                                                kMaxAngularAccelerationRadiansPerSecondSquared);
        }

        public static final class OIConstants {
                public static final int kDriverControllerPort = 0;

                public static final int kDriverYAxis = 1;
                public static final int kDriverXAxis = 0;
                public static final int kDriverRotAxis = 2;
                public static final int kDriverFieldOrientedButtonIdx = 1;

                public static final double kDeadband = 0.05;
        }

        public static final class JoystickControlConstans {
                public static final int StopSwerveButton = 1; // Swerve Stop Button
                public static final int ZeroGyroButton = 2; // Swerve Zero Gyro Button
                public static final int ShootButton = 3;
                public static final int StopButton = 4;
                public static final int IntakeButton = 5;  // Start Shooter Button
        }

        public static final class TransferConstans {
                public static final int LeftMotorCan = 5; // Shooter Sol Can
                public static final int RightMotorCan = 4; // Shooter Sağ Can
        }

        public static final class ShooterConstans {
                public static final int LeftMotorCan = 3; // Aktarım Sol Can
                public static final int RightMotorCan = 8; // Aktarım Sağ Can
        }

        public static final class ShooterPositionConstans {

        public static final double kAngleEncoderRot2Degrees = (1 / 12.0) * 360;
        }

        public static final class VisionConstants {
                public static final double CamHeight = 0;
        }

}
