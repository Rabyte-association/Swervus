package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = 0.095;
        public static final double kDriveMotorGearRatio = 1 / 7;
        public static final double kTurningMotorGearRatio = 7/64  ;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.2;
    }

    public static final class WindaConstants{
        public static final int kChwytakLewyPort = 13;
        public static final int kChwytakPrawyPort = 14;
        public static final int kWciagarkaPort = 15;
        public static final int kPrzodTylPort = 16;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = 0.594;
        // Distance between right and left wheels
        public static final double kWheelBase = 0.594;
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 7;
        public static final int kBackLeftDriveMotorPort = 10;

        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 8;
        public static final int kBackLeftTurningMotorPort = 11;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 3;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 12;
        public static final int kBackRightDriveAbsoluteEncoderPort = 9;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 6;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;



        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;//-0.117*Math.PI*2;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;// -0.361*Math.PI*2;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;//0.263*Math.PI*2; //-0.262*Math.PI;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;//-0.117*Math.PI*2;//0.112*Math.PI;
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 3.25932;// -0.254;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -0.13188;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 1.46324;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.884;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 3.8;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2* 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond/2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 10;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond ;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.1;
        public static final double kPYController =0.1;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kWindochwytakControllerPort = 1;
        public static final int kLedJoystickPort = 2;

        public static final int kGyroCANID = 20;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 5;

        public static final int kWyciagarkaAxis = 1;
        public static final int kPrzodTylAxis = 5;
        public static final int kLewyPrzod = 2;
        public static final int kLewyTyl = 5;
        public static final int kPrawyPrzod = 3;
        public static final int kPrawyTyl = 6;

        public static final int kLedMaster = 3 ;
        public static final int kLedDriverSwitch = 2;
        public static final int kLedBrighnessAxis = 1;
        public static final int kRedBlue = 4;
        public static final int kStrobeButton = 7;


        public static final double kDeadbandDrive = 0.05;
        public static final double kDeadbandDriveTurn = 0.03;
        public static final double kDeadbandWindochwytak = 0.2;
    }
}
