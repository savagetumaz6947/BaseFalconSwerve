package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    // Inital pose of the robot
    public static Pose2d initialPose = new Pose2d(1.24, 5.57, new Rotation2d(0));

    public static final class Vision {
        public static final String cameraName = "BackCamera";
        // Cam mounted facing foward, 54 degrees up, at the front of the robot, 23cm up.
        public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.02-0.36,0,0.52), new Rotation3d(0, Math.toRadians(58-90), Math.PI));
        // public static final Transform3d robotToCam = new Transform3d(new Translation3d(0,0,0), new Rotation3d(0, 0, 0));

        // public static final AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(List.of(
        //     new AprilTag(1, new Pose3d(1.5, 2, 0.5, new Rotation3d(0, 0, Math.PI)))
        // ), 16.4846, 8.1026);

        public static final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    public static final class GameObjects {
        public static final class BlueAlliance {
            public static final Pose3d speaker = new Pose3d(0, 5.54, 2.05, new Rotation3d());
        }
        public static final class RedAlliance {
            public static final Pose3d speaker = new Pose3d(16.54, 5.54, 2.05, new Rotation3d(Math.PI, 0, 0));
        }
    }

    public static final class Swerve {
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSTalonFXSwerveConstants chosenModule =
            COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.595;
        public static final double wheelBase = 0.595;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.13; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.075;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.1;  // TODO: This must be tuned to specific robot / This is the theoretical maxSpeed of your robot (aka the 100% value)
                                                        //   You can ignore this value if "isOpenLoop" is set to FALSE
        public static final double[] speedSelection = {1.3, 3.0, 4.5}; //TODO: You can set this to your desired speed
        public static final double autonomousMaxSpeedSelection  = 4.5; // This refers to the index of the speedSelection defined on top by speedSelection[] used by Autonomous mode

        public static final double teleopMaxTranslateAcceleration = 15;
        /** Radians per Second */
        public static final double maxAngularVelocity = Units.degreesToRadians(540); //TODO: This must be tuned to specific robot

        public static final double teleopMaxAngularAcceleration = Units.degreesToRadians(720);

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Open Loop */
        public static final boolean isOpenLoop = false; // TRUE uses calculated duty cycle with maxSpeed, whereas FALSE uses feedfoward velocity output

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 31;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(57.7);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 13;
            public static final int angleMotorID = 14;
            public static final int canCoderID = 32;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.26);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 15;
            public static final int angleMotorID = 16;
            public static final int canCoderID = 33;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(132.89);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 17;
            public static final int angleMotorID = 18;
            public static final int canCoderID = 34;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(304.62);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final HolonomicPathFollowerConfig autoConstants = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(1.9, 0, 0), // Translation PID constants
        new PIDConstants(3, 0.0, 0.0), // Rotation PID constants
        Swerve.autonomousMaxSpeedSelection, // Max module speed, in m/s
        Math.sqrt(Math.pow(Swerve.wheelBase / 2, 2) + Math.pow(Swerve.trackWidth / 2, 2)), // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
    );

    public static final PathConstraints defaultPathConstraints = new PathConstraints(3.3, 5, Math.toRadians(420), Math.toRadians(720));
}
