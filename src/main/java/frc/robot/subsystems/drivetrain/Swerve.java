package frc.robot.subsystems.drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.SwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private Vision s_Vision;
    public SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field = new Field2d();

    public SwerveModule[] mSwerveMods;
    public AHRS navx;

    public SlewRateLimiter[] rateLimiters;

    public Swerve() {
        navx = new AHRS(SPI.Port.kMXP);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        rateLimiters = new SlewRateLimiter[] {
            new SlewRateLimiter(Constants.Swerve.teleopMaxWheelAcceleration),
            new SlewRateLimiter(Constants.Swerve.teleopMaxWheelAcceleration),
            new SlewRateLimiter(Constants.Swerve.teleopMaxWheelAcceleration),
            new SlewRateLimiter(Constants.Swerve.teleopMaxWheelAcceleration)
        };

        zeroGyro();

        s_Vision = new Vision(Constants.Vision.cameraName, Constants.Vision.robotToCam, Constants.Vision.fieldLayout);
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), Constants.initialPose, VecBuilder.fill(0.9, 0.9, 0.9), VecBuilder.fill(0.1, 0.1, 0.1));

        AutoBuilder.configureHolonomic(this::getPose, this::setPose, 
                                        () -> Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()),
                                        this::driveChassis,
                                        Constants.autoConstants,
                                        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                                        this);
        Pathfinding.setPathfinder(new LocalADStar());
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
            // Logger.recordOutput(
            //     "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                field.getObject("path").setPoses(activePath);
            });
        // PathPlannerLogging.setLogTargetPoseCallback(
        //     (targetPose) -> {
        //     Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        //     });

        // s_Vision.setLastPose(getPose());
    }

    /* Wrapper function that uses the Autonomous maxSpeedIndex for autonomous */
    public void driveChassis(ChassisSpeeds cSpeeds) {
        driveChassis(cSpeeds, Constants.Swerve.autonomousMaxSpeedSelection);
    }

    public void driveChassis(ChassisSpeeds cSpeeds, double maxSpeedSelection) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(cSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeedSelection);

        for (SwerveModule mod : mSwerveMods){
            // Max Wheel Acceleration (only in Teleop)
            if (DriverStation.isTeleop())
                swerveModuleStates[mod.moduleNumber].speedMetersPerSecond =
                    rateLimiters[mod.moduleNumber].calculate(swerveModuleStates[mod.moduleNumber].speedMetersPerSecond);
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, double maxSpeedSelection) {
        drive(translation, rotation, fieldRelative, maxSpeedSelection, new ChassisSpeeds());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, double maxSpeedSelection, ChassisSpeeds offsetChassisSpeeds) {
        driveChassis(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading()).plus(offsetChassisSpeeds)
                      : new ChassisSpeeds(
                          translation.getX(),
                          translation.getY(),
                          rotation).plus(offsetChassisSpeeds),
                    maxSpeedSelection);
    }

    public void zeroGyro(){
        navx.reset();
    }

    public Rotation2d getGyroYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - (navx.getYaw() < 0 ? (navx.getYaw()+360)%360 : navx.getYaw())) : Rotation2d.fromDegrees((navx.getYaw() < 0 ? (navx.getYaw()+360)%360 : navx.getYaw()));
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void setHeading(Rotation2d heading){
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }


    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }


    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    /**
     * This function returns the shortest distance in 2D between the robot and the speaker of the robot's alliance.
     * @return The distance in 2D between the pose and the speaker
     */
    public double getDistToSpeaker() {
        Pose2d pose = getPose();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
            return Math.sqrt(Math.pow(pose.getX() - Constants.GameObjects.BlueAlliance.speaker.getX(),2) + 
                                Math.pow(pose.getY() - Constants.GameObjects.BlueAlliance.speaker.getY(), 2));
        else
            return Math.sqrt(Math.pow(pose.getX() - Constants.GameObjects.RedAlliance.speaker.getX(),2) + 
                                Math.pow(pose.getY() - Constants.GameObjects.RedAlliance.speaker.getY(), 2));
    }

    public double getAngleToSpeaker() {
        Pose2d pose = getPose();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
            return Math.atan((pose.getY() - Constants.GameObjects.BlueAlliance.speaker.getY()) / 
                                (pose.getX() - Constants.GameObjects.BlueAlliance.speaker.getX()));
        else
            return (Math.PI + Math.atan((pose.getY() - Constants.GameObjects.RedAlliance.speaker.getY()) / 
                                (pose.getX() - Constants.GameObjects.RedAlliance.speaker.getX())) + (Math.PI * 2)) % (Math.PI * 2);
    }
    
    @Override
    public void periodic(){
        poseEstimator.update(getGyroYaw(), getModulePositions());

        Optional<EstimatedRobotPose> visionPose = s_Vision.getEstimatedGlobalPose();
        if (visionPose.isPresent()) {
            // Logger.recordOutput("Odometry/Vision3d", visionPose.get().estimatedPose);
            // Logger.recordOutput("Odometry/Vision2d", visionPose.get().estimatedPose.toPose2d());
            poseEstimator.addVisionMeasurement(visionPose.get().estimatedPose.toPose2d(), visionPose.get().timestampSeconds);
        }

        field.setRobotPose(getPose());
        // s_Vision.setLastPose(getPose());

        SmartDashboard.putData("Field", field);

        // Logger.recordOutput("Odometry/Composite Pose", getPose());
        // Logger.recordOutput("Odometry/Module States", getModuleStates());

        // Logger.recordOutput("Odometry/Dist to Speaker", getDistToSpeaker());
    }
}