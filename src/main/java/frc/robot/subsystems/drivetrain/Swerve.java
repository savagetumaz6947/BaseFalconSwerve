package frc.robot.subsystems.drivetrain;

import frc.lib.util.LocalADStarAK;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.SwerveModule;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private Vision s_Vision;
    public SwerveDrivePoseEstimator poseEstimator;

    public SwerveModule[] mSwerveMods;
    public AHRS navx;

    public Swerve() {
        navx = new AHRS(SPI.Port.kMXP);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        zeroGyro();

        AutoBuilder.configureHolonomic(this::getPose, this::setPose, 
                                        () -> Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()),
                                        this::driveChassis,
                                        Constants.autoConstants,
                                        this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
            Logger.recordOutput(
                "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            });

        s_Vision = new Vision(Constants.Vision.cameraName, Constants.Vision.robotToCam, Constants.Vision.fieldLayout);
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), Constants.initialPose);
    }

    /* Wrapper function that uses the Autonomous maxSpeedIndex for autonomous */
    public void driveChassis(ChassisSpeeds cSpeeds) {
        driveChassis(cSpeeds, Constants.Swerve.autonomousMaxSpeedIndex);
    }

    public void driveChassis(ChassisSpeeds cSpeeds, int maxSpeedMode) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(cSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed[maxSpeedMode]);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false, maxSpeedMode);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, int maxSpeedMode) {
        driveChassis(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), 
                        translation.getY(), 
                        rotation, 
                        getHeading())
                      : new ChassisSpeeds(
                          translation.getX(), 
                          translation.getY(), 
                          rotation)
                      , maxSpeedMode);
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

    @Override
    public void periodic(){
        poseEstimator.update(getGyroYaw(), getModulePositions());

        Optional<EstimatedRobotPose> visionPose = s_Vision.getEstimatedGlobalPose();
        if (visionPose.isPresent()) {
            Logger.recordOutput("Odometry/Vision3d", visionPose.get().estimatedPose);
            Logger.recordOutput("Odometry/Vision2d", visionPose.get().estimatedPose.toPose2d());
            poseEstimator.addVisionMeasurement(visionPose.get().estimatedPose.toPose2d(), visionPose.get().timestampSeconds);
        }

        Logger.recordOutput("Odometry/Composite Pose", getPose());
        Logger.recordOutput("Odometry/Module States", getModuleStates());
    }
}