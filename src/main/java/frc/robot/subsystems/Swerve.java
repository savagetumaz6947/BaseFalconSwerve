package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS navx;
    public ChassisSpeeds robotChassisSpeeds;
    // public Pigeon2 gyro;

    public Swerve() {
        navx = new AHRS(SPI.Port.kMXP);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        zeroGyro();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d(2, 2, getYaw()));

        AutoBuilder.configureHolonomic(this::getPose, this::resetOdometry, 
                                        () -> Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()),
                                        this::driveToChasis,
                                        Constants.autoConstants,
                                        this);
    }

    /* Wrapper function that uses the Autonomous maxSpeedIndex for autonomous */
    public void driveToChasis(ChassisSpeeds cSpeeds) {
        driveToChasis(cSpeeds, Constants.Swerve.autonomousMaxSpeedIndex);
    }

    public void driveToChasis(ChassisSpeeds cSpeeds, int maxSpeedMode) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(cSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed[maxSpeedMode]);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false, maxSpeedMode);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, int maxSpeedMode) {
        driveToChasis(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), 
                        translation.getY(), 
                        rotation, 
                        getYaw())
                      : new ChassisSpeeds(
                          translation.getX(), 
                          translation.getY(), 
                          rotation)
                      , maxSpeedMode);
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
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

    public void zeroGyro(){
        navx.reset();
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - (navx.getYaw() < 0 ? (navx.getYaw()+360)%360 : navx.getYaw())) : Rotation2d.fromDegrees((navx.getYaw() < 0 ? (navx.getYaw()+360)%360 : navx.getYaw()));
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());
        System.out.println(getPose().toString());
        Logger.recordOutput("RobotPose", getPose());
        Logger.recordOutput("Robot Module States", getModuleStates());

        for(SwerveModule mod : mSwerveMods){
            Logger.recordOutput("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            Logger.recordOutput("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            Logger.recordOutput("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);  
        }
    }
}