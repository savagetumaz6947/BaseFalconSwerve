package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private IntSupplier maxSpeedMode;

    private Alliance alliance;

    private SlewRateLimiter translateXLimiter = new SlewRateLimiter(Constants.Swerve.teleopMaxTranslateAcceleration);
    private SlewRateLimiter translateYLimiter = new SlewRateLimiter(Constants.Swerve.teleopMaxTranslateAcceleration);

    public TeleopSwerve(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, IntSupplier maxSpeedMode) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.maxSpeedMode = maxSpeedMode;

        this.alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        swerve.navx.reset();
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        SmartDashboard.putNumber("Max Speed", Constants.Swerve.speedSelection[maxSpeedMode.getAsInt()]);

        double transXVal = translateXLimiter.calculate(this.translationSup.getAsDouble() * Constants.Swerve.speedSelection[maxSpeedMode.getAsInt()]);
        double transYVal = translateYLimiter.calculate(this.strafeSup.getAsDouble() * Constants.Swerve.speedSelection[maxSpeedMode.getAsInt()]);

        this.alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        /* Drive */
        swerve.drive(
            new Translation2d(
                alliance == Alliance.Blue ? transXVal : -transXVal,
                alliance == Alliance.Blue ? transYVal : -transYVal
            ), 
            this.rotationSup.getAsDouble() * Constants.Swerve.maxAngularVelocity,
            robotCentricSup.getAsBoolean(), 
            Constants.Swerve.speedSelection[maxSpeedMode.getAsInt()]
        );
    }
}