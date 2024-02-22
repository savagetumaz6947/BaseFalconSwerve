package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier shooterInRange;
    private BooleanSupplier enableAutoAimWhileDrive;
    private IntSupplier maxSpeedMode;

    private SlewRateLimiter translateXLimiter = new SlewRateLimiter(Constants.Swerve.teleopMaxTranslateAcceleration);
    private SlewRateLimiter translateYLimiter = new SlewRateLimiter(Constants.Swerve.teleopMaxTranslateAcceleration);

    private PIDController aimToShootController = new PIDController(10, 0, 1.0);

    public TeleopSwerve(Swerve s_Swerve, 
                        DoubleSupplier translationSup, 
                        DoubleSupplier strafeSup,
                        DoubleSupplier rotationSup,
                        BooleanSupplier robotCentricSup,
                        IntSupplier maxSpeedMode,
                        BooleanSupplier shooterInRange,
                        BooleanSupplier enableAutoAimWhileDrive) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.maxSpeedMode = maxSpeedMode;
        this.shooterInRange = shooterInRange;
        this.enableAutoAimWhileDrive = enableAutoAimWhileDrive;

        aimToShootController.setTolerance(1);

        s_Swerve.navx.reset();
    }

    public boolean atSetpoint() {
        return aimToShootController.atSetpoint();
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        SmartDashboard.putNumber("Max Speed", Constants.Swerve.speedSelection[maxSpeedMode.getAsInt()]);

        double rotateValue = this.rotationSup.getAsDouble() * Constants.Swerve.maxAngularVelocity;

        if (enableAutoAimWhileDrive.getAsBoolean() && shooterInRange.getAsBoolean() && Math.abs(this.rotationSup.getAsDouble()) < Constants.stickDeadband) {
            double pV = s_Swerve.getPose().getRotation().getDegrees();
            double sP = Math.toDegrees(s_Swerve.getAngleToSpeaker());
            rotateValue = aimToShootController.calculate(pV, sP);
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(
                translateXLimiter.calculate(this.translationSup.getAsDouble() * Constants.Swerve.speedSelection[maxSpeedMode.getAsInt()]),
                translateYLimiter.calculate(this.strafeSup.getAsDouble() * Constants.Swerve.speedSelection[maxSpeedMode.getAsInt()])
            ), 
            rotateValue,
            robotCentricSup.getAsBoolean(), 
            Constants.Swerve.speedSelection[maxSpeedMode.getAsInt()]
        );
    }
}