package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.MidIntake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drivetrain.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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


    // For Assisted Intake
    private Vision vision;
    private PIDController assistedIntakeController = new PIDController(0.08, 0, 0);
    private BooleanSupplier enableAssistedIntake;

    private Alliance alliance;

    public TeleopSwerve(Swerve swerve, MidIntake midIntake, Vision vision, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, IntSupplier maxSpeedMode,
            BooleanSupplier enableAssistedIntake) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.maxSpeedMode = maxSpeedMode;

        this.vision = vision;
        this.enableAssistedIntake = enableAssistedIntake;
        assistedIntakeController.setTolerance(2);
        assistedIntakeController.setSetpoint(0);

        this.alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        swerve.navx.reset();
    }

    public double calculateAssistedIntake() {
        if (enableAssistedIntake.getAsBoolean() && vision.hasTargets() && Math.abs(vision.getBestTarget().getYaw()) > 2) {
            return assistedIntakeController.calculate(vision.getBestTarget().getYaw());
        } else {
            return 0;
        }
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        SmartDashboard.putNumber("Max Speed", Constants.Swerve.speedSelection[maxSpeedMode.getAsInt()]);

        double transXVal = this.translationSup.getAsDouble() * Constants.Swerve.speedSelection[maxSpeedMode.getAsInt()];
        double transYVal = this.strafeSup.getAsDouble() * Constants.Swerve.speedSelection[maxSpeedMode.getAsInt()];

        this.alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        double offsetVy = calculateAssistedIntake();
        SmartDashboard.putBoolean("Assisted Intake Enabled", enableAssistedIntake.getAsBoolean());
        SmartDashboard.putNumber("Assisted Intake Offset", offsetVy);

        /* Drive */
        swerve.drive(
                new Translation2d(
                        alliance == Alliance.Blue ? transXVal : (robotCentricSup.getAsBoolean() ? -transXVal : transXVal),
                        alliance == Alliance.Blue ? transYVal : (robotCentricSup.getAsBoolean() ? -transYVal : transYVal)),
                this.rotationSup.getAsDouble() * Constants.Swerve.maxAngularVelocity,
                robotCentricSup.getAsBoolean(),
                Constants.Swerve.speedSelection[maxSpeedMode.getAsInt()],
                new ChassisSpeeds(0, offsetVy, 0));
    }
}