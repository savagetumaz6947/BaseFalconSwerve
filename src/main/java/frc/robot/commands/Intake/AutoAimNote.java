package frc.robot.commands.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drivetrain.Swerve;

public class AutoAimNote extends PIDCommand {
    public AutoAimNote(Swerve swerve, Vision intakeVision) {
        super(new PIDController(0.02, 0.015, 0.005),
            () -> {
                if (intakeVision.hasTargets()) {
                    return intakeVision.getBestTarget().getYaw();
                } else {
                    return 0;
                }
            },
            0,
            (double omega_per_second) -> {
                swerve.driveChassis(new ChassisSpeeds(0, 0, omega_per_second));
            },
            swerve
        );
        getController().setTolerance(1);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
