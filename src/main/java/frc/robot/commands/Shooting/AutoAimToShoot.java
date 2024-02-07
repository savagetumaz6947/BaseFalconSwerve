package frc.robot.commands.Shooting;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drivetrain.Swerve;

public class AutoAimToShoot extends PIDCommand {
    public AutoAimToShoot(Swerve swerve) {
        super(new PIDController(0.02, 0.015, 0.005),
            () -> swerve.getPose().getRotation().getDegrees(),
            () -> Math.toDegrees(swerve.getAngleToSpeaker()),
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

    @Override
    public void execute() {
        super.execute();
        Logger.recordOutput("Commands/Shooting/AutoAimToShoot/Setpoint", getController().getSetpoint());
        // SmartDashboard.putBoolean("AutoMoveToAngle OK?", isFinished());
    }
}
