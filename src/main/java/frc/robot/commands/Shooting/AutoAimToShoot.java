package frc.robot.commands.Shooting;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.drivetrain.Swerve;

public class AutoAimToShoot extends PIDCommand {
    public AutoAimToShoot(Swerve swerve) {
        super(new PIDController(8, .5, 1.0),
            () -> {
                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                    return swerve.getPose().getRotation().getDegrees();
                }
                return (swerve.getPose().getRotation().getDegrees() + 360) % 360;
            },
            () -> Math.toDegrees(swerve.getAngleToSpeaker()),
            (double omega_per_second) -> {
                swerve.driveChassis(new ChassisSpeeds(0, 0, Math.toRadians(omega_per_second)));
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
        // Logger.recordOutput("Commands/Shooting/AutoAimToShoot/Setpoint", getController().getSetpoint());
        // SmartDashboard.putBoolean("AutoMoveToAngle OK?", isFinished());
    }
}
