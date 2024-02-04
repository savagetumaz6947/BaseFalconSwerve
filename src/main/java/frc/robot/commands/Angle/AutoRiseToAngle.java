package frc.robot.commands.Angle;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.AngleSys;
import frc.robot.subsystems.drivetrain.Swerve;

public class AutoRiseToAngle extends PIDCommand {
    public AutoRiseToAngle(AngleSys angleSys, Swerve swerve) {
        super(
            new PIDController(0.045, 0.06, 0),
            angleSys::getAngle,
            () -> angleSys.getAutoAngle(swerve.getDistToSpeaker()),
            angleSys::move,
            angleSys
        );
        getController().setTolerance(0.3);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("AutoMoveToAngle OK?", getController().atSetpoint());
        return getController().atSetpoint();
    }

    @Override
    public void execute() {
        super.execute();
        Logger.recordOutput("Commands/Angle/AutoMoveToAngle/Setpoint", getController().getSetpoint());
    }
}
