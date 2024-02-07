package frc.robot.commands.Angle;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AngleSys;
import frc.robot.subsystems.drivetrain.Swerve;

public class AutoRiseToAngle extends RiseToAngle {
    public AutoRiseToAngle(AngleSys angleSys, Swerve swerve) {
        super(() -> angleSys.getAutoAngle(swerve.getDistToSpeaker()), angleSys);
        getController().setTolerance(1);
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
        Logger.recordOutput("Commands/Angle/AutoMoveToAngle/AtSetpoint", getController().atSetpoint());
    }
}
