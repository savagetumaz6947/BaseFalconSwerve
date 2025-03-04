package frc.robot.commands.Angle;

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
        return getController().atSetpoint();
    }

    @Override
    public void execute() {
        super.execute();
        SmartDashboard.putBoolean("AutoRiseToAngle", isFinished());
    }
}
