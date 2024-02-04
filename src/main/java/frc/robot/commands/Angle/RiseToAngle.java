package frc.robot.commands.Angle;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.AngleSys;

public class RiseToAngle extends PIDCommand {
    public RiseToAngle(double targetAngleDegrees, AngleSys angleSys) {
        super(
            new PIDController(0.045, 0.06, 0),
            angleSys::getAngle,
            targetAngleDegrees,
            angleSys::move,
            angleSys
        );
        getController().setTolerance(360/4096);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
