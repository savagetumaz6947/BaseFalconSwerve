package frc.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class DeadzoneJoystick extends Joystick {
    public DeadzoneJoystick (final int port) {
        super(port);
    }

    @Override
    public double getRawAxis(int axis) {
        return MathUtil.applyDeadband(super.getRawAxis(axis), Constants.stickDeadband);
    }
}
