package frc.lib.util;

import edu.wpi.first.wpilibj.Joystick;

public class DeadzoneJoystick extends Joystick {
    public DeadzoneJoystick (final int port) {
        super(port);
    }

    @Override
    public double getRawAxis(int axis) {
        double val = super.getRawAxis(axis);
        return Math.abs(val) > 0.2 ? val : 0;
    }
}
