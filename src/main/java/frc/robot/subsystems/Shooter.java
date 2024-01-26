package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkMax up = new CANSparkMax(62, MotorType.kBrushless);
    private CANSparkMax down = new CANSparkMax(61, MotorType.kBrushless);

    public void shoot() {
        up.set(1);
        down.set(-1);
    }

    public void stop() {
        up.set(0);
        down.set(0);
    }
}
