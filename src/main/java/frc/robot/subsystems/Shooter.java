package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkMax up = new CANSparkMax(62, MotorType.kBrushless);
    private CANSparkMax down = new CANSparkMax(61, MotorType.kBrushless);

    public Shooter() {
        down.follow(up, true);
    }

    public void shoot() {
        up.set(-1);
    }

    public void stop() {
        up.set(0);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/UP_RPM",  up.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/DOWN_RPM",  up.getEncoder().getVelocity());
    }
}
