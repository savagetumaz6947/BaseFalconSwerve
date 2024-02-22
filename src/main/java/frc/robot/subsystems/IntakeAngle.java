package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAngle extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(58, MotorType.kBrushless);

    public IntakeAngle () {
        motor.setIdleMode(IdleMode.kBrake);
    }

    public void rawMove (double value) {
        motor.set(value);
    }
}
