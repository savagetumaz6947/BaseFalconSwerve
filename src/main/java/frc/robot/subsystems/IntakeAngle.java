package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAngle extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(58, MotorType.kBrushless);

    public IntakeAngle () {
        motor.setIdleMode(IdleMode.kBrake);
        motor.getEncoder().setPosition(0);
    }

    public void rawMove (double value) {
        motor.set(value);
    }

    public Command drop(AngleSys angleSys) {
        Command command = this.runOnce(() -> {
            rawMove(0.5);
        }).repeatedly().withTimeout(0.5);
        command.addRequirements(angleSys);
        return command;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("IntakeAngle/EncoderPos", motor.getEncoder().getPosition());
    }
}
