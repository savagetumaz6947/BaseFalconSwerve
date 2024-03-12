package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAngle extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(58, MotorType.kBrushless);
    private DoubleSupplier intakeAngle;
    private DigitalInput upLimit = new DigitalInput(0);
    private DigitalInput downLimit = new DigitalInput(1);

    public IntakeAngle (DoubleSupplier intakeAngleSupplier) {
        motor.setIdleMode(IdleMode.kBrake);
        motor.getEncoder().setPosition(0);
        intakeAngle = intakeAngleSupplier;
    }

    public void rawMove (double value) {
        if (value > 0) {
            if (!downLimit.get()) {
                motor.set(value);
            } else {
                motor.set(0);
            }
        } else {
            if (!upLimit.get() && intakeAngle.getAsDouble() < 31) {
                motor.set(value);
            } else {
                motor.set(0);
            }
        }
    }

    public Command drop(AngleSys angleSys) {
        Command command = this.runOnce(() -> {
            rawMove(0.5);
        }).repeatedly().withTimeout(2.5).finallyDo(() -> {
            rawMove(0);
        });
        command.addRequirements(angleSys);
        return command;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("IntakeAngle/EncoderPos", motor.getEncoder().getPosition());
        Logger.recordOutput("IntakeAngle/DownLimit", downLimit.get());
        Logger.recordOutput("IntakeAngle/UpLimit", upLimit.get());
        SmartDashboard.putBoolean("IntakeAngle/AtBottom", downLimit.get());
    }
}
