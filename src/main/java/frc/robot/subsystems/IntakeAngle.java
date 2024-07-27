package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeAngle extends SubsystemBase {
    private CANSparkMax motor = new CANSparkMax(58, MotorType.kBrushless);
    private BooleanSupplier intakeBottom;
    private DigitalInput upLimit = new DigitalInput(1);
    private DigitalInput downLimit = new DigitalInput(0);

    public IntakeAngle (BooleanSupplier intakeBottomSupplier) {
        motor.setIdleMode(IdleMode.kBrake);
        motor.getEncoder().setPosition(0);
        intakeBottom = intakeBottomSupplier;

        Constants.DEBUG_TAB.addBoolean("IntakeAngle/DownLimit", () -> downLimit.get());
        Constants.DEBUG_TAB.addBoolean("IntakeAngle/UpLimit", () -> upLimit.get());
    }

    public void rawMove (double value) {
        if (value > 0) {
            if (!downLimit.get()) {
                motor.set(value);
            } else {
                motor.set(0);
            }
        } else {
            if (!upLimit.get() && !intakeBottom.getAsBoolean()) {
                motor.set(value);
            } else {
                motor.set(0);
            }
        }
    }

    public Command drop(AngleSys angleSys) {
        Command command = this.runOnce(() -> {
            rawMove(0.5);
        }).repeatedly().withTimeout(2).finallyDo(() -> {
            rawMove(0);
        });
        command.addRequirements(angleSys);
        return command;
    }

    @Override
    public void periodic() {
        // Logger.recordOutput("IntakeAngle/EncoderPos", motor.getEncoder().getPosition());
        SmartDashboard.putBoolean("IntakeAngle/AtBottom", downLimit.get());
    }
}
