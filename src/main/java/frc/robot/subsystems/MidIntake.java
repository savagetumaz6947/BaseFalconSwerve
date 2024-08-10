package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MidIntake extends SubsystemBase {    
    private CANSparkMax motor = new CANSparkMax(57, MotorType.kBrushless);

    private ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kMXP);

    private BooleanSupplier isAssistedIntaking;
    
    public MidIntake (BooleanSupplier isAssistedIntaking) {
        motor.setIdleMode(IdleMode.kBrake);

        this.isAssistedIntaking = isAssistedIntaking;
    }

    public void rawMove(double speed) {
        motor.set(speed);
    }

    public boolean hasNote() {
        return getColor().red > 0.28;
    }

    private Color getColor() {
        return sensor.getColor();
    }

    public Command assistedIntake() {
        return new ConditionalCommand(
            this.runOnce(() -> rawMove(0)),
            new ConditionalCommand(
                this.run(() -> rawMove(-1)).withTimeout(3),
                this.runOnce(() -> rawMove(0)),
                isAssistedIntaking).until(this::hasNote),
            this::hasNote).repeatedly();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Color", getColor().toString());
        SmartDashboard.putBoolean("HasNote", hasNote());
    }
}
