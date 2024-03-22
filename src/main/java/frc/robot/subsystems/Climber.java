package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(51, "canivore");
    private TalonFX rightMotor = new TalonFX(52, "canivore");

    public Climber () {
        TalonFXConfigurator leftConfig = leftMotor.getConfigurator();
        TalonFXConfigurator rightConfig = rightMotor.getConfigurator();
        MotorOutputConfigs leftMotorOutputConfigs = new MotorOutputConfigs();
        MotorOutputConfigs rightMotorOutputConfigs = new MotorOutputConfigs();
        leftMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        rightMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        leftMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        leftConfig.apply(leftMotorOutputConfigs);
        rightConfig.apply(rightMotorOutputConfigs);

        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }

    public void move (DoubleSupplier left, DoubleSupplier right, BooleanSupplier force) {
        if (force.getAsBoolean()) {
            leftMotor.set(left.getAsDouble());
            rightMotor.set(right.getAsDouble());
        } else {
            if ((left.getAsDouble() < 0 && leftMotor.getPosition().getValueAsDouble() > -180) || (left.getAsDouble() > 0 && leftMotor.getPosition().getValueAsDouble() < 0)) {
                leftMotor.set(left.getAsDouble());
            } else {
                leftMotor.set(0);
            }

            if ((right.getAsDouble() < 0 && rightMotor.getPosition().getValueAsDouble() > -180) || (right.getAsDouble() > 0 && rightMotor.getPosition().getValueAsDouble() < 0)) {
                rightMotor.set(right.getAsDouble());
            } else {
                rightMotor.set(0);
            }
        }
    }

    @Override
    public void periodic() {
        // Logger.recordOutput("Climber/Left", leftMotor.getPosition().getValueAsDouble());
        // Logger.recordOutput("Climber/Right", rightMotor.getPosition().getValueAsDouble());
    }
}
