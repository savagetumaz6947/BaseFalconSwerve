package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(51, "canivore");
    private TalonFX rightMotor = new TalonFX(52, "canivore");

    public Climber () {
        var leftConfig = leftMotor.getConfigurator();
        var rightConfig = rightMotor.getConfigurator();
        var motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        leftConfig.apply(motorConfigs);
        rightConfig.apply(motorConfigs);
    }

    public void move (double left, double right) {
        leftMotor.set(left);
        rightMotor.set(right);
        Logger.recordOutput("Hang/leftMotor", left);
    }
}
