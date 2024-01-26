package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AngleSys extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(41, "canivore");
    private TalonFX rightMotor = new TalonFX(42, "canivore");
    // private CANSparkMax sparkMaxEncoderOnly = new CANSparkMax(43, MotorType.kBrushless);
    // private RelativeEncoder encoder;

    public AngleSys() {
        // rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));

        // rightMotor.setInverted(true);
        // encoder = sparkMaxEncoderOnly.getAlternateEncoder(4096);

        var leftConfig = leftMotor.getConfigurator();
        var rightConfig = rightMotor.getConfigurator();
        var motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        leftConfig.apply(motorConfigs);
        rightConfig.apply(motorConfigs);
    }

    public void move(double val) {
        final DutyCycleOut lRequest = new DutyCycleOut(val * 0.2);
        final DutyCycleOut rRequest = new DutyCycleOut(-val * 0.2);
        leftMotor .setControl(lRequest);
        rightMotor .setControl(rRequest);
    }

    @Override
    public void periodic() {
        // Logger.recordOutput("Angle", encoder.getPosition());
    }
}
