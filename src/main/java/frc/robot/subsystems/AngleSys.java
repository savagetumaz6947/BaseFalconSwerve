package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AngleSys extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(41, "canivore");
    private TalonFX rightMotor = new TalonFX(42, "canivore");
    private CANSparkMax sparkMaxEncoderOnly = new CANSparkMax(43, MotorType.kBrushed);
    private DigitalInput limitSwitch = new DigitalInput(9);
    // private TalonSRX srxEncoderOnly = new TalonSRX(43);
    private RelativeEncoder encoder;

    public AngleSys() {
        // rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));

        // rightMotor.setInverted(true);
        encoder = sparkMaxEncoderOnly.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);

        var leftConfig = leftMotor.getConfigurator();
        var rightConfig = rightMotor.getConfigurator();
        var motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        leftConfig.apply(motorConfigs);
        rightConfig.apply(motorConfigs);

        encoder.setPosition(0);
    }

    public double getAngle() {
        return encoder.getPosition() * 360 + 29;
    }

    /**
     * This function returns the automatically calculated angle in order for the shooter to be shot accurately into the speaker.
     * When it is not in range, it will return 34 degrees, the minimum angle available.
     * @return
     */
    public double getAutoAngle(double dist) {
        // if (dist < 2.5) return ; // Refer to Google Sheets
        // return 34;
        return 0;
        // return (58.5 - 10.5 * dist) >= 28 ? 58.5 - 10.5 * dist : 34;
    }

    public void move(double val) {
        if (limitSwitch.get() || val > 0) {
            final DutyCycleOut lRequest = new DutyCycleOut(-val * 0.5);
            final DutyCycleOut rRequest = new DutyCycleOut(val * 0.5);
            leftMotor.setControl(lRequest);
            rightMotor.setControl(rRequest);
        } else {
            final DutyCycleOut stopRequest = new DutyCycleOut(0);
            leftMotor.setControl(stopRequest);
            rightMotor.setControl(stopRequest);
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput("AngleSys/EncoderPos", encoder.getPosition());
        Logger.recordOutput("AngleSys/EncoderAngleDeg", getAngle());
        Logger.recordOutput("AngleSys/Limit", limitSwitch.get());
    }
}
