package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AngleSys extends SubsystemBase {
    private TalonFX leftMotor = new TalonFX(41, "canivore");
    private TalonFX rightMotor = new TalonFX(42, "canivore");
    // private CANSparkMax sparkMaxEncoderOnly = new CANSparkMax(43, MotorType.kBrushed);
    private CANcoder encoder = new CANcoder(43, "canivore");
    private DigitalInput downLimit = new DigitalInput(9);
    private DigitalInput upLimit = new DigitalInput(8);
    // private RelativeEncoder encoder;

    public AngleSys() {
        // encoder = sparkMaxEncoderOnly.getEncoder(SparkRelativeEncoder.Type.kQuadrature, 4096);

        TalonFXConfigurator leftConfig = leftMotor.getConfigurator();
        TalonFXConfigurator rightConfig = rightMotor.getConfigurator();
        // CANcoderConfigurator cancoderConfig = encoder.getConfigurator();

        MotorOutputConfigs motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        leftConfig.apply(motorConfigs);
        rightConfig.apply(motorConfigs);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));

        // encoder.setPosition(0);

        Constants.DEBUG_TAB.addBoolean("AngleSys/DownLimit", () -> downLimit.get());
        Constants.DEBUG_TAB.addBoolean("AngleSys/UpLimit", () -> upLimit.get());
    }

    public double getAngle() {
        return -encoder.getAbsolutePosition().getValueAsDouble() * 360 + 30;
    }

    public boolean getDownLimit() {
        return downLimit.get();
    }

    /**
     * This function returns the automatically calculated angle in order for the shooter to be shot accurately into the speaker.
     * When it is not in range, it will return 34 degrees, the minimum angle available.
     * @return
     */
    public double getAutoAngle(double dist) {
        double value = (58.9 - 1.375 - 8.09 * dist);
        if (value < 29) return 29;
        else if (value > 54) return 54;
        return value; // Refer to Google Sheets
    }

    public void move(double val) {
        if ((downLimit.get() && val < 0) || (!upLimit.get() && SmartDashboard.getBoolean("IntakeAngle/AtBottom", false) && val > 0)) {
            final DutyCycleOut lRequest = new DutyCycleOut(-val * 1);
            leftMotor.setControl(lRequest);
        } else {
            final DutyCycleOut stopRequest = new DutyCycleOut(0);
            leftMotor.setControl(stopRequest);
        }
    }

    public void reset() {
        encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // Logger.recordOutput("AngleSys/EncoderPos", encoder.getPosition());
        // Logger.recordOutput("AngleSys/EncoderAngleDeg", getAngle());

        SmartDashboard.putNumber("AngleSys/EncoderPos", encoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("AngleSys/EncoderAngleDeg", getAngle());
    }
}
