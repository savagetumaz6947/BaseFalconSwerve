package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MidIntake extends SubsystemBase {    
    private CANSparkMax intakeMid = new CANSparkMax(57, MotorType.kBrushless);

    private ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kOnboard);
    
    public MidIntake () {
        intakeMid.setIdleMode(IdleMode.kBrake);
    }

    public void moveMid(double speed) {
        intakeMid.set(speed);
    }

    public Color getColor() {
        return sensor.getColor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Color", getColor().toString());
    }
}
