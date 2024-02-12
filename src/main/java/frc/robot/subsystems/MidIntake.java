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
    private CANSparkMax motor = new CANSparkMax(57, MotorType.kBrushless);

    private ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kOnboard);
    
    public MidIntake () {
        motor.setIdleMode(IdleMode.kBrake);
    }

    public void rawMove(double speed) {
        motor.set(speed);
    }

    public boolean hasNote() {
        return getColor().red > 0.32;
    }

    private Color getColor() {
        return sensor.getColor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Color", getColor().toString());
        SmartDashboard.putBoolean("HasNote", hasNote());
    }
}
