package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomIntake extends SubsystemBase {
    private CANSparkMax intakeU = new CANSparkMax(55, MotorType.kBrushless);
    private CANSparkMax intakeD = new CANSparkMax(56, MotorType.kBrushless);
    
    public void moveDown (double speed) {
        intakeU.set(speed);
        intakeD.set(speed);
    }
}
