package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestRev extends SubsystemBase {
    private CANSparkMax motor1 = new CANSparkMax(61, MotorType.kBrushless);
    private CANSparkMax motor2 = new CANSparkMax(62, MotorType.kBrushless);

    public TestRev() {
        // motor2.follow(motor1, true);
    }
    
    public void move (double speed) {
        Logger.recordOutput("sparkspeed", speed);
        motor1.set(speed);
        motor2.set(speed);
    }
}
