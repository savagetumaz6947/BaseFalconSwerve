package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomIntake extends SubsystemBase {
    private CANSparkMax intakeU = new CANSparkMax(55, MotorType.kBrushless);
    private CANSparkMax intakeD = new CANSparkMax(56, MotorType.kBrushless);

    private final Vision camera = new Vision("IntakeCam");

    public BottomIntake() {
        intakeD.follow(intakeU);
    }

    public Vision getCamera() {
        return camera;
    }

    public void rawMove (double speed) {
        intakeU.set(-speed);
    }

    public Command stop() {
        return new InstantCommand(() -> {
            rawMove(0);
        }, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("See Note?", camera.hasTargets());
    }
}
