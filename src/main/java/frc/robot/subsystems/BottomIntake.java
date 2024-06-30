package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomIntake extends SubsystemBase {
    private CANSparkMax intakeU = new CANSparkMax(55, MotorType.kBrushless);
    private CANSparkMax intakeD = new CANSparkMax(56, MotorType.kBrushless);

    private final Vision camera;

    private BooleanSupplier isAssistedIntaking;

    public BottomIntake(Vision intakeCamera, BooleanSupplier isAssistedIntaking) {
        intakeD.follow(intakeU);
        camera = intakeCamera;

        this.isAssistedIntaking = isAssistedIntaking;
    }

    public void rawMove (double speed) {
        intakeU.set(-speed);
    }

    public Command assistedIntake() {
        // XXX: possible failure point here
        return new ConditionalCommand(
            this.run(() -> rawMove(0)),
            new ConditionalCommand(
                this.run(() -> rawMove(0.5)).repeatedly().withTimeout(3),
                this.run(() -> rawMove(0)), 
                isAssistedIntaking).until(() -> SmartDashboard.getBoolean("HasNote", false)),
            () -> SmartDashboard.getBoolean("HasNote", false)).repeatedly();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("See Note?", camera.hasTargets());
    }
}
