package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Swerve;

public class Shooter extends SubsystemBase {
    private CANSparkMax up = new CANSparkMax(62, MotorType.kBrushless);
    private CANSparkMax down = new CANSparkMax(61, MotorType.kBrushless);

    private Swerve swerve;

    public Shooter(Swerve swerve) {
        down.follow(up, true);
        this.swerve = swerve;
    }

    public void shoot() {
        up.set(-1);
    }

    public Command idle() {
        return new InstantCommand(() -> {
            if (swerve.getDistToSpeaker() < 3.5) {
                up.set(-0.2);
            } else {
                up.set(0);
            }
        }, this);
    }

    public void revIdle() {
        up.set(0.3);
    }

    public boolean rpmOk() {
        return up.getEncoder().getVelocity() <= -4500;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/UP_RPM",  up.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/DOWN_RPM",  up.getEncoder().getVelocity());
    }
}
