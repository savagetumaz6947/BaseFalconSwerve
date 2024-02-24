package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Swerve;

public class Shooter extends SubsystemBase {
    private CANSparkMax up = new CANSparkMax(62, MotorType.kBrushless);
    private CANSparkMax down = new CANSparkMax(61, MotorType.kBrushless);

    private Swerve swerve;

    public Shooter(Swerve swerve) {
        this.swerve = swerve;
    }

    public Command shootRepeatedly() {
        return this.runEnd(() -> {
            up.set(-1);
            down.set(1);
        }, () -> up.set(0));
    }

    public boolean inRange() {
        return swerve.getDistToSpeaker() < 3.5;
    }

    public Command idle() {
        return new InstantCommand(() -> {
            if (inRange()) {
                up.set(-0.2);
                down.set(0.2);
            } else {
                up.set(0);
                down.set(0);
            }
        }, this).repeatedly().finallyDo(() -> {
            up.set(0);
            down.set(0);
        });
    }

    public Command revIdle() {
        return this.runEnd(() -> {
            up.set(0.3);
            down.set(-0.3);
        }, () -> {
            up.set(0);
            down.set(0);
        });
    }

    public void reverse() {
        up.set(0.5);
        down.set(-0.5);
    }

    public void stop() {
        up.set(0);
        down.set(0);
    }

    public boolean rpmOk() {
        return up.getEncoder().getVelocity() <= -4800;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/UP_RPM",  up.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/DOWN_RPM",  up.getEncoder().getVelocity());

        SmartDashboard.putBoolean("Shooter In Range", inRange());
    }
}
