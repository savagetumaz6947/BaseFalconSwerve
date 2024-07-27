package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStrip extends SubsystemBase {
    private AddressableLED led = new AddressableLED(8);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(62);
    private int position = 7;
    private int flashState = 0;

    private boolean shooting = false;
    private BooleanSupplier isAssistedIntaking;

    public LedStrip (BooleanSupplier isAssistedIntaking) {
        led.setLength(buffer.getLength());
        for (int i = 0; i < 7; i++) {
            buffer.setLED(i, Color.kWhite);
            buffer.setLED(61 - i, Color.kWhite);
        }
        led.setData(buffer);
        led.start();

        this.isAssistedIntaking = isAssistedIntaking;
    }

    @Override
    public void periodic() {
        if (shooting) {
            buffer.setLED(position % 32, Color.kGreen);
            buffer.setLED(61 - (position % 32), Color.kGreen);
            buffer.setLED((position - 7) % 32, Color.kBlack);
            buffer.setLED(61 - ((position - 7) % 32), Color.kBlack);
            position++;
            led.setData(buffer);
            return;
        }

        if (SmartDashboard.getBoolean("HasNote", false)) {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setLED(i, Color.kOrange);
            }
            position = 7;
            led.setData(buffer);
            return;
        }

        if (isAssistedIntaking.getAsBoolean()) {
            double offset = SmartDashboard.getNumber("Assisted Intake Offset", 1);
            // if positive, flash orange on the right and black on the left
            // if negative, flash orange on the left and black on the right
            // if zero, flash orange on the left and right
            if ((flashState % 6) >= 3) {
                if (offset < 0) {
                    for (int i = 0; i < 32; i++) {
                        buffer.setLED(i, Color.kOrange);
                        buffer.setLED(61 - i, Color.kBlack);
                    }
                } else if (offset > 0) {
                    for (int i = 0; i < 31; i++) {
                        buffer.setLED(i, Color.kBlack);
                        buffer.setLED(61 - i, Color.kOrange);
                    }
                } else {
                    for (int i = 0; i < 32; i++) {
                        buffer.setLED(i, Color.kOrange);
                        buffer.setLED(61 - i, Color.kOrange);
                    }
                }
            } else {
                for (int i = 0; i < buffer.getLength(); i++) {
                    buffer.setLED(i, Color.kBlack);
                }
            }
            flashState++;
            led.setData(buffer);
            return;
        }

        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            buffer.setLED(position % 32, Color.kWhite);
            buffer.setLED(61 - (position % 32), Color.kWhite);
        } else if (alliance.get() == Alliance.Blue) {
            buffer.setLED(position % 32, Color.kBlue);
            buffer.setLED(61 - (position % 32), Color.kBlue);
        } else {
            buffer.setLED(position % 32, Color.kRed);
            buffer.setLED(61 - (position % 32), Color.kRed);
        }
        buffer.setLED((position - 7) % 32, Color.kBlack);
        buffer.setLED(61 - ((position - 7) % 32), Color.kBlack);
        position++;
        led.setData(buffer);
    }

    public void shoot(boolean status) {
        shooting = status;
        if (status == true) {
            for (int i = 0; i < 7; i++) {
                buffer.setLED(i, Color.kGreen);
                buffer.setLED(61 - i, Color.kGreen);
            }
            for (int i = 7; i < buffer.getLength(); i++) {
                buffer.setLED(i, Color.kBlack);
                buffer.setLED(61 - i, Color.kBlack);
            }
            position = 7;
        }
    }
}
