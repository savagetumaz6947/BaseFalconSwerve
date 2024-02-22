package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStrip extends SubsystemBase {
    private AddressableLED led = new AddressableLED(1);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(80);

    public LedStrip () {
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        // hasNote = SmartDashboard.getBoolean("HasNote", hasNote);
        // if (hasNote && shooterReady) {
        //     brightGreen();
        // } else if (hasNote) {
        //     brightOrange();
        // } else {
        //     brightBlue();
        // }
    }

    // public void shooterReady (boolean state) {
    //     this.shooterReady = state;
    // }

    public void brightOrange () {
        setRGB(128, 255, 0);
    }

    public void brightBlue () {
        setColor(Color.kBlue);
    }

    public void brightGreen () {
        setColor(Color.kGreen);
    }

    private void setRGB (int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        led.setData(buffer);
    }

    private void setColor (Color color) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
        led.setData(buffer);
    }
}
