package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStrip extends SubsystemBase {
    private DigitalOutput green = new DigitalOutput(1);
    private DigitalOutput red = new DigitalOutput(2);
    private DigitalOutput blue = new DigitalOutput(3);

    private boolean hasNote = false;
    private boolean shooterReady = false;

    public LedStrip () {
        green.enablePWM(0);
        red.enablePWM(0);
        blue.enablePWM(255);
    }

    @Override
    public void periodic() {
        hasNote = SmartDashboard.getBoolean("HasNote", hasNote);
        if (hasNote && shooterReady) {
            brightGreen();
        } else if (hasNote) {
            brightOrange();
        } else {
            brightBlue();
        }
    }

    public void shooterReady (boolean state) {
        this.shooterReady = state;
    }

    private void brightOrange () {
        setGRB(128, 255, 0);
    }

    private void brightBlue () {
        setGRB(0, 0, 255);
    }

    private void brightGreen () {
        setGRB(255, 0, 0);
    }

    private void setGRB (int g, int r, int b) {
        green.updateDutyCycle(g/255.0);
        red.updateDutyCycle(r/255.0);
        blue.updateDutyCycle(b/255.0);
    }
}
