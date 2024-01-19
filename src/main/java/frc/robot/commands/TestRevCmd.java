package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestRev;

public class TestRevCmd extends Command {
    private TestRev rev;
    private DoubleSupplier dbl;

    public TestRevCmd (TestRev rev, DoubleSupplier dbl) {
        this.rev = rev;
        addRequirements(rev);
        this.dbl = dbl;
    }

    @Override
    public void execute() {
        rev.move(dbl.getAsDouble());
    }
}
