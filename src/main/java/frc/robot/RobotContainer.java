package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.lib.util.DeadzoneJoystick;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Angle.RiseToAngle;
import frc.robot.commands.Intake.AutoAimNote;
import frc.robot.commands.Shooting.AutoAimToShoot;
import frc.robot.subsystems.AngleSys;
import frc.robot.subsystems.BottomIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.MidIntake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final DeadzoneJoystick driver = new DeadzoneJoystick(0);
    private final DeadzoneJoystick operator = new DeadzoneJoystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    // private final int intakeBAxis = XboxController.Axis.kLeftY.value;
    // private final int intakeMAxis = XboxController.Axis.kRightY.value;
    // private final int angleDAxis = XboxController.Axis.kLeftTrigger.value;
    // private final int angleUAxis = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    private final JoystickButton startShooterButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton stopShooterButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton startPickupButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton stopPickupButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton kLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton eStopSwerve = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton autoPickupButton = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton zeroSwerveButton = new JoystickButton(driver, XboxController.Button.kStart.value);

    /* Operator Controls */
    private final int leftHangAxis = XboxController.Axis.kLeftY.value;
    private final int rightHangAxis = XboxController.Axis.kRightY.value;

    // private final JoystickButton kLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton kRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private boolean robotCentric = true;
    private int maxSpeedMode = 1;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final MidIntake midIntake = new MidIntake();
    private final BottomIntake bottomIntake = new BottomIntake();
    private final Shooter shooter = new Shooter();
    private final AngleSys angle = new AngleSys();
    private final Climber climber = new Climber();

    private final Vision intakeCam = new Vision("IntakeCam");

    public final Command pickUpNoteCommand = new InstantCommand(() -> {
            midIntake.moveMid(-.7);
            bottomIntake.moveDown(0.25);
        }, midIntake, bottomIntake).repeatedly().until(() -> midIntake.getColor().red >= 0.32).finallyDo(() -> {
            // new InstantCommand(() -> {
            //     midIntake.moveMid(0.3);
            // }, midIntake).repeatedly().withTimeout(0.15).alongWith(new InstantCommand(() -> {
            //     bottomIntake.moveDown(0);
            // }, bottomIntake).finallyDo(() -> {
            //     midIntake.moveMid(0);
            //     bottomIntake.moveDown(0);
            // }))
            bottomIntake.moveDown(0);
            midIntake.moveMid(0);
        });
    public final Command autoGrabNoteCommand = new SequentialCommandGroup(
            new AutoAimNote(s_Swerve, intakeCam),
            new ParallelDeadlineGroup(
                pickUpNoteCommand,
                new InstantCommand(() -> 
                    s_Swerve.driveChassis(new ChassisSpeeds(1.5, 0, 0))
                , s_Swerve).repeatedly()),
            new InstantCommand(() -> s_Swerve.driveChassis(new ChassisSpeeds(0, 0, 0)), s_Swerve)
        );

    // private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");
    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric,
                () -> maxSpeedMode
            )
        );

        midIntake.setDefaultCommand(new InstantCommand(() -> midIntake.moveMid(driver.getPOV() == 270 ? 1 : (driver.getPOV() == 90 ? -1 : 0)), midIntake));
        // bottomIntake.setDefaultCommand(new InstantCommand(() -> bottomIntake.moveDown(driver.getRawAxis(intakeBAxis)), bottomIntake));
        angle.setDefaultCommand(new InstantCommand(() -> angle.move(driver.getPOV() == 0 ? 1 : (driver.getPOV() == 180 ? -1 : 0)), angle));
        // angle.setDefaultCommand(new AutoRiseToAngle(angle, s_Swerve));
        climber.setDefaultCommand(new InstantCommand(() -> climber.move(operator.getRawAxis(leftHangAxis), operator.getRawAxis(rightHangAxis)), climber));

        // Register named commands
        NamedCommands.registerCommand("printWait", new InstantCommand(() -> System.out.println("PathPlanner: Waiting")));
        NamedCommands.registerCommand("PickUpNote", pickUpNoteCommand);
        NamedCommands.registerCommand("ShootAtMidNote", new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new RiseToAngle(37.5, angle),
                new InstantCommand(() -> midIntake.moveMid(-1), midIntake).repeatedly().withTimeout(1)
            ),
            new InstantCommand(() -> shooter.shoot(), shooter).repeatedly()
        ).andThen(new InstantCommand(() -> {
            midIntake.moveMid(0);
            shooter.stop();
        })));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        // zeroHeading.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        // robotCentricToggleButton.toggleOnTrue(new InstantCommand(() -> robotCentric = !robotCentric));
        // turboMode.onTrue(new InstantCommand(() -> maxSpeedMode = 2));
        // turboMode.onFalse(new InstantCommand(() -> maxSpeedMode = 1));
        startShooterButton.onTrue(new InstantCommand(() -> shooter.shoot(), shooter).repeatedly());
        stopShooterButton.onTrue(new InstantCommand(() -> shooter.stop(), shooter).repeatedly());
        // kY.onTrue(new InstantCommand(() -> midIntake.moveMid(-.7), midIntake).repeatedly().until(() -> midIntake.getColor().red >= 0.32).andThen(
        //     new InstantCommand(() -> midIntake.moveMid(0.3), midIntake).repeatedly().withTimeout(0.15)
        // ).finallyDo(() -> midIntake.moveMid(0)));
        startPickupButton.onTrue(new InstantCommand(() -> {
            midIntake.moveMid(-.7);
            bottomIntake.moveDown(0.4);
        }, midIntake, bottomIntake).repeatedly().until(() -> midIntake.getColor().red >= 0.32).finallyDo(() -> {
            // new InstantCommand(() -> {
            //     midIntake.moveMid(0.3);
            // }, midIntake).repeatedly().withTimeout(0.15).alongWith(new InstantCommand(() -> {
            //     bottomIntake.moveDown(0);
            // }, bottomIntake).finallyDo(() -> {
            //     midIntake.moveMid(0);
            //     bottomIntake.moveDown(0);
            // }))
            bottomIntake.moveDown(0);
            midIntake.moveMid(0);
        }));
        stopPickupButton.onTrue(new InstantCommand(() -> {
            midIntake.moveMid(0);
            bottomIntake.moveDown(0);
        }, midIntake, bottomIntake));
        // kLeftBumper.whileTrue(new MoveToAngle(60, angle));
        // kLeftBumper.whileTrue(new AutoAimNote(s_Swerve, intakeCam));
        kLeftBumper.onTrue(new AutoAimToShoot(s_Swerve));
        eStopSwerve.onTrue(new InstantCommand(() -> {}, s_Swerve));
        zeroSwerveButton.whileTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        autoPickupButton.whileTrue(autoGrabNoteCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
