package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.lib.util.DeadzoneJoystick;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Angle.AutoRiseToAngle;
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

    /* Driver Buttons */
    private final Trigger autoPickupButton = new Trigger(() -> {
        return driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8;
    });
    private final JoystickButton autoDriveToAmpPos = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton autoShootButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton driverCancelSwerve = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton zeroSwerveButton = new JoystickButton(driver, XboxController.Button.kStart.value);

    /* Operator Controls */
    private final int leftHangAxis = XboxController.Axis.kLeftY.value;
    private final int rightHangAxis = XboxController.Axis.kRightY.value;

    /* Operator Buttons */
    private final Trigger manualAngleUp = new Trigger(() -> (operator.getPOV() == 0));
    private final Trigger manualAngleDown = new Trigger(() -> (operator.getPOV() == 180));
    private final JoystickButton resetAngle = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton driveToTrap = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton startShooterButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton stopShooterButton = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton startPickupButton = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton stopPickupButton = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton operatorCancelSwerve = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);

    private boolean robotCentric = true;
    private int maxSpeedMode = 1;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final MidIntake midIntake = new MidIntake();
    private final BottomIntake bottomIntake = new BottomIntake();
    private final Shooter shooter = new Shooter(s_Swerve);
    private final AngleSys angle = new AngleSys();
    private final Climber climber = new Climber();

    private final RiseToAngle riseToTrapAngle = new RiseToAngle(() -> 50, angle);
    private final RiseToAngle riseToAmpPosAngle = new RiseToAngle(() -> 36.734, angle);

    private final Vision intakeCam = new Vision("IntakeCam");

    private final Command autoAimToShootCommand = new AutoAimToShoot(s_Swerve);

    public final Command pickUpNoteCommand = new InstantCommand(() -> {
            midIntake.moveMid(-.7);
            bottomIntake.moveDown(0.5);
        }, midIntake, bottomIntake).repeatedly().until(() -> midIntake.getColor().red >= 0.32).finallyDo(() -> {
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
    public final AutoRiseToAngle autoRiseToAngleCommand = new AutoRiseToAngle(angle, s_Swerve);
    public final Command dropIntake = new InstantCommand(() -> {
        midIntake.moveMid(-1);
        angle.move(0);
        bottomIntake.moveDown(0);
        shooter.revIdle();
    }, bottomIntake, midIntake, angle, shooter).repeatedly().withTimeout(0.3).andThen(
        new SequentialCommandGroup(
            new InstantCommand(() -> midIntake.moveMid(1), midIntake).repeatedly().withTimeout(0.3),
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new InstantCommand(() -> midIntake.moveMid(0), midIntake),
                    new InstantCommand(() -> System.out.println("waiting for shooter...")).repeatedly().until(() -> shooter.rpmOk()).withTimeout(3),
                    new InstantCommand(() -> midIntake.moveMid(-1), midIntake).repeatedly().withTimeout(1)
                ),
                new InstantCommand(() -> shooter.shoot(), shooter).repeatedly()
            ).andThen(shooter.idle())
        )
    );
    public final Command shootCommand = new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    System.out.println("waiting for finish (autoRise: " + autoRiseToAngleCommand.isFinished() + ")");
                }).repeatedly().until(() -> autoRiseToAngleCommand.isFinished()).withTimeout(2),
                new InstantCommand(() -> {
                    System.out.println("waiting for rpm (rpm: " + shooter.rpmOk() + ")");
                }).repeatedly().until(() -> shooter.rpmOk()).withTimeout(2),
                new InstantCommand(() -> {
                    System.out.println("waiting for aimtoshoot (aimtoshoot: " + autoAimToShootCommand.isFinished() + ")");
                }).repeatedly().until(() -> autoAimToShootCommand.isFinished()).withTimeout(2),
                new InstantCommand(() -> midIntake.moveMid(-1), midIntake).repeatedly().withTimeout(1)
            ),
            autoAimToShootCommand,
            new InstantCommand(() -> shooter.shoot(), shooter).repeatedly()
        ).finallyDo(() -> shooter.idle());

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
        // angle.setDefaultCommand(new InstantCommand(() -> angle.move(driver.getPOV() == 0 ? 0.5 : (driver.getPOV() == 180 ? -0.5 : 0)), angle));
        shooter.setDefaultCommand(shooter.idle());
        angle.setDefaultCommand(autoRiseToAngleCommand.repeatedly());
        // angle.setDefaultCommand(new AutoRiseToAngle(angle, s_Swerve));
        climber.setDefaultCommand(new InstantCommand(() -> climber.move(operator.getRawAxis(leftHangAxis), operator.getRawAxis(rightHangAxis)), climber));

        // Register named commands
        NamedCommands.registerCommand("PickUpNote", pickUpNoteCommand);
        NamedCommands.registerCommand("Shoot", shootCommand);
        NamedCommands.registerCommand("RiseToAngleAtAmpPos", riseToAmpPosAngle.repeatedly());
        // NamedCommands.registerCommand("ShooterReady", new InstantCommand(() -> shooter.shoot(), shooter).repeatedly());

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
        startShooterButton.onTrue(new InstantCommand(() -> shooter.shoot(), shooter));
        stopShooterButton.onTrue(new InstantCommand(() -> shooter.idle(), shooter));
        startPickupButton.onTrue(new InstantCommand(() -> {
            midIntake.moveMid(-.7);
            bottomIntake.moveDown(0.4);
        }, midIntake, bottomIntake).repeatedly().until(() -> midIntake.getColor().red >= 0.32).finallyDo(() -> {
            bottomIntake.moveDown(0);
            midIntake.moveMid(0);
        }));
        stopPickupButton.onTrue(new InstantCommand(() -> {
            midIntake.moveMid(0);
            bottomIntake.moveDown(0);
        }, midIntake, bottomIntake));
        autoShootButton.onTrue(shootCommand);
        autoDriveToAmpPos.onTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ToAmpShootSpot"), new PathConstraints(2.5, 5, 360, 720)));
        // kLeftBumper.onTrue(dropIntake);
        operatorCancelSwerve.onTrue(new InstantCommand(() -> {}, s_Swerve));
        driverCancelSwerve.onTrue(new InstantCommand(() -> {}, s_Swerve));
        zeroSwerveButton.whileTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        autoPickupButton.onTrue(autoGrabNoteCommand);

        manualAngleUp.whileTrue(new InstantCommand(() -> angle.move(0.5), angle).repeatedly());
        manualAngleDown.whileTrue(new InstantCommand(() -> angle.move(-0.5), angle).repeatedly());
        resetAngle.onTrue(new InstantCommand(() -> angle.reset()));

        driveToTrap.onTrue(
            // riseToTrapAngle.repeatedly()
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    // AutoBuilder.pathfindToPose(new Pose2d(4.17, 5.303, new Rotation2d(2.041)), new PathConstraints(1.5, 5, 360, 720)),
                    AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("ToTrap1"), new PathConstraints(1.5, 5, 360, 720)),
                    new InstantCommand(() -> {
                        System.out.println("Waiting for angle... (" + riseToTrapAngle.isFinished() + ")");
                    }).repeatedly().until(() -> riseToTrapAngle.isFinished()),
                    new InstantCommand(() -> midIntake.moveMid(-1), midIntake).repeatedly().withTimeout(1)
                ),
                riseToTrapAngle.repeatedly(),
                new InstantCommand(() -> shooter.shoot(), shooter).repeatedly()
            ).finallyDo(() -> midIntake.moveMid(0))

        );

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
