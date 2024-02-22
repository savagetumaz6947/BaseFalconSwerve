package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;
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
import frc.robot.subsystems.IntakeAngle;
import frc.robot.subsystems.LedStrip;
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
    private final DoubleSupplier translationAxis = () -> -driver.getRawAxis(XboxController.Axis.kLeftY.value);
    private final DoubleSupplier strafeAxis = () -> -driver.getRawAxis(XboxController.Axis.kLeftX.value);
    private final DoubleSupplier rotationAxis = () -> -driver.getRawAxis(XboxController.Axis.kRightX.value);

    /* Driver Buttons */
    private final Trigger autoPickupButton = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);
    private final JoystickButton autoShootButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final Trigger autoDriveToAmpPosBtn = new Trigger(() -> {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return driver.getRawButton(XboxController.Button.kX.value);
        } else {
            return driver.getRawButton(XboxController.Button.kB.value);
        }
    });
    private final JoystickButton autoDriveToMidPosBtn = new JoystickButton(driver, XboxController.Button.kA.value);
    private final Trigger autoDriveToStagePosBtn = new Trigger(() -> {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return driver.getRawButton(XboxController.Button.kB.value);
        } else {
            return driver.getRawButton(XboxController.Button.kX.value);
        }
    });
    private final Trigger autoAimWhileDriveToggle = new Trigger(() -> driver.getPOV() == 270);
    private final JoystickButton autoDriveToSourceBtn = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driverCancelSwerveBtn = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton fodButton = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final Trigger maxSpeedUp = new Trigger(() -> driver.getPOV() == 0);
    private final Trigger maxSpeedDown = new Trigger(() -> driver.getPOV() == 180);

    /* Operator Controls */
    private final DoubleSupplier leftClimbAxis = () -> operator.getRawAxis(XboxController.Axis.kLeftY.value);
    private final DoubleSupplier rightClimbAxis = () -> operator.getRawAxis(XboxController.Axis.kRightY.value);
    private final DoubleSupplier bottomIntakeAxis = () -> -operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) + operator.getRawAxis(XboxController.Axis.kRightTrigger.value);

    /* Operator Buttons */
    private final Trigger manualAngleUpBtn = new Trigger(() -> (operator.getPOV() == 0));
    private final Trigger manualAngleDownBtn = new Trigger(() -> (operator.getPOV() == 180));
    private final Trigger manualMidIntakeUpBtn = new Trigger(() -> (operator.getPOV() == 90));
    private final Trigger manualMidIntakeDownBtn = new Trigger(() -> (operator.getPOV() == 270));
    private final JoystickButton resetAngleBtn = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton compositeKillBtn = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton manualPickupBtn = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final Trigger trap1Btn = new Trigger(() -> {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return operator.getRawButton(XboxController.Button.kX.value);
        } else {
            return operator.getRawButton(XboxController.Button.kB.value);
        }
    });
    private final JoystickButton trap2Btn = new JoystickButton(operator, XboxController.Button.kY.value);
    private final Trigger trap3Btn = new Trigger(() -> {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return operator.getRawButton(XboxController.Button.kB.value);
        } else {
            return operator.getRawButton(XboxController.Button.kX.value);
        }
    });
    private final JoystickButton manualStartShooterBtn = new JoystickButton(operator, XboxController.Button.kA.value);

    private int maxSpeedMode = 1;
    private boolean enableAutoAimWhileDrive = true;

    /* Subsystems */
    private final LedStrip ledStrip = new LedStrip();
    private final Swerve s_Swerve = new Swerve();
    private final MidIntake midIntake = new MidIntake();
    private final BottomIntake bottomIntake = new BottomIntake();
    private final Shooter shooter = new Shooter(s_Swerve);
    private final AngleSys angle = new AngleSys();
    private final Climber climber = new Climber();
    private final IntakeAngle intakeAngle = new IntakeAngle();

    private final RiseToAngle riseToTrap1Angle = new RiseToAngle(() -> 50, angle);
    private final RiseToAngle riseToTrap2Angle = new RiseToAngle(() -> 50, angle);
    private final RiseToAngle riseToTrap3Angle = new RiseToAngle(() -> 50, angle);

    /* Command Definitions */
    private final AutoAimToShoot autoAimToShootCommand = new AutoAimToShoot(s_Swerve);
    private final AutoRiseToAngle autoRiseToAngleCommand = new AutoRiseToAngle(angle, s_Swerve);

    private final TeleopSwerve teleopSwerve = new TeleopSwerve(
                s_Swerve,
                translationAxis,
                strafeAxis,
                rotationAxis,
                () -> !fodButton.getAsBoolean(),
                () -> maxSpeedMode,
                shooter::inRange,
                () -> enableAutoAimWhileDrive
            );
    private final Command pickUpNoteCommand = new InstantCommand(() -> {
            midIntake.rawMove(-.7);
            bottomIntake.rawMove(0.5);
        }, midIntake, bottomIntake).repeatedly().until(() -> midIntake.hasNote()).finallyDo(() -> {
            bottomIntake.rawMove(0);
            midIntake.rawMove(0);
        });
    private final Command autoGrabNoteCommand = new SequentialCommandGroup(
            new AutoAimNote(s_Swerve, bottomIntake.getCamera()),
            new ParallelDeadlineGroup(
                /* PICK UP NOTE COMMAND */
                new InstantCommand(() -> {
                    midIntake.rawMove(-.7);
                    bottomIntake.rawMove(0.5);
                }, midIntake, bottomIntake).repeatedly().until(() -> midIntake.hasNote()).finallyDo(() -> {
                    bottomIntake.rawMove(0);
                    midIntake.rawMove(0);
                })
                /* PICK UP NOTE COMMAND END */,
                s_Swerve.run(() -> s_Swerve.driveChassis(new ChassisSpeeds(1.5, 0, 0)))
            ),
            s_Swerve.runOnce(() -> s_Swerve.driveChassis(new ChassisSpeeds(0, 0, 0)))
        );
    private final Command autoShootCommandInAuto = new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> autoRiseToAngleCommand.isFinished()).withTimeout(2),
                new WaitUntilCommand(() -> shooter.rpmOk()).withTimeout(2),
                new WaitUntilCommand(() -> autoAimToShootCommand.isFinished()).withTimeout(2),
                new InstantCommand(() -> ledStrip.shooterReady(true)),
                midIntake.run(() -> midIntake.rawMove(-1)).withTimeout(1)
            ),
            autoAimToShootCommand.repeatedly(),
            shooter.shootRepeatedly()
        ).finallyDo(() -> {
            shooter.idle();
            midIntake.rawMove(0);
            ledStrip.shooterReady(false);
        });
    private final Command autoAimToShootCommandInTeleop = new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> autoRiseToAngleCommand.isFinished()).withTimeout(2),
                new WaitUntilCommand(() -> shooter.rpmOk()).withTimeout(2),
                new WaitUntilCommand(() -> teleopSwerve.atSetpoint()).withTimeout(2),
                new InstantCommand(() -> ledStrip.shooterReady(true)),
                midIntake.run(() -> midIntake.rawMove(-1)).withTimeout(1)
            ),
            shooter.shootRepeatedly()
        ).finallyDo(() -> {
            shooter.idle();
            midIntake.rawMove(0);
            ledStrip.shooterReady(false);
        });

    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Set default commands
        s_Swerve.setDefaultCommand(teleopSwerve);
        shooter.setDefaultCommand(shooter.idle());
        angle.setDefaultCommand(autoRiseToAngleCommand.repeatedly());
        climber.setDefaultCommand(climber.run(() -> climber.move(leftClimbAxis, rightClimbAxis)));
        intakeAngle.setDefaultCommand(intakeAngle.run(() -> intakeAngle.rawMove(bottomIntakeAxis.getAsDouble() * 0.5)));

        // Register named commands
        NamedCommands.registerCommand("PickUpNote", pickUpNoteCommand);
        NamedCommands.registerCommand("Shoot", autoShootCommandInAuto);

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
        autoAimWhileDriveToggle.toggleOnTrue(new InstantCommand(() -> enableAutoAimWhileDrive = !enableAutoAimWhileDrive));
        driverCancelSwerveBtn.onTrue(s_Swerve.runOnce(() -> s_Swerve.driveChassis(new ChassisSpeeds())));
        autoPickupButton.onTrue(autoGrabNoteCommand);
        autoShootButton.onTrue(autoAimToShootCommandInTeleop);
        autoDriveToAmpPosBtn.onTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ToAmpShootSpot"), Constants.defaultPathConstraints));
        autoDriveToMidPosBtn.onTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ToMidShootSpot"), Constants.defaultPathConstraints));
        autoDriveToStagePosBtn.onTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ToStageShootSpot"), Constants.defaultPathConstraints));
        autoDriveToSourceBtn.onTrue(AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ToSource"), Constants.defaultPathConstraints));
        maxSpeedUp.toggleOnTrue(new InstantCommand(() ->
            maxSpeedMode = maxSpeedMode + 1 < Constants.Swerve.speedSelection.length ? maxSpeedMode + 1 : maxSpeedMode
        ));
        maxSpeedDown.toggleOnTrue(new InstantCommand(() ->
            maxSpeedMode = maxSpeedMode - 1 >= 0 ? maxSpeedMode - 1 : maxSpeedMode
        ));

        /* Operator Buttons */
        manualAngleUpBtn.whileTrue(angle.run(() -> angle.move(0.5)).repeatedly().finallyDo(() -> angle.move(0)));
        manualAngleDownBtn.whileTrue(angle.run(() -> angle.move(-0.5)).repeatedly().finallyDo(() -> angle.move(0)));
        manualMidIntakeUpBtn.whileTrue(midIntake.run(() -> midIntake.rawMove(-0.7)).repeatedly().finallyDo(() -> midIntake.rawMove(0)));
        manualMidIntakeDownBtn.whileTrue(midIntake.run(() -> midIntake.rawMove(0.7)).repeatedly().finallyDo(() -> midIntake.rawMove(0)));
        resetAngleBtn.onTrue(new InstantCommand(() -> angle.reset()));
        compositeKillBtn.whileTrue(new InstantCommand(() -> {
            s_Swerve.driveChassis(new ChassisSpeeds());
            midIntake.rawMove(0);
            bottomIntake.rawMove(0);
            shooter.stop();
            angle.move(0);
            climber.move(() -> 0, () -> 0);
        }, s_Swerve, midIntake, bottomIntake, shooter, angle, climber));
        manualPickupBtn.onTrue(pickUpNoteCommand);
        trap1Btn.onTrue(new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                AutoBuilder.pathfindThenFollowPath(
                    PathPlannerPath.fromPathFile("ToTrap1"), new PathConstraints(2.5, 5, Math.toRadians(420), Math.toRadians(720))),
                new WaitUntilCommand(() -> riseToTrap1Angle.isFinished()).withTimeout(2),
                midIntake.run(() -> midIntake.rawMove(-1)).withTimeout(1)
            ),
            riseToTrap1Angle.repeatedly(),
            shooter.shootRepeatedly()
        ).finallyDo(() -> midIntake.rawMove(0)));
        trap2Btn.onTrue(new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                AutoBuilder.pathfindThenFollowPath(
                    PathPlannerPath.fromPathFile("ToTrap2"), Constants.defaultPathConstraints),
                new WaitUntilCommand(() -> riseToTrap2Angle.isFinished()).withTimeout(2),
                midIntake.run(() -> midIntake.rawMove(-1)).withTimeout(1)
            ),
            riseToTrap2Angle.repeatedly(),
            shooter.shootRepeatedly()
        ).finallyDo(() -> midIntake.rawMove(0)));
        trap3Btn.onTrue(new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                AutoBuilder.pathfindThenFollowPath(
                    PathPlannerPath.fromPathFile("ToTrap3"), Constants.defaultPathConstraints),
                new WaitUntilCommand(() -> riseToTrap3Angle.isFinished()).withTimeout(2),
                midIntake.run(() -> midIntake.rawMove(-1)).withTimeout(1)
            ),
            riseToTrap3Angle.repeatedly(),
            shooter.shootRepeatedly()
        ).finallyDo(() -> midIntake.rawMove(0)));
        manualStartShooterBtn.onTrue(shooter.shootRepeatedly());
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
