package frc.robot;

import org.photonvision.PhotonCamera;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.zylve.Controller;
import frc.robot.auto.PhotonRunnable;
import frc.robot.constants.OperatorConstants;
import frc.robot.elevator.Elevator;

import frc.robot.elevator.commands.AmpScoreKill;
import frc.robot.elevator.commands.ElevatorClimb;
import frc.robot.elevator.commands.ElevatorDown;
import frc.robot.elevator.commands.ElevatorUp;
import frc.robot.elevator.commands.ElevatorSmallUp;

import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakePickup;
import frc.robot.intake.commands.IntakeShooterFeed;
import frc.robot.intake.commands.IntakeShooterFeedPreload;
import frc.robot.intake.commands.IntakeAmpScore;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.commands.ShooterSpinUp;
import frc.robot.shooter.commands.ShooterPreload;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.swerve.SwerveDrive;

public class RobotContainer {
    private final Controller controller = new Controller(OperatorConstants.CONTROLLER_PORT);

    private final PhotonCamera photonCamera = new PhotonCamera("limelight");
    private final PhotonRunnable photonRunnable = new PhotonRunnable(photonCamera);

    private final SwerveDrive swerveDrive = new SwerveDrive(photonRunnable);
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();

    private final Command elevatorUp = new ElevatorUp(elevator);
    private final Command elevatorDown = new ElevatorDown(elevator);
    private final Command ElevatorSmallUp = new ElevatorSmallUp(elevator);

    private final Command intakePickup = new IntakePickup(intake);

    private final Command intakeShooterFeed = new IntakeShooterFeed(intake);
    private final Command intakeShooterFeedPreload = new IntakeShooterFeedPreload(intake, shooter);

    private final Command intakeEject = new IntakeAmpScore(intake);
    private final Command ampScoreKill = new AmpScoreKill(elevator);
    private final Command shooterSpinUp = new ShooterSpinUp(shooter);
    private final Command shooterPreload = new ShooterPreload(shooter);

    // private final Command elevatorClimb = new ElevatorClimb(elevator);

    private final SequentialCommandGroup ampScore = new SequentialCommandGroup(
        new ElevatorUp(elevator),
        new IntakeAmpScore(intake),
        new ElevatorDown(elevator)
    );

    /*
     * private final ParallelCommandGroup speakerScore = new ParallelCommandGroup( new ShooterSpinUp(shooter), Commands.waitSeconds(3).andThen(new
     * IntakeShooterFeed(intake)) );
     */
    private final SequentialCommandGroup speakerScore = new SequentialCommandGroup(
        new ShooterSpinUp(shooter),
        new IntakeShooterFeed(intake)
    );

    private final SequentialCommandGroup speakerScorePreload = new SequentialCommandGroup(
        new ShooterPreload(shooter),
        new IntakeShooterFeedPreload(intake, shooter)
    );

    SendableChooser<Command> autoSelector = new SendableChooser<>();

    public RobotContainer() {
        configureAutonomous();
        configureSubsystemCommands();
        configureSwerveDriveCommands();

        swerveDrive.setDefaultCommand(
            new RunCommand(
                () -> swerveDrive.drive(
                    -MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.DEADBAND),
                    -MathUtil.applyDeadband(controller.getLeftX(), OperatorConstants.DEADBAND),
                    -MathUtil.applyDeadband(controller.getRightX(), OperatorConstants.DEADBAND),
                    controller.getLeftTrigger().getAsBoolean(),
                    true, true
                ),
                swerveDrive
            )
        );

    }

    private void configureSubsystemCommands() {
        controller.getLeftBumper().whileTrue(intakeEject);
        controller.getRightBumper().whileTrue(intakePickup);
        // controller.getB().whileTrue(elevatorClimb);
        controller.getUp().whileTrue(elevatorUp);
        controller.getDown().whileTrue(elevatorDown);
        // controller.getY().whileTrue(intakeShooterFeed);

        controller.getA().whileTrue(shooterSpinUp);
        // controller.getX().onTrue(speakerScorePreload);
        // controller.getX().whileTrue(speakerScorePreload);

        // controller.getB().onTrue(speakerScore);
        // controller.getB().whileTrue(shooterSpinUp);

        // controller.getRightTrigger().onTrue(ElevatorSmallUp);
        controller.getX().whileTrue(intakeShooterFeed);
    }

    private void configureSwerveDriveCommands() {
        controller.getRightStick()
            .whileTrue(
                new RunCommand(
                    () -> swerveDrive.zeroHeading(),
                    swerveDrive
                )
            );

        controller.getLeftStick().whileTrue(new RunCommand(() -> swerveDrive.lockPosition(), swerveDrive));

    }

    private void configureAutonomous() {
        NamedCommands.registerCommand("PickupNote", new IntakePickup(intake));
        NamedCommands.registerCommand("AmpNote", ampScore);
        NamedCommands.registerCommand("IntakeShooterFeedPreload", new IntakeShooterFeedPreload(intake, shooter));
        NamedCommands.registerCommand("ShooterPreload", new ShooterPreload(shooter));
        NamedCommands.registerCommand("speaker score", speakerScorePreload);

        autoSelector.addOption("Right Roundhouse Amp", new PathPlannerAuto("Right Roundhouse Amp"));
        autoSelector.addOption("Score speaker 1x", new PathPlannerAuto("Score Leave"));
        // autoSelector.addOption("Score speaker 1x", speakerScorePreload);
        autoSelector.addOption("Left Amp", new PathPlannerAuto("Left Amp"));
        autoSelector.setDefaultOption("Leave", new PathPlannerAuto("Leave"));

        SmartDashboard.putData("Autonomous Path", autoSelector);
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }

    public void onAllianceChanged(Alliance alliance) {
        swerveDrive.setAlliance(alliance);
    }
}
