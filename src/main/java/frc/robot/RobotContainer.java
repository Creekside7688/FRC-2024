package frc.robot;

import org.photonvision.PhotonCamera;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.zylve.Controller;
import frc.robot.auto.PhotonRunnable;
import frc.robot.auto.commands.FollowAprilTag;
import frc.robot.constants.OperatorConstants;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ElevatorDown;
import frc.robot.elevator.commands.ElevatorUp;
import frc.robot.elevator.commands.ElevatorTempUp;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakePickup;
import frc.robot.intake.commands.IntakeShooterFeed;
import frc.robot.intake.commands.IntakeAmpScore;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.commands.ShooterSpinUp;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.swerve.SwerveDrive;

public class RobotContainer {
    Controller controller = new Controller(OperatorConstants.CONTROLLER_PORT);

    private final PhotonCamera photonCamera = new PhotonCamera("limelight");
    private final PhotonRunnable photonRunnable = new PhotonRunnable(photonCamera);

    private final SwerveDrive swerveDrive = new SwerveDrive(photonRunnable);
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final FollowAprilTag followAprilTag = new FollowAprilTag(swerveDrive, photonCamera, swerveDrive::getPose);

    private final Command elevatorAutoTest = new ElevatorUp(elevator);
    private final Command elevatorAutoDown = new ElevatorDown(elevator);
    private final Command elevatorTempUp = new ElevatorTempUp(elevator);

    private final Command intakePickup = new IntakePickup(intake);
    private final Command intakeShooterFeed = new IntakeShooterFeed(intake);
    private final Command intakeEject = new IntakeAmpScore(intake);

    private final Command shooterSpinUp = new ShooterSpinUp(shooter);

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
                    true, true
                ),
                swerveDrive
            )
        );
    }

    private void configureAutonomous() {
        autoSelector.addOption("Leave", new PathPlannerAuto("Leave"));
        autoSelector.addOption("Left Amp", new PathPlannerAuto("Left Amp"));
        autoSelector.setDefaultOption("Right Roundhouse Amp", new PathPlannerAuto("Right Roundhouse Amp"));

        // NamedCommands.registerCommand("Amp Score", AmpScore);
        // NamedCommands.registerCommand("Shoot Note", shootnotefeed);

        SmartDashboard.putData("Auto Selector", autoSelector);
    }

    private void configureSubsystemCommands() {
        controller.getX().whileTrue(followAprilTag);

        controller.getRightBumper().whileTrue(intakePickup);
        controller.getLeftTrigger().onTrue(intakeShooterFeed);
        controller.getLeftBumper().onTrue(intakeEject);

        controller.getY().onTrue(elevatorTempUp);
        controller.getA().onTrue(elevatorAutoTest);
        controller.getB().onTrue(elevatorAutoDown);

        controller.getRightTrigger().onTrue(shooterSpinUp);

        // controller.getY().whileTrue(elevatorSlowRaise);
        // controller.getA().whileTrue(ampSuperAlign);
    }

    private void configureSwerveDriveCommands() {
        controller.getLeftStick()
            .whileTrue(
                new RunCommand(
                    () -> swerveDrive.lockPosition(),
                    swerveDrive
                )
            );

        controller.getRightStick()
            .whileTrue(
                new RunCommand(
                    () -> swerveDrive.zeroHeading(),
                    swerveDrive
                )
            );
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }

    public void onAllianceChanged(Alliance alliance) {
        swerveDrive.setAlliance(alliance);
    }
}
