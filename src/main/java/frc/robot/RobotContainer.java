package frc.robot;

import org.photonvision.PhotonCamera;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.zylve.Controller;
import frc.robot.auto.AutoCommands;
import frc.robot.auto.PhotonRunnable;
import frc.robot.auto.commands.FollowAprilTag;
import frc.robot.constants.OperatorConstants;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakePickup;
import frc.robot.intake.commands.IntakeAmpScore;
import frc.robot.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.swerve.SwerveDrive;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
    Controller controller = new Controller(OperatorConstants.CONTROLLER_PORT);

    private final PhotonCamera photonCamera = new PhotonCamera("limelight");
    private final PhotonRunnable photonRunnable = new PhotonRunnable(photonCamera);

    private final SwerveDrive swerveDrive = new SwerveDrive(photonRunnable);
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final AutoCommands autoCommands = new AutoCommands();

    private final FollowAprilTag followAprilTag = new FollowAprilTag(swerveDrive, photonCamera, swerveDrive::getPose);

    private final SequentialCommandGroup ampSuperAlign = autoCommands.ampSuperAlign(swerveDrive, elevator, photonCamera);
    private final SequentialCommandGroup ampSuperShoot = autoCommands.ampSuperShoot(elevator, intake);

    private final SequentialCommandGroup shooterSuperAlign = autoCommands.shooterSuperAlign(swerveDrive, shooter, photonCamera);
    private final SequentialCommandGroup shooterSuperShoot = autoCommands.shooterSuperShoot(shooter, intake);

    private final Command intakePickup = new IntakePickup(intake);
    private final Command eject = new IntakeAmpScore(intake);

    SendableChooser<Command> autoSelector = new SendableChooser<>();

    public RobotContainer() {
        configureSubsystems();
        configureAutonomous();
        configureSwerveDriveCommands();
        configureSuperCommands();

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

    private void configureButtonBindings() {
        controller.getA().whileTrue(elevator.getSysIdQuasistatic(SysIdRoutine.Direction.kForward));
        controller.getB().whileTrue(elevator.getSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        controller.getX().whileTrue(elevator.getSysIdDynamic(SysIdRoutine.Direction.kForward));
        controller.getY().whileTrue(elevator.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private void configureSubsystems() {
    }

    private void configureAutonomous() {
        autoSelector.addOption("Left Auto", new PathPlannerAuto("Left Auto"));
        autoSelector.addOption("Right Auto", new PathPlannerAuto("Right Auto"));
        autoSelector.setDefaultOption("Middle Auto", new PathPlannerAuto("Middle Auto"));

        // NamedCommands.registerCommand("Amp Score", AmpScore);
        // NamedCommands.registerCommand("Shoot Note", shootnotefeed);

        SmartDashboard.putData("Auto Selector", autoSelector);
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

        controller.getX().whileTrue(followAprilTag);
        controller.getA().whileTrue(ampSuperAlign);
        controller.getRightTrigger().onTrue(ampSuperShoot);
        controller.getY().whileTrue(ampSuperAlign);
        controller.getLeftTrigger().onTrue(shooterSuperShoot);
        controller.getRightBumper().onTrue(intakePickup);
        controller.getLeftBumper().onTrue(eject);

    }

    private void configureSuperCommands() {

    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }

    public void onAllianceChanged(Alliance alliance) {
        swerveDrive.setAlliance(alliance);
    }
}
