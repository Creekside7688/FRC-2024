package frc.robot;

import org.photonvision.PhotonCamera;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.zylve.Controller;
import frc.robot.auto.AutoCommands;
import frc.robot.auto.PhotonRunnable;
import frc.robot.auto.commands.FollowAprilTag;
import frc.robot.constants.OperatorConstants;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.swerve.SwerveDrive;

public class RobotContainer {
    Controller controller = new Controller(OperatorConstants.CONTROLLER_PORT);
    private final PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);

    private final PhotonCamera photonCamera = new PhotonCamera("Limelight");
    private final PhotonRunnable photonRunnable = new PhotonRunnable(photonCamera);

    private final SwerveDrive swerveDrive = new SwerveDrive(photonRunnable);
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final AutoCommands autoCommands = new AutoCommands();

    private final FollowAprilTag followAprilTag = new FollowAprilTag(swerveDrive, photonCamera, swerveDrive::getPose);

    private final SequentialCommandGroup shooterSuperCommand = autoCommands.shooterSuperCommand(swerveDrive, shooter, intake);
    private final SequentialCommandGroup ampSuperCommand = autoCommands.ampSuperCommand(swerveDrive, elevator, intake);

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

    private void configureSubsystems() {
        PDH.setSwitchableChannel(true);
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

        controller.getA().whileTrue(followAprilTag);
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
