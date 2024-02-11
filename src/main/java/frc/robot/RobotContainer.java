package frc.robot;

import org.photonvision.PhotonCamera;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.lib.zylve.Controller;
import frc.robot.auto.PhotonRunnable;
import frc.robot.auto.PoseEstimatorSubsystem;
import frc.robot.auto.commands.AmpAlign;
import frc.robot.auto.commands.FollowAprilTag;
import frc.robot.auto.commands.SpeakerAlign;
import frc.robot.constants.OperatorConstants;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ElevatorDown;
import frc.robot.elevator.commands.ElevatorUp;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeAmpScore;
import frc.robot.intake.commands.IntakeShooterFeed;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.commands.ShooterSpinDown;
import frc.robot.shooter.commands.ShooterSpinUp;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.swerve.SwerveDrive;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
    Controller controller = new Controller(OperatorConstants.CONTROLLER_PORT);
    private final PhotonCamera photonCamera = new PhotonCamera("Limelight");
    private final PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);

    private final SwerveDrive swerveDrive = new SwerveDrive();
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();

    private final PhotonRunnable photonRunnable = new PhotonRunnable(photonCamera);
    private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(swerveDrive::getRotation2d, swerveDrive::getModulePositions, photonRunnable);
    private final FollowAprilTag followAprilTag = new FollowAprilTag(swerveDrive, photonCamera, poseEstimator::getCurrentPose);

    private final SequentialCommandGroup ampSuperCommand = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new AmpAlign(swerveDrive, poseEstimator::getCurrentPose),
            new ElevatorUp(elevator)
        ),

        new IntakeAmpScore(intake),
        new ElevatorDown(elevator).withTimeout(0.25)
    );

    private final SequentialCommandGroup shooterSuperCommand = new SequentialCommandGroup(
        new ParallelCommandGroup(
            new SpeakerAlign(swerveDrive, poseEstimator::getCurrentPose),
            new ShooterSpinUp(shooter)
        ),

        new IntakeShooterFeed(intake),
        new ShooterSpinDown(shooter)
    );

    SendableChooser<Command> autoSelector = new SendableChooser<>();

    public RobotContainer() {
        configureSubsystems();
        configureAutonomous();
        configureSwerveDriveCommands();
        configureButtonBindings();

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
        PDH.setSwitchableChannel(true);
    }

    private void configureAutonomous() {
        autoSelector.addOption("Left Auto", new PathPlannerAuto("Left Auto"));
        autoSelector.addOption("Right Auto", new PathPlannerAuto("Right Auto"));
        autoSelector.setDefaultOption("Middle Auto", new PathPlannerAuto("Middle Auto"));

        // NamedCommands.registerCommand("Amp Score", AmpScore);
        // NamedCommands.registerCommand("Shoot Note", shootnotefeed);

        Shuffleboard.getTab("auto").add(autoSelector);
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
        poseEstimator.setAlliance(alliance);
    }
}
