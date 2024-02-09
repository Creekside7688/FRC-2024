package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.lib.zylve.Controller;
import frc.robot.auto.AmpScore;
import frc.robot.auto.FollowAprilTag;
import frc.robot.auto.PhotonRunnable;
import frc.robot.auto.PoseEstimatorSubsystem;
import frc.robot.auto.ShootNote;
import frc.robot.constants.OperatorConstants;
import frc.robot.elevator.Elevator;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.swerve.SwerveDrive;

public class RobotContainer {
    Controller controller = new Controller(OperatorConstants.CONTROLLER_PORT);

    private final PowerDistribution PDH = new PowerDistribution(1,ModuleType.kRev);
    private final SwerveDrive swerveDrive = new SwerveDrive();

    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();


    private final PhotonCamera photonCamera  = new PhotonCamera("Limelight");
    private final PhotonRunnable photonRunnable = new PhotonRunnable(photonCamera);
    private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(swerveDrive::getRotation2d, swerveDrive::getModulePositions, photonRunnable);
    private final FollowAprilTag followAprilTag = new FollowAprilTag(swerveDrive, photonCamera, poseEstimator::getCurrentPose);

    private final AmpScore AmpScore = new AmpScore(elevator, intake);
    private final ShootNote shootnotefeed = new ShootNote(shooter, intake);

    // @SuppressWarnings("unused")
    // private final Elevator elevator = new Elevator();

    // @SuppressWarnings("unused")
    // private final Intake intake = new Intake();

    // @SuppressWarnings("unused")
    // private final Shooter shooter = new Shooter();

    SendableChooser<Command> autoSelector = new SendableChooser<>();

    public RobotContainer() {
        configureButtonBindings();

        PDH.setSwitchableChannel(true);
        autoSelector.addOption("Left Auto", new PathPlannerAuto("Left Auto"));
        autoSelector.addOption("Right Auto", new PathPlannerAuto("Right Auto"));
        autoSelector.setDefaultOption("Middle Auto", new PathPlannerAuto("Middle Auto"));

        NamedCommands.registerCommand("Amp Score", AmpScore);
        NamedCommands.registerCommand("Shoot Note", shootnotefeed);
        

        Shuffleboard.getTab("auto").add(autoSelector);

        swerveDrive.setDefaultCommand(
            new RunCommand(
                () -> swerveDrive.drive(
                    -MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.DEADBAND),
                    -MathUtil.applyDeadband(controller.getLeftX(), OperatorConstants.DEADBAND),
                    -MathUtil.applyDeadband(controller.getRightX(), OperatorConstants.DEADBAND),
                    true, true),
                swerveDrive));
    }

    private void configureButtonBindings() {
        controller.getLeftStick()
            .whileTrue(new RunCommand(
                () -> swerveDrive.lockPosition(),
                swerveDrive));

        controller.getRightStick()
            .whileTrue(new RunCommand(
                () -> swerveDrive.zeroHeading(),
                swerveDrive));


        controller.getA().whileTrue(followAprilTag);
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }

    public void onAllianceChanged(Alliance alliance) {
        // poseEstimator.setAlliance(alliance);
    }
}
