package frc.robot;

// import java.util.List;

// import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;

import frc.lib.zylve.Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.Snap;

public class RobotContainer {
    private final SwerveDrive swerveDrive = new SwerveDrive();
    private final Snap snap90 = new Snap(90, swerveDrive);
    
    Controller controller = new Controller(OperatorConstants.CONTROLLER_PORT);

    public RobotContainer() {
        configureButtonBindings();
        swerveDrive.setDefaultCommand(
                new RunCommand(
                        () -> swerveDrive.joystickDrive(
                                -MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.DEADBAND),
                                -MathUtil.applyDeadband(controller.getLeftX(), OperatorConstants.DEADBAND),
                                -MathUtil.applyDeadband(controller.getRightX(), OperatorConstants.DEADBAND),
                                true, true),
                        swerveDrive));

        controller.getLeftTrigger().onTrue(snap90);
    
    }

    private void configureButtonBindings() {
        controller.getRightBumper()
                .whileTrue(new RunCommand(
                        () -> swerveDrive.lockPosition(),
                        swerveDrive));
    }

    public Command getAutonomousCommand() {
        // // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        // AutonomousConstants.MAXIMUM_SPEED_METRES_PER_SECOND,
        // AutonomousConstants.MAXIMUM_ACCELERATION_METRES_PER_SECOND_SQUARED)
        // // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(DriveConstants.SWERVE_KINEMATICS);

        // // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // // Start at the origin facing the +X dir ection
        // new Pose2d(0, 0, new Rotation2d(0)),
        // // Pass through these two interior waypoints, making an 's' curve path
        // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // // End 3 meters straight ahead of where we started, facing forward
        // new Pose2d(3, 0, new Rotation2d(0)),
        // config);

        // ProfiledPIDController thetaController = new ProfiledPIDController(
        // AutonomousConstants.THETACONTROLLER_P, 0, 0, AutonomousConstants.THETA_CONTROLLER_CONSTRAINTS);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        // exampleTrajectory,
        // swerveDrive::getPose, // Functional interface to feed supplier
        // DriveConstants.SWERVE_KINEMATICS,

        // // Position controllers
        // new PIDController(AutonomousConstants.XCONTROLER_P, 0, 0),
        // new PIDController(AutonomousConstants.YCONTROLLER_P, 0, 0),
        // thetaController,
        // swerveDrive::setModuleStates,
        // swerveDrive);

        // // Reset odometry to the starting pose of the trajectory.
        // swerveDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> swerveDrive.drive(0, 0, 0, false, false));

        return null;
    }
}
