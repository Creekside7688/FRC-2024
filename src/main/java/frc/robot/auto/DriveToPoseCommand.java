package frc.robot.auto;

import static frc.robot.constants.AutonomousConstants.X_P;
import static frc.robot.constants.AutonomousConstants.X_I;
import static frc.robot.constants.AutonomousConstants.X_D;
import static frc.robot.constants.AutonomousConstants.Y_P;
import static frc.robot.constants.AutonomousConstants.Y_I;
import static frc.robot.constants.AutonomousConstants.Y_D;
import static frc.robot.constants.AutonomousConstants.THETA_D;
import static frc.robot.constants.AutonomousConstants.THETA_I;
import static frc.robot.constants.AutonomousConstants.THETA_P;
import static frc.robot.constants.AutonomousConstants.TRANSLATION_TOLERANCE;
import static frc.robot.constants.AutonomousConstants.THETA_TOLERANCE;
import static frc.robot.constants.DriveConstants.MAXIMUM_SPEED_METRES_PER_SECOND;
import static frc.robot.constants.DriveConstants.MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND;
import static frc.robot.constants.VisionConstants.FIELD_WIDTH_METERS;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveDrive;

public class DriveToPoseCommand extends Command {

    /** Default constraints are 90% of max speed, accelerate to full speed in 1/3 second. */
    private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAXIMUM_SPEED_METRES_PER_SECOND * 0.5,
        MAXIMUM_SPEED_METRES_PER_SECOND);
    private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND * 0.4,
        MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND);

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    private final SwerveDrive swerveDrive;
    private final Supplier<Pose2d> poseProvider;
    private final Pose2d goalPose;
    private final boolean useAllianceColor;

    public DriveToPoseCommand(SwerveDrive swerveDrive, Supplier<Pose2d> poseProvider, Pose2d goalPose, boolean useAllianceColor) {
        this(swerveDrive, poseProvider, goalPose, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS, useAllianceColor);
    }

    public DriveToPoseCommand(SwerveDrive swerveDrive, Supplier<Pose2d> poseProvider, Pose2d goalPose, TrapezoidProfile.Constraints xyConstraints,
        TrapezoidProfile.Constraints omegaConstraints,
        boolean useAllianceColor) {
        this.swerveDrive = swerveDrive;
        this.poseProvider = poseProvider;
        this.goalPose = goalPose;
        this.useAllianceColor = useAllianceColor;

        xController = new ProfiledPIDController(X_P, X_I, X_D, xyConstraints);
        yController = new ProfiledPIDController(Y_P, Y_I, Y_D, xyConstraints);
        xController.setTolerance(TRANSLATION_TOLERANCE);
        yController.setTolerance(TRANSLATION_TOLERANCE);
        thetaController = new ProfiledPIDController(THETA_P, THETA_I, THETA_D, omegaConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(THETA_TOLERANCE);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        resetPIDControllers();

        Pose2d pose = goalPose;

        if(useAllianceColor && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            Translation2d transformedTranslation = new Translation2d(pose.getX(), FIELD_WIDTH_METERS - pose.getY());
            Rotation2d transformedHeading = pose.getRotation().times(-1);
            pose = new Pose2d(transformedTranslation, transformedHeading);
        }

        xController.setGoal(pose.getX());
        yController.setGoal(pose.getY());
        thetaController.setGoal(pose.getRotation().getRadians());
    }

    public boolean atGoal() {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }

    private void resetPIDControllers() {
        Pose2d robotPose = poseProvider.get();

        thetaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        Pose2d robotPose = poseProvider.get();
    
        double xSpeed = xController.calculate(robotPose.getX());

        if(xController.atGoal()) {
            xSpeed = 0;
        }

        double ySpeed = yController.calculate(robotPose.getY());

        if(yController.atGoal()) {
            ySpeed = 0;
        }

        double omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());

        if(thetaController.atGoal()) {
            omegaSpeed = 0;
        }

        swerveDrive.driveRelative(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
    }

    @Override
    public boolean isFinished() {
        return atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.lockPosition();
    }

}