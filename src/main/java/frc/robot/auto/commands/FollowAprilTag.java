package frc.robot.auto.commands;

import java.util.function.Supplier;

import static frc.robot.constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT;

import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveDrive;

public class FollowAprilTag extends Command {
    private final SwerveDrive swerveDrive;
    private final PhotonCamera camera;
    private final Supplier<Pose2d> poseProvider;

    private PhotonTrackedTarget lastTarget;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints T_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController tController = new ProfiledPIDController(2, 0, 0, T_CONSTRAINTS);

    private static final int TAG_ID = 6;
    private static final Transform3d DISTANCE_FROM_TAG = new Transform3d(
        new Translation3d(1.5, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, Math.PI));

    public FollowAprilTag(SwerveDrive swerveDrive, PhotonCamera camera, Supplier<Pose2d> poseProvider) {
        this.swerveDrive = swerveDrive;
        this.camera = camera;
        this.poseProvider = poseProvider;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        tController.setTolerance(Units.degreesToRadians(3));
        tController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        lastTarget = null;

        Pose2d robotPose = poseProvider.get();

        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
        tController.reset(robotPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = poseProvider.get();

        Pose3d robotPose = new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        PhotonPipelineResult photonRes = camera.getLatestResult();

        if(photonRes.hasTargets()) {

            // Find the tag we want to chase
            Optional<PhotonTrackedTarget> targetOpt = photonRes.getTargets().stream()
                .filter(t -> t.getFiducialId() == TAG_ID)
                .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                .findFirst();

            if(targetOpt.isPresent()) {
                PhotonTrackedTarget target = targetOpt.get();
                // This is new target data, so recalculate the goal
                lastTarget = target;

                // Transform the robot's pose to find the camera's pose
                Pose3d cameraPose = robotPose.transformBy(APRILTAG_CAMERA_TO_ROBOT);

                // Trasnform the camera's pose to the target's pose
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d targetPose = cameraPose.transformBy(camToTarget);

                // Transform the tag's pose to set our goal
                Pose2d goalPose = targetPose.transformBy(DISTANCE_FROM_TAG).toPose2d();

                // Drive
                xController.setGoal(goalPose.getX());
                yController.setGoal(goalPose.getY());
                tController.setGoal(goalPose.getRotation().getRadians());
            }
        }

        if(lastTarget == null) {
            // No target has been visible
            swerveDrive.lockPosition();
        } else {
            // Drive to the target
            double xSpeed = xController.calculate(robotPose.getX());
            if(xController.atGoal()) {
                xSpeed = 0;
            }

            double ySpeed = yController.calculate(robotPose.getY());
            if(yController.atGoal()) {
                ySpeed = 0;
            }

            double tSpeed = tController.calculate(robotPose2d.getRotation().getRadians());
            if(tController.atGoal()) {
                tSpeed = 0;
            }

            swerveDrive.driveRelative(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, tSpeed, robotPose2d.getRotation()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.lockPosition();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
