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
import frc.robot.constants.VisionConstants;
import frc.robot.swerve.SwerveDrive;

public class FollowAprilTag extends Command {
    private final SwerveDrive swerveDrive;
    private final PhotonCamera camera;
    private final Supplier<Pose2d> poseProvider;

    private PhotonTrackedTarget lastTag;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints T_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController tController = new ProfiledPIDController(2, 0, 0, T_CONSTRAINTS);

    private static final int TAG_ID = 6;

    // How far we **want** to be from the tag
    private static final Transform3d DESIRED_DISTANCE = new Transform3d(
        new Translation3d(1.5, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, Math.PI)
    );

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
        // Clear targets from previous run
        lastTag = null;

        // Get current pose of the robot on the field.
        Pose2d robotPose = poseProvider.get();

        // Reset the PID controllers to the current robot pose.
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
        tController.reset(robotPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = poseProvider.get();

        // Convert the 2d robot pose to a 3d one.
        Pose3d robotPose3d = new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians())
        );

        // Get the latest results from our limelight.
        PhotonPipelineResult photonResults = camera.getLatestResult();

        // If we see tags
        if(photonResults.hasTargets()) {

            // Filter tags
            Optional<PhotonTrackedTarget> filteredTag = photonResults.getTargets().stream()
                .filter(t -> t.getFiducialId() == TAG_ID) // To find the tag ID we want
                .filter(t -> !t.equals(lastTag)) // That is not in **exactly** the same position as the last one (To prevent recalculations)
                .filter(t -> t.getPoseAmbiguity() <= VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD) // That is not too ambiguous
                .filter(t -> t.getPoseAmbiguity() != -1) // Where something didn't go wrong
                .findFirst();

            // If such a tag exists
            if(filteredTag.isPresent()) {
                // Get the target information
                PhotonTrackedTarget tag = filteredTag.get();

                // Store tag information to compare in the next period
                lastTag = tag;

                // Transform the robot pose by the camera offset to find the exact position of the camera in the field. (Adding the two poses together)
                Pose3d cameraPose = robotPose3d.transformBy(APRILTAG_CAMERA_TO_ROBOT);

                // Get the position of the camera relative to the tag
                Transform3d cameraToTag = tag.getBestCameraToTarget();

                // Transform the camera pose by the distance to the tag to get the exact position of the tag in the field.
                Pose3d tagPose = cameraPose.transformBy(cameraToTag);

                // Transform the tag pose by the desired distance to get the pose we want to drive to.
                Pose2d goalPose = tagPose.transformBy(DESIRED_DISTANCE).toPose2d();

                // Set the PID controller goals.
                xController.setGoal(goalPose.getX());
                yController.setGoal(goalPose.getY());
                tController.setGoal(goalPose.getRotation().getRadians());
            }
        }

        // If we don't see any tags
        if(lastTag == null) {
            // Lock the drive position
            swerveDrive.lockPosition();

            // Otherwise,
        } else {
            // Calculate the needed speeds
            double xSpeed = xController.calculate(robotPose3d.getX());

            if(xController.atGoal()) {
                xSpeed = 0;
            }

            double ySpeed = yController.calculate(robotPose3d.getY());

            if(yController.atGoal()) {
                ySpeed = 0;
            }

            double tSpeed = tController.calculate(robotPose2d.getRotation().getRadians());

            if(tController.atGoal()) {
                tSpeed = 0;
            }

            // Drive the robot using the calculated speeds
            swerveDrive.driveRelative(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, tSpeed, robotPose2d.getRotation())
            );
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
