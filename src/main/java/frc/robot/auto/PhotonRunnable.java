package frc.robot.auto;

import static frc.robot.constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT;
import static frc.robot.constants.VisionConstants.FIELD_LENGTH_METERS;
import static frc.robot.constants.VisionConstants.FIELD_WIDTH_METERS;
import java.util.concurrent.atomic.AtomicReference;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PhotonRunnable implements Runnable {

    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera photonCamera;
    

    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();


    public PhotonRunnable(PhotonCamera camera) {
        this.photonCamera = camera;

        PhotonPoseEstimator photonPoseEstimator = null;

        try {
            AprilTagFieldLayout layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

            // PV estimates will always be blue, they'll get flipped by robot thread
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            if(photonCamera != null) {
                photonPoseEstimator = new PhotonPoseEstimator(
                    layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, APRILTAG_CAMERA_TO_ROBOT.inverse());
            }
        } catch(Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }

        this.photonPoseEstimator = photonPoseEstimator;
    }

    @Override
    public void run() {
        // Get AprilTag data
        if(photonPoseEstimator != null && photonCamera != null && !RobotState.isAutonomous()) {
            PhotonPipelineResult photonResults = photonCamera.getLatestResult();

            if(photonResults.hasTargets() && (photonResults.targets.size() > 1
                || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {

                photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {

                    Pose3d estimatedPose = estimatedRobotPose.estimatedPose;

                    // Make sure the measurement is on the field
                    if(estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
                        && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
                        atomicEstimatedRobotPose.set(estimatedRobotPose);
                    }
                });
            }
        }
    }

    /**
     * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is a new estimate that hasn't been returned before. This pose
     * will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
     * 
     * @return latest estimated pose
     */
    public EstimatedRobotPose grabLatestEstimatedPose() {
        return atomicEstimatedRobotPose.getAndSet(null);
    }


    public PhotonPipelineResult getLatestResult() {
        return photonCamera.getLatestResult();
    }

}