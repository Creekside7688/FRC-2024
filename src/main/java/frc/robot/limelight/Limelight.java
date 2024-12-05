package frc.robot.limelight;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.net.PortForwarder;
import frc.robot.swerve.SwerveDrive;

import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class Limelight extends SubsystemBase {
    private final PhotonCamera robotcamera = new PhotonCamera("photonvision"); 
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    private final Transform3d cameraToRobot = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotcamera, robotToCam);

// Construct PhotonPoseEstimator


    public Limelight() {
        //PortForwarder.add(5800, "photonvision.local", 5800);
    }
    
    public void getrobotpos() {
        Transform3d cameraToRobot = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
        
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
            
        }
        
    
    
    }

    public void limelightinit() {
        SmartDashboard.putNumber("Target Yaw", 0);
        SmartDashboard.putNumber("Target Pitch", 0);
        SmartDashboard.putNumber("Target FID", 0);
        SmartDashboard.putBoolean("Robot Has targets?", false);
        



    }
    public Pose3d getEstimateAprilTag() {
        
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
            
        } else {
            return null;
        }
    }
    

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public double TargetYaw() {
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        SmartDashboard.putNumber("Target Yaw", target.getYaw());
        return target.getYaw();

    }

    public double TargetPitch() {
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        SmartDashboard.putNumber("Target Pitch", target.getPitch());
        return target.getPitch();
    }

    public int getAprilTag() {
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        SmartDashboard.putNumber("Target FID", target.getFiducialId());
        return target.getFiducialId();
    }

    public boolean CameraHasTargets() {
        var results = robotcamera.getLatestResult();
        SmartDashboard.putBoolean("Robot Has targets?", results.hasTargets());
        return results.hasTargets();
    }

    @Override
    public void periodic() {
    }

}