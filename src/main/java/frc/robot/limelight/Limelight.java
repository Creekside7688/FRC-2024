package frc.robot.limelight;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveDrive;

import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class Limelight extends SubsystemBase {
    private final PhotonCamera robotcamera = new PhotonCamera("photonvision"); 
    



    public Limelight() {
        
    }
    
    public void getrobotpos() {
        Transform3d cameraToRobot = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
}


    }
    public double TargetYaw() {
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        return target.getYaw();
    }

    public double TargetPitch() {
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        return target.getPitch();
    }

    public int getAprilTag() {
        var results = robotcamera.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        return target.getFiducialId();
    }

    public boolean CameraHasTargets() {
        var results = robotcamera.getLatestResult();
        return results.hasTargets();
    }

    @Override
    public void periodic() {
    }

}