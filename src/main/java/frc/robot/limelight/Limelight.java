package frc.robot.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Limelight extends SubsystemBase {
    private final PhotonCamera robotcamera = new PhotonCamera("photonvision"); 




    public Limelight() {
        
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