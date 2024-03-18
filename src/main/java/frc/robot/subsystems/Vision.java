package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    public PhotonCamera cam;
    public PhotonPoseEstimator poseEstimator;
    public Pose3d robotPose;
    public PhotonTrackedTarget target;
    public AprilTagFieldLayout fieldLayout;
    private double CAMERA_HEIGHT_METERS = 1; // Burayı değiştir

    public Vision() {
        cam = new PhotonCamera("Camera_Module_v1");

        AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = cam.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();

        }
    }

    public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d previousEstimatedPose) {
        if (poseEstimator == null) {
            return null;
        }
        poseEstimator.setReferencePose(previousEstimatedPose);
        return poseEstimator.update();
    }

    public double getDeviationAngle() {
        var result = cam.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if (hasTargets) {
            PhotonTrackedTarget target = result.getBestTarget();
            return target.getYaw();
        } else {
            return 0;
        }
    }

    public double getAprilTagSpeakerDistance() {
        var result = cam.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if (hasTargets) {

            return PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS,
                    1.32,
                    Units.degreesToRadians(180),
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
        } else {
            return 0;
        }
    }

    public double getAprilTagAmphiDistance() {
        var result = cam.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if (hasTargets) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS,
                    1.22,
                    Units.degreesToRadians(180),
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
        } else {
            return 0;
        }
    }

    public double getShooterAngle(){
        return 0; //Burayı unutma Eren
    }
}