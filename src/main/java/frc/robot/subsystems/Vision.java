package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    public PhotonCamera cam = new PhotonCamera("Camera_Module_v1");

    public Vision() {

    }

    public void getDeviationAngle() {
        var result = cam.getLatestResult();
        boolean hasTargets = result.hasTargets();
        if(hasTargets){
            List<PhotonTrackedTarget> targets = result.getTargets();
            
        }
    }
}