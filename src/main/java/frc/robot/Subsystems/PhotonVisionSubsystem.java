package frc.robot.Subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;

    public PhotonVisionSubsystem(String cameraName) {
        camera = new PhotonCamera(cameraName); 
    }

    @SuppressWarnings("removal")
    public boolean hasTarget() {
        return camera.getLatestResult().hasTargets();
    }

    public double getYaw() {
        @SuppressWarnings("removal")
        PhotonPipelineResult result = camera.getLatestResult();
        return result.hasTargets() ? result.getBestTarget().getYaw() : 0.0;
    }

    public void setPipeline(int pipelineIndex) {
        camera.setPipelineIndex(pipelineIndex); // Switch pipelines dynamically
        camera.setDriverMode(true);
    }
 @SuppressWarnings("removal")
@Override
    public void periodic() {
        // Update whether a target is visible on the SmartDashboard
        SmartDashboard.putNumber("PhotonVision/Target Yaw", getYaw());
        SmartDashboard.putBoolean("PhotonVision/Has Target", camera.getLatestResult().hasTargets());
  
     }

}