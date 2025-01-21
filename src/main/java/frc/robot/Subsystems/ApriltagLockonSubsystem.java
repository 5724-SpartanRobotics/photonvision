package frc.robot.Subsystems;

import java.net.ProtocolException;
import java.util.HashMap;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.hal.simulation.AnalogInDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Constant.AutoConstants;
import frc.robot.Subsystems.Constant.LockonSubsystem;

public class ApriltagLockonSubsystem extends LockonSubsystem {
    private final DriveTrainSubsystem driveTrain;
    private final PhotonVisionSubsystem vision;
    private final PIDController pidController;
    private final Joystick drivestick;

    public double lastTheta_2024apriltagbase = 0;

    public double VISION_TURN_kP = 0.01F;
    public double VISION_DRIVE_kP = 0.1F;
    public double VISION_DEADBAND = 0.1F;

    public ApriltagLockonSubsystem(
        DriveTrainSubsystem driveTrain,
        PhotonVisionSubsystem vision,
        Joystick drivestick
    ) {
        super();
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.drivestick = drivestick; // Assign drivestick
        this.pidController = new PIDController(0.005, 0.0, 0.0);
    }

    @Override
    public Double getDistance() {
        PhotonTrackedTarget target = vision.getBestResult();
        if (target != null) {
            Transform3d cam2target = target.getBestCameraToTarget();
            if (cam2target != null) {
                double lateral_x = cam2target.getX();
                double vertical_y = cam2target.getY();
                double forward_z = cam2target.getZ();

                // Do some chatgpt maths: Euclidean equation to calculate distance to the tag.
                return Math.sqrt(lateral_x * lateral_x + vertical_y * vertical_y + forward_z * forward_z);
            }
            return Double.NEGATIVE_INFINITY;
        }
        return Double.NEGATIVE_INFINITY;
    }

    @Override
    public double getTheta() {
        PhotonTrackedTarget target = vision.getBestResult();
        if (target != null) return -target.getYaw() + driveTrain.getGyroHeading().getDegrees();
        else return 0F;
    }

    // public double getTheta() { return lastTheta_2024apriltagbase; }
}
