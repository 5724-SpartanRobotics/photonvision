package frc.robot.Subsystems;

import java.net.ProtocolException;
import java.util.HashMap;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Constant.AutoConstants;

public class ApriltagLockonSubsystem extends SubsystemBase {
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
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.drivestick = drivestick; // Assign drivestick
        this.pidController = new PIDController(0.005, 0.0, 0.0);
    }

    /**
     * @deprecated Does not work.
     * @param speedMod
     */
    protected void _drive(double speedMod) {
        pidController.reset();

        // Calculate drivetrain commands from Joystick values
        double forward = -drivestick.getRawAxis(1) * Constant.DriveConstants.maxRobotSpeedmps; // Axis 1 for forward/backward
        double strafe = -drivestick.getRawAxis(0) * Constant.DriveConstants.maxRobotSpeedmps;  // Axis 0 for left/right
        double turn = -drivestick.getRawAxis(2) * Constant.DriveConstants.maxAngularVelocityRadps; // Axis 2 for rotation

        PhotonTrackedTarget bestTarget = vision.getBestResult();

        if (bestTarget != null) {
            // Get the yaw (horizontal angle offset) and distance to the target
            double yaw = bestTarget.getYaw();
            double area = bestTarget.getArea();

            // Rotation correction
            double rotationSpeed = VISION_TURN_kP * yaw;
            if (Math.abs(rotationSpeed) < VISION_DEADBAND) {
                rotationSpeed = 0;
            }

            // Driving correction based on target area (distance)
            double driveSpeed = VISION_DRIVE_kP * (1.0 - area); // Adjust based on desired area
            if (Math.abs(driveSpeed) < VISION_DEADBAND) {
                driveSpeed = 0;
            }

            double finalRotation;
            if (rotationSpeed == 0) {
                finalRotation = 0F;
            } else {
                finalRotation = turn/rotationSpeed;
            }
            
            Translation2d driveTranslation = new Translation2d(forward, strafe);
            driveTrain.drive(driveTranslation, finalRotation);
        }
    }

    /**
     * Advance to the target linearally.
     * @param speedMod default: 1
     */
    public void _drive_2024pixybase(double speedMod) {
        PhotonTrackedTarget target = vision.getBestResult();
        if (target != null) {
            double yaw = target.getYaw();
            double gyroAngle = driveTrain.getGyroHeading().getRadians();
            double forward_x = Math.sin(gyroAngle) * (speedMod * 2);
            double forward_y = Math.cos(gyroAngle) * (speedMod * 2);
            double strafe = -(speedMod * 2) * yaw * pidController.getP();
            double strafe_x = Math.sin(yaw + Constant.HalfPI) * strafe;
            double strafe_y = Math.cos(yaw + Constant.HalfPI) * strafe;
            Translation2d translation = new Translation2d(forward_y + strafe_y, forward_x + strafe_x);
            driveTrain.drive(translation, 0);
        }
    }

    public void _drive_2024apriltagbase(double speedMod) {
        double targetDistance = 0; // PARAM
        String tagName = "36h11"; // PARAM

        PhotonTrackedTarget target = vision.getBestResult();
        double distance_ft = 0;
        if (target != null) {
            Transform3d cam2target = target.getBestCameraToTarget();
            if (cam2target != null) {
                double lateral_x = cam2target.getX();
                double vertical_y = cam2target.getY();
                double forward_z = cam2target.getZ();

                // Do some chatgpt maths: Euclidean equation to calculate distance to the tag.
                distance_ft = Math.sqrt(lateral_x * lateral_x + vertical_y * vertical_y + forward_z * forward_z);
            }

            double theta = -target.getYaw() + driveTrain.getGyroHeading().getDegrees() - /* AutoConstants.cameraAngleOffset */ 0;
            double delta_d_m = Units.feetToMeters(distance_ft - targetDistance + /* AutoConstants.cameraDepthOffset */ 0);
            double delta_x = -delta_d_m * Math.sin(Units.degreesToRadians(theta));
            double delta_y = delta_d_m * Math.cos(Units.degreesToRadians(theta));
            Translation2d dTranslation = new Translation2d(-delta_y, delta_x);
            Pose2d pose = new Pose2d(dTranslation, driveTrain.getGyroHeading().times(-1F));
            driveTrain.ZeroDriveSensors(pose);
            lastTheta_2024apriltagbase = theta;
        } else {
            Translation2d dTranslation = new Translation2d(0, 0);
            Pose2d pose = new Pose2d(dTranslation, driveTrain.getGyroHeading().times(-1F));
            driveTrain.ZeroDriveSensors(pose);
            lastTheta_2024apriltagbase = driveTrain.getGyroHeading().getDegrees();
        }
    }

    public double getTheta() { return lastTheta_2024apriltagbase; }

    public void drive(double speedMod) {
        this._drive(speedMod);
    }

    public void drive() {
        this._drive(1);
    }
}
