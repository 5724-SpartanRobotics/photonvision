package frc.robot.Subsystems;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;

public class ApriltagLockon {
    private final DriveTrainSubsystem driveTrain;
    private final PhotonVisionSubsystem vision;
    private final PIDController pidController;
    private final Joystick drivestick;

    public double VISION_TURN_kP = 0.01F;
    public double VISION_DRIVE_kP = 0.1F;
    public double VISION_DEADBAND = 0.1F;

    public ApriltagLockon(
        DriveTrainSubsystem driveTrain,
        PhotonVisionSubsystem vision,
        Joystick drivestick
    ) {
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.drivestick = drivestick; // Assign drivestick
        this.pidController = new PIDController(0.02, 0.0, 0.0);
    }

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

    public void drive(double speedMod) {
        this._drive(speedMod);
    }

    public void drive() {
        this._drive(1);
    }
}
