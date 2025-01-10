package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.PhotonVisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Subsystems.Constant;

import java.util.List;

public class AlignToTargetCommand extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final PhotonVisionSubsystem vision;
    private final PIDController pidController;
    private final XboxController controller;

    private final double VISION_TURN_kP = 0.01F;

    public AlignToTargetCommand(
        DriveTrainSubsystem driveTrain,
        PhotonVisionSubsystem vision,
        XboxController controller
    ) {
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.pidController = new PIDController(0.02, 0.0, 0.0); // Tune PID gains as needed
        this.controller = controller; // Tune PID gains as needed

        addRequirements(driveTrain, vision);
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    // @Override
    public void execute_bad() {
        if (vision.hasTarget()) {
            // Retrieve the best target
            PhotonTrackedTarget bestTarget = vision.getBestTarget();
            double yawError;

            if (bestTarget != null) {
                yawError = bestTarget.getYaw(); // Align to the best target
            } else {
                // Optional: Align to the average yaw of all targets
                List<PhotonTrackedTarget> targets = vision.getAllTargets();
                yawError = targets.stream()
                                  .mapToDouble(PhotonTrackedTarget::getYaw)
                                  .average()
                                  .orElse(0.0); // Default to 0.0 if no targets
            }

            double turnSpeed = pidController.calculate(yawError, 0.0); // Align to 0 yaw error

            // Drive the robot with calculated turning speed
            driveTrain.drive(new Translation2d(0.0, 0.0), turnSpeed);
        } else {
            // Stop if no target is detected
            driveTrain.drive(new Translation2d(0.0, 0.0), 0.0);
        }
    }

    @Override
    public void execute() {
        // Calculate drivetrain commands from Joystick values
        double forward = -controller.getLeftY() * Constant.DriveConstants.maxRobotSpeedmps;
        double strafe = -controller.getLeftX() * Constant.DriveConstants.maxRobotSpeedmps;
        double turn = -controller.getRightX() * Constant.DriveConstants.maxAngularVelocityRadps;

        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        if (vision.hasTarget()) {
            PhotonTrackedTarget target = vision.getBestTarget();
            double yawError;

            // At least one AprilTag was seen by the camera
            if (target.getFiducialId() == 7) {
                    // Found Tag 7, record its information
                    targetYaw = target.getYaw();
                    targetVisible = true;
                }
            }
        
        // Auto-align when requested
        if (targetVisible) {
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn command with an automatic one that turns toward the tag.
            turn = -1.0 * targetYaw * VISION_TURN_kP * Constant.DriveConstants.maxAngularVelocityRadps;
        }

        // Command drivetrain motors based on target speeds
        Translation2d translation = new Translation2d(forward, strafe);
        driveTrain.drive(translation, turn);
    }

    @Override
    public boolean isFinished() {
        // Finish when aligned within 1 degree
        PhotonTrackedTarget bestTarget = vision.getBestTarget();
        return bestTarget != null && Math.abs(bestTarget.getYaw()) < 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        driveTrain.drive(new Translation2d(0.0, 0.0), 0.0);
    }
}
