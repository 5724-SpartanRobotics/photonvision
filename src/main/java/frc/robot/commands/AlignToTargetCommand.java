package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
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
    private final Joystick drivestick;

    private final double VISION_TURN_kP = 0.01F;

    public AlignToTargetCommand(
    DriveTrainSubsystem driveTrain,
    PhotonVisionSubsystem vision,
    Joystick drivestick
) {
    this.driveTrain = driveTrain;
    this.vision = vision;
    this.drivestick = drivestick; // Assign drivestick
    this.pidController = new PIDController(0.02, 0.0, 0.0);

    addRequirements(driveTrain, vision);
}


    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        // Calculate drivetrain commands from Joystick values
        double forward = -drivestick.getRawAxis(1) * Constant.DriveConstants.maxRobotSpeedmps; // Axis 1 for forward/backward
        double strafe = -drivestick.getRawAxis(0) * Constant.DriveConstants.maxRobotSpeedmps;  // Axis 0 for left/right
        double turn = -drivestick.getRawAxis(2) * Constant.DriveConstants.maxAngularVelocityRadps; // Axis 2 for rotation

        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        if (vision.hasTarget()) {
            PhotonTrackedTarget target = vision.getBestTarget();

            // At least one target is detected by the camera
            if (target != null) {
                targetYaw = target.getYaw();
                targetVisible = true;
            }
        }

        // Auto-align when requested
        if (targetVisible && drivestick.getRawButton(4)) { // Button 1 for alignment
            // Override the driver's turn command with an automatic one that turns toward the tag
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
