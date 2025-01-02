package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.PhotonVisionSubsystem;

public class AlignToTargetCommand extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final PhotonVisionSubsystem vision;
    private final PIDController pidController;

    public AlignToTargetCommand(DriveTrainSubsystem driveTrain, PhotonVisionSubsystem vision) {
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.pidController = new PIDController(0.02, 0.0, 0.0); // Tune PID gains as needed

        addRequirements(driveTrain, vision);
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double yawError = vision.getYaw(); // Get target yaw error
            double turnSpeed = pidController.calculate(yawError, 0); // Align to 0 yaw error
           


            // Use drive() with a stationary Translation2d (no X/Y movement)
            driveTrain.drive(new Translation2d(0.0, 0.0), turnSpeed);
        } else {
            // Stop if no target is detected
            driveTrain.drive(new Translation2d(0.0, 0.0), 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        // Finish when aligned within 1 degree
        return vision.hasTarget() && Math.abs(vision.getYaw()) < 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        driveTrain.drive(new Translation2d(0.0, 0.0), 0.0);
    }
}