package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveTrainSubsystem;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.Subsystems.Constant;

public class LimelightAlignToTargetCommand extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final PIDController pidController;
    private final Joystick drivestick;

    public String limelightName = "Gamepiecedetection";

    private final double VISION_TURN_kP = 0.01F;

    public LimelightAlignToTargetCommand(
    DriveTrainSubsystem driveTrain,
    Joystick drivestick
) {
    this.driveTrain = driveTrain;
    this.drivestick = drivestick; // Assign drivestick
    this.pidController = new PIDController(0.02, 0.0, 0.0);

    addRequirements(driveTrain);
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
        boolean useTarget = false;
        double tx = 0F, ty = 0F, ta = 0F;
        if (LimelightHelpers.getTargetCount(limelightName) > 0) {
            LimelightResults results = LimelightHelpers.getLatestResults(limelightName);

            // At least one target is detected by the camera
            if (results != null) {
                tx = LimelightHelpers.getTX(limelightName);
                ty = LimelightHelpers.getTY(limelightName);
                ta = LimelightHelpers.getTA(limelightName);
                useTarget = true;
            }
        }

        // Auto-align when requested
        if (useTarget && drivestick.getRawButton(2)) { // Button 1 for alignment
            // Override the driver's turn command with an automatic one that turns toward the tag
            turn = -1.0 * ta * VISION_TURN_kP * Constant.DriveConstants.maxAngularVelocityRadps;
        }

        // Command drivetrain motors based on target speeds
        Translation2d translation = new Translation2d(forward, strafe);
        Translation2d _xyTranslation = new Translation2d(tx, ty);
        driveTrain.drive(translation, turn);
    }

    @Override
    public boolean isFinished() {
        // Finish when aligned within 1 degree
        LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        int restultCount = LimelightHelpers.getTargetCount(limelightName);
        return results != null && restultCount > 0D && Math.abs(LimelightHelpers.getTA(limelightName)) < 1D;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        driveTrain.drive(new Translation2d(0.0, 0.0), 0.0);
    }
}
