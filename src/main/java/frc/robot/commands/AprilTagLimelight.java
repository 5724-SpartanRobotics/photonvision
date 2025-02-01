package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Subsystems.Constant.ControllerConstants;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.LimelightLockonSubsystem;

public class AprilTagLimelight extends Command {
    private final DriveTrainSubsystem drive;
    private final LimelightLockonSubsystem llLockon;
    private final Joystick hid;

    private boolean rejectUpdate = false;
    private final AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public AprilTagLimelight(
        DriveTrainSubsystem driveTrainSubsystem,
        LimelightLockonSubsystem limelightLockonSubsystem,
        Joystick joystick
    ) {
        super();

        addRequirements(driveTrainSubsystem, limelightLockonSubsystem);
        this.drive = driveTrainSubsystem;
        this.llLockon = limelightLockonSubsystem;
        this.hid = joystick;
    }

    @Override
    public void execute() {
        PoseEstimate mt2 = llLockon.getPoseEstimate();
        Pose2d mt2pose = mt2.pose;
        // Pose2d robotPose = drive.getPose();
        Pose2d targetPose = tagFieldLayout.getTagPose(13).get().toPose2d();
        SmartDashboard.putNumber("apriltag mt2pose X", mt2pose.getX());
        SmartDashboard.putNumber("apriltag mt2pose Y", mt2pose.getY());
        SmartDashboard.putBoolean("apriltag has tag", mt2.tagCount > 0);
        SmartDashboard.putBoolean("apriltag is megatag2", mt2.isMegaTag2);
        if (mt2.tagCount > 0) SmartDashboard.putNumber("apriltag tag id", mt2.rawFiducials[0].id);
        if (mt2.tagCount > 0) drive.drive(targetPose.getTranslation(), -LimelightHelpers.getTX("limelight-threeg"));
        else drive.drive();
    }

        @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drive.brake();
    }

    @Override
    public boolean isFinished() {
        boolean r = false;
        if (RobotState.isAutonomous()) {
            // perform checks to see if other subsystems have done what they need
            return true;
        } else if (
            !hid.getRawButton(ControllerConstants.ButtonMap.TagLockon) &&
            !hid.getRawButton(ControllerConstants.ButtonMap.TagLockonAlt)
        ) {
            r = true;
            // turn off other things because they are done too.
        }
        return r;
    }
}
