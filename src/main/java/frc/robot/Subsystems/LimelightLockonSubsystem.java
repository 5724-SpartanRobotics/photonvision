package frc.robot.Subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Subsystems.Constant.LockonSubsystem;

public class LimelightLockonSubsystem extends LockonSubsystem {
    private final DriveTrainSubsystem driveTrain;
    private final PIDController pidController;
    // private final Joystick drivestick;

    public PoseEstimate poseEstimate;

    public LimelightLockonSubsystem(
        DriveTrainSubsystem driveTrain
    ) {
        this.driveTrain = driveTrain;
        // this.drivestick = drivestick; // Assign drivestick
        this.pidController = new PIDController(0.005, 0.0, 0.0);
    }

    @Override
    public void periodic() {
        boolean rejectUpdate = false;
        LimelightHelpers.SetRobotOrientation(
            "limelight-threeg", driveTrain.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees(), 
            0, 0, 0, 0, 0
        );
        LimelightHelpers.PoseEstimate mt2;
        if (Robot.isRedAlliance()) mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-threeg");
        else mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-threeg");
        if (Math.abs(driveTrain.getGyroRate()) > 720) rejectUpdate = true;
        if (mt2.tagCount <= 0) rejectUpdate = true;
        if (!rejectUpdate) {
            driveTrain.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            driveTrain.getPoseEstimator().addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            poseEstimate = mt2;
        } else {
            poseEstimate = new PoseEstimate();
        }
    }

    public PoseEstimate getPoseEstimate() {
        return poseEstimate;
    }

    /**
     * @apiNote I don't know if this even works.
     */
    @Override
    public Double getDistance() {
        final double kLLMountAngle_deg = -0F;
        final double kLLLensHeight_in = 15F;
        final double kTargetHeight_in = 2.5F;

        double targetVerticalOffsetAngle_deg = getTheta();
        double targetAngle_rad = Units.degreesToRadians(kLLMountAngle_deg + targetVerticalOffsetAngle_deg);

        return (kTargetHeight_in - kLLLensHeight_in) / Math.tan(targetAngle_rad);
    }

    public double getVerticalTheta() {
        // return -LimelightHelpers.getTX(Constant.AutoConstants.kLimelightName) + driveTrain.getGyroHeading().getDegrees();
        return LimelightHelpers.getTY(Constant.AutoConstants.kLimelightName) + Constant.CameraConstants.Limelight2Plus.MountAngleOffset;
    }

    public Pose2d getT() {
        return new Pose2d(
            LimelightHelpers.getTX(Constant.AutoConstants.kLimelightName),
            LimelightHelpers.getTY(Constant.AutoConstants.kLimelightName),
            new Rotation2d(LimelightHelpers.getTA(Constant.AutoConstants.kLimelightName))
        );
    }

    public int getTargetCount() {
        return LimelightHelpers.getTargetCount(Constant.AutoConstants.kLimelightName);
    }
}
