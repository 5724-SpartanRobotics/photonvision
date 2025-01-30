package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Constant.LockonSubsystem;

public class LimelightLockonSubsystem extends LockonSubsystem {
    private final DriveTrainSubsystem driveTrain;
    private final PIDController pidController;
    // private final Joystick drivestick;

    public LimelightLockonSubsystem(
        DriveTrainSubsystem driveTrain
    ) {
        this.driveTrain = driveTrain;
        // this.drivestick = drivestick; // Assign drivestick
        this.pidController = new PIDController(0.005, 0.0, 0.0);
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

    @Override
    public double getTheta() {
        return -LimelightHelpers.getTX(Constant.AutoConstants.kLimelightName) + driveTrain.getGyroHeading().getDegrees();
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
