package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Constant.LockonSubsystem;

public class LimelightLockonSubsystem extends LockonSubsystem {
    private final DriveTrainSubsystem driveTrain;
    private final PIDController pidController;
    // private final Joystick drivestick;

    private double lastDistance = 0;

    public final class CoralPosition {
        public double distance_m;
        public double rotationOffset_rad;

        public CoralPosition(double distance_m, double rotationOffset_rad) {
            this.distance_m = distance_m;
            this.rotationOffset_rad = rotationOffset_rad;
        }
    }

    public CoralPosition coralPosition;

    public LimelightLockonSubsystem(
        DriveTrainSubsystem driveTrain
    ) {
        this.driveTrain = driveTrain;
        // this.drivestick = drivestick; // Assign drivestick
        this.pidController = new PIDController(0.005, 0.0, 0.0);
    }

    @Override
    public void periodic() {
        super.periodic();
        double currentDistance = Constant.CameraConstants.Limelight2Plus.HeightFromFloor * Math.tan(getVerticalTheta() - Constant.HalfPI);
        double averageDistance = lastDistance * 0.8 + currentDistance * 0.2;


        coralPosition = new CoralPosition(
            averageDistance,
            Units.degreesToRadians(getT().getX())
        );

        SmartDashboard.putNumber("Limelight distance m", coralPosition.distance_m);
        SmartDashboard.putNumber("Limelight rotation offset rad", coralPosition.rotationOffset_rad);

        lastDistance = averageDistance;
    }

    /**
     * @apiNote Find the distance from the object being targeted.
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
