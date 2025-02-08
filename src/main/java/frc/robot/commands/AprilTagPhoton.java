package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Constant.DriveConstants;
import frc.robot.Subsystems.DriveTrainSubsystem;

public class AprilTagPhoton extends Command {
    private static final Distance DISTANCE_FROM_TARGET = Units.Meters.of(1.5);

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(DriveConstants.maxRobotSpeedmps, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(DriveConstants.maxRobotSpeedmps, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private static final Transform3d TAG_TO_GOAL = new Transform3d(
        new Translation3d(DISTANCE_FROM_TARGET.baseUnitMagnitude(), 0, 0),
        new Rotation3d(0, 0, Math.PI)
    );

    private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(Units.Inches.of(12).in(Units.Meters), 0, Units.Inches.of(19).in(Units.Meters)),
        new Rotation3d(0, Units.Radians.of(Math.PI / 6.5).baseUnitMagnitude(), 0)
    );

    private final PhotonCamera camera;
    private final DriveTrainSubsystem drive;
    private final Pose2d poseProvider;

    private final ProfiledPIDController xPidController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yPidController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaPidController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

    private PhotonTrackedTarget lastTarget;

    public AprilTagPhoton(
        PhotonCamera photonCamera,
        DriveTrainSubsystem driveTrainSubsystem,
        Pose2d pose
    ) {
        this.camera = photonCamera;
        this.drive = driveTrainSubsystem;
        this.poseProvider = pose;

        xPidController.setTolerance(0.2);
        yPidController.setTolerance(0.2);
        omegaPidController.setTolerance(Math.toRadians(3));
        omegaPidController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        Pose2d robotPose = poseProvider;
        omegaPidController.reset(robotPose.getRotation().getRadians());
        xPidController.reset(robotPose.getX());
        xPidController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = poseProvider;
        Pose3d robotPose = new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians())
        );

        List<PhotonPipelineResult> photonRes = camera.getAllUnreadResults();
        if (!photonRes.isEmpty()) {
            PhotonTrackedTarget target = photonRes.get(0).getBestTarget();

            SmartDashboard.putBoolean("photon res opts", true);
            SmartDashboard.putBoolean("photon res target", target != null);
            
            if (target != null) {
                lastTarget = target;
                Pose3d cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d targetPose = cameraPose.transformBy(camToTarget);
                Pose2d goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                xPidController.setGoal(goalPose.getX());
                yPidController.setGoal(goalPose.getY());
                omegaPidController.setGoal(goalPose.getRotation().getRadians());
            }
        } else SmartDashboard.putBoolean("photon res opts", false);

        if (lastTarget == null) {
            SmartDashboard.putBoolean("photon res targets", false);
            drive.drive();
        } else {
            SmartDashboard.putBoolean("photon res targets", true);

            double xSpeed = xPidController.calculate(robotPose.getX());
            if (xPidController.atGoal()) xSpeed = 0;

            double ySpeed = yPidController.calculate(robotPose.getY());
            if (yPidController.atGoal()) ySpeed = 0;

            double wSpeed = omegaPidController.calculate(robotPose2d.getRotation().getRadians());
            if (omegaPidController.atGoal()) wSpeed = 0;

            drive.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, wSpeed, robotPose2d.getRotation()
                )
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive();
    }
}
