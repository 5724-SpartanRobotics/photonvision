package frc.robot.commands;

import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
    private final SwerveDrivePoseEstimator poseProvider;

    private final ProfiledPIDController xPidController = new ProfiledPIDController(.05, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yPidController = new ProfiledPIDController(.05, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaPidController = new ProfiledPIDController(.025, 0, 0, OMEGA_CONSTRAINTS);

    private PhotonTrackedTarget lastTarget;

    public AprilTagPhoton(
        PhotonCamera photonCamera,
        DriveTrainSubsystem driveTrainSubsystem,
        SwerveDrivePoseEstimator poseProvider
    ) {
        this.camera = photonCamera;
        this.drive = driveTrainSubsystem;
        this.poseProvider = poseProvider;

        xPidController.setTolerance(0.2);
        yPidController.setTolerance(0.2);
        omegaPidController.setTolerance(Math.toRadians(3));
        omegaPidController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        Pose2d robotPose = poseProvider.getEstimatedPosition();
        omegaPidController.reset(robotPose.getRotation().getRadians());
        xPidController.reset(robotPose.getX());
        yPidController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = poseProvider.getEstimatedPosition();
        Pose3d robotPose = new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians())
        );
        SmartDashboard.putNumber("robot estimated position X", robotPose.getX());
        SmartDashboard.putNumber("robot estimated position Y", robotPose.getY());
        SmartDashboard.putNumber("robot estimated position rot", robotPose.getRotation().getAngle());

        PhotonPipelineResult photonRes = camera.getLatestResult();
        if (photonRes.hasTargets()) {
            List<PhotonTrackedTarget> targetStream = photonRes.getTargets();
            Optional<PhotonTrackedTarget> target = targetStream.stream()
                .filter(t -> {
                    SmartDashboard.putNumber("tagfilter_id_", t.getFiducialId());
                    return t.getFiducialId() != -1;
                })
                .filter(t -> {
                    SmartDashboard.putNumber("tagfilter_ambiguity_" + t.getFiducialId(), t.getPoseAmbiguity());
                    return !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2;  /* && t.getPoseAmbiguity() == -1; */
                })
                .findFirst();

            SmartDashboard.putBoolean("photon res opts", true);
            SmartDashboard.putBoolean("photon res target", target != null);
            
            if (target.isPresent()) {
                lastTarget = target.get();

                SmartDashboard.putNumber("at_robot_pose_X", robotPose.getX());
                SmartDashboard.putNumber("at_robot_pose_Y", robotPose.getY());
                SmartDashboard.putNumber("at_robot_pose_rot", robotPose.getRotation().getAngle());

                Pose3d cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);
                SmartDashboard.putNumber("at_camera_pose_X", cameraPose.getX());
                SmartDashboard.putNumber("at_camera_pose_Y", cameraPose.getY());
                SmartDashboard.putNumber("at_camera_pose_rot", cameraPose.getRotation().getAngle());

                Transform3d camToTarget = target.get().getBestCameraToTarget();
                SmartDashboard.putNumber("at_camtotarget_X", camToTarget.getX());
                SmartDashboard.putNumber("at_camtotarget_Y", camToTarget.getY());
                SmartDashboard.putNumber("at_camtotarget_rot", camToTarget.getRotation().getAngle());
                Pose3d targetPose = cameraPose.transformBy(camToTarget);
                SmartDashboard.putNumber("at_target_pose_X", targetPose.getX());
                SmartDashboard.putNumber("at_target_pose_Y", targetPose.getY());
                SmartDashboard.putNumber("at_target_pose_rot", targetPose.getRotation().getAngle());

                Pose2d goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
                SmartDashboard.putNumber("at_goal_pose_X", goalPose.getX());
                SmartDashboard.putNumber("at_goal_pose_Y", goalPose.getY());
                SmartDashboard.putNumber("at_goal_pose_rot", goalPose.getRotation().getRadians());

                xPidController.setGoal(goalPose.getX());
                yPidController.setGoal(goalPose.getY());
                omegaPidController.setGoal(goalPose.getRotation().getRadians());
            } else {
                lastTarget = null;
                // System.out.println("apriltag has filtered out all of the target.s");
                // System.out.println("tag stream to string? " + targetStream.toString());
            }
        } else {
            lastTarget = null;
            SmartDashboard.putBoolean("photon res opts", false);
        }

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
