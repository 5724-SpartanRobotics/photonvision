package frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ApriltagLockon2Subsystem;
import frc.robot.Subsystems.Constant.ControllerConstants;
import frc.robot.Subsystems.DriveTrainSubsystem;

/**
 * {@link}https://github.com/lasarobotics/PH2024/blob/master/src/main/java/frc/robot/subsystems/vision/AprilTagCamera.java
 */
public class ApriltagToPoseCommand extends Command {
    private final DriveTrainSubsystem drive;
    private final ApriltagLockon2Subsystem atLockon;
    private final Joystick hid;
    private double distance;
    private Integer[] tagSubset;
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout field;

    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    private final Transform3d transform = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));

    private EstimatedRobotPose finalEstPose = null;
    private Vector<N3> finalVector = null;

    public ApriltagToPoseCommand(
        DriveTrainSubsystem drivetrainSubsystem,
        ApriltagLockon2Subsystem apriltagLockonSubsystem,
        Joystick joystick,
        Optional<Integer[]> allowedTags,
        double targetDistance
    ) {
        super();

        addRequirements(drivetrainSubsystem, apriltagLockonSubsystem);
        joystick.getName(); // Check for null (should throw and exception if it is)

        this.drive = drivetrainSubsystem;
        this.atLockon = apriltagLockonSubsystem;
        this.hid = joystick;
        this.distance = targetDistance;

        this.tagSubset = allowedTags != null ? allowedTags.get() : new Integer[] {};
        
        this.field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        this.poseEstimator = new PhotonPoseEstimator(
            this.field,
            PoseStrategy.LOWEST_AMBIGUITY,
            transform
        );
    }

    @Override
    public void execute() {
        final double MAX_POSE_AMBIGUITY = 0.1;
        final double MAX_POSE_HEIGHT = 0.75;
        final Distance SINGLE2MULTI_dTAG_POSE = Units.Meters.of(edu.wpi.first.math.util.Units.feetToMeters(distance));
        super.execute();

        PhotonPipelineResult res = atLockon.getVisionSubsystem().getCamera().getLatestResult();
        
        if (res == null || !res.hasTargets()) return;
        if (res.targets.size() == 1 && res.targets.get(0).getPoseAmbiguity() > MAX_POSE_AMBIGUITY) return;

        poseEstimator.update(res).ifPresent(estPose -> {
            Pose3d pose = estPose.estimatedPose;
            if (
                pose.getX() < 0.0 || pose.getX() > field.getFieldLength() ||
                pose.getY() < 0.0 || pose.getY() > field.getFieldWidth() ||
                pose.getZ() < 0.0 || pose.getZ() > MAX_POSE_HEIGHT
            ) return;

            var closestDist = Units.Meters.of(100.0);
            for (PhotonTrackedTarget target : estPose.targetsUsed) {
                AprilTag tag = field.getTags().stream().filter(t -> t.ID == target.getFiducialId()).findFirst().get();
                Distance dist = Units.Meters.of(target.getBestCameraToTarget().getTranslation().getNorm());
                var singleTargetPose = tag.pose.transformBy(target.getBestCameraToTarget().inverse()).transformBy(transform.inverse());
                if (pose.relativeTo(singleTargetPose).getTranslation().getNorm() > SINGLE2MULTI_dTAG_POSE.in(Units.Meters)) return;
                if (dist.lte(closestDist)) closestDist = dist;
            }

            if (closestDist.gte(Units.Meters.of(5.0))) return;
            double xyStdDev = 0.01 * Math.pow(closestDist.in(Units.Meters), 2.0) / estPose.targetsUsed.size();
            double thetaStdDev = RobotState.isDisabled() ? xyStdDev : Double.MAX_VALUE;
            
            this.finalEstPose = estPose;
            this.finalVector = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
        });

        if (finalEstPose != null && finalVector != null) {
            drive.drive(finalEstPose.estimatedPose.getTranslation().toTranslation2d(), finalEstPose.estimatedPose.getRotation().toRotation2d().getRadians());
        } else {drive.drive();}
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
