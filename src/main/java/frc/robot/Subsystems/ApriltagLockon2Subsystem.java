package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Constant.AutoConstants;
import frc.robot.Util.Conversions;

public class ApriltagLockon2Subsystem extends SubsystemBase {
    private DriveTrainSubsystem drive;
    private PhotonVisionSubsystem vision;

    private PhotonTrackedTarget bestTarget;
    private double tagDistance = 0;
    private double tagDegrees = 0;
    private int tagFiducial = -1;

    private double lastTheta = 0;

    public ApriltagLockon2Subsystem(DriveTrainSubsystem d, PhotonVisionSubsystem v) {
        super();
        this.drive = d;
        this.vision = v;
    }

    @Override
    public void periodic() {
        super.periodic();
        bestTarget = vision.getBestResult();
        tagDistance = getDistance();
        tagDegrees = getDegrees();
        tagFiducial = getFiducialId();
    }

    private double getDistance() {
        if (bestTarget == null) { return 0; }
        List<TargetCorner> points = bestTarget.getDetectedCorners();
        double topavg = (points.get(3).y + points.get(2).y) / 2;
        double btmavg = (points.get(0).y + points.get(1).y) / 2;
        double diff = topavg - btmavg;
        double diffangle = (diff/Constant.CameraConstants.MicrosoftLifeCamHD3000.Height_px) * Constant.CameraConstants.MicrosoftLifeCamHD3000.vFOV;
        double radians = Math.toRadians(diffangle);
        double goodValue = 6 / Math.tan(radians);
        return Math.abs(goodValue);
    }

    private double getDegrees() {
        if (bestTarget == null) { return 0; }
        List<TargetCorner> points = bestTarget.getDetectedCorners();
        double xavg = ( points.get(0).x + points.get(1).x + points.get(2).x  + points.get(3).x ) / 4;
        return ( xavg / Constant.CameraConstants.MicrosoftLifeCamHD3000.Width_px ) * Constant.CameraConstants.MicrosoftLifeCamHD3000.hFOV - 
            Constant.CameraConstants.MicrosoftLifeCamHD3000.vFOV;
    }

    public int getFiducialId() {
        var res = vision.getBestResult();
        // System.out.println("THis should NOT be nulll! is?" + String.valueOf(res == null));
        return res == null ? -1 : res.getFiducialId();
    }

    private void _setDrivePosition(double targetDistance, Integer[] tagId) {
        int f = getFiducialId();
        double di = getDistance();
        double de = getDegrees();
        ArrayList<Integer> l = new ArrayList<>(Arrays.asList(tagId));
        if (l.contains(f)) {
            double theta = -de + drive.getGyroHeading().getDegrees() - AutoConstants.cameraAngleOffset;
            double dD = Units.feetToMeters(di - targetDistance + AutoConstants.cameraDepthOffset);
            double dX = -dD * Math.sin(Math.toRadians(theta));
            double dY = dD * Math.cos(Math.toRadians(theta));
            SmartDashboard.putNumber("AprilTagTargetXY X", dX);
            SmartDashboard.putNumber("AprilTagTargetXY Y", dY);
            Translation2d t2d = new Translation2d(-dY, dX);
            Pose2d p2d = new Pose2d(t2d, drive.getGyroHeading().times(-1));
            drive.ZeroDriveSensors(p2d);
            lastTheta = theta;
        } else {
            Rotation2d r2d = drive.getGyroHeading();
            drive.ZeroDriveSensors(new Pose2d(new Translation2d(), r2d.times(-1)));
            lastTheta = r2d.getDegrees();
        }
    }

    public void setDrivePosition(double targetDistance, int tagId) {
        _setDrivePosition(targetDistance, new Integer[] {tagId});
    }

    public void setDrivePosition(double targetDistance, int[] tagId) {
        _setDrivePosition(targetDistance, Conversions.intArrayToIntegerArray(tagId));
    }

    public void setDrivePosition(double targetDistance, Integer[] tagId) {
        _setDrivePosition(targetDistance, tagId);
    }

    public double getTheta(int tagId) {
        return lastTheta;
    }

    public boolean haveTag(int tagId) {
        return getFiducialId() > -1;
    }
}
