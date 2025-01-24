package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Constant.AutoConstants;

public class VisionSubsystem2024 extends SubsystemBase {
    public static DoubleArraySubscriber AprilTagSpeakerTag;
    private DriveTrainSubsystem swerveDrive;
    private double[] tag;
    private double[] defaultTag = {-1, -1};
    private double lastTheta = 0;

    /**
     * Drive Controller
     * @param swerveDrive The drive train subsystem
     * @param controller A joystick
     */
    public VisionSubsystem2024(DriveTrainSubsystem swerveDrive){
        this.swerveDrive = swerveDrive;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        AprilTagSpeakerTag = inst.getDoubleArrayTopic("/april/speakerTag").subscribe(defaultTag);;
    }

    public void setDrivePosition(double targetDistance, String tagName){
        // TODO better default detection
        // TODO update robot position once the robot slows down    
        
        //The AprilTagSpeakerTag is created in the constructor and is set to the network table entry
        // /april/speakerTag. If we really want different tags based upon the passed in parameter tagName
        // this will need to either change to creating the double array subscriber here, which I fear is 
        // a performance penality, or create multiple tag subscribers in the constructor and change the tagName argument
        // to an enum used to select the one of interest.
        tag = AprilTagSpeakerTag.get(defaultTag);
        if(tag.length > 0 && tag[0] > 0) {
            double theta = -tag[1] + swerveDrive.getGyroHeading().getDegrees() - AutoConstants.cameraAngleOffset; // offsets for non-straight camera
            double deltaD = (tag[0] - targetDistance + AutoConstants.cameraDepthOffset) / 3.28; // Feet to meters // Also the offset is an approximation
            double deltaX = -deltaD * Math.sin(Math.toRadians(theta));
            double deltaY = deltaD * Math.cos(Math.toRadians(theta));
            System.out.println("Target: " + deltaX + ", " + deltaY);
            swerveDrive.ZeroDriveSensors(new Pose2d(new Translation2d(-deltaY, deltaX), swerveDrive.getGyroHeading().times(-1)));
            lastTheta = theta;
        } else {
            swerveDrive.ZeroDriveSensors(new Pose2d(new Translation2d(0, 0), swerveDrive.getGyroHeading().times(-1)));
            lastTheta = swerveDrive.getGyroHeading().getDegrees();
        }
    }
    
    public double getTheta(String tagName) {
        // tag = SmartDashboard.getNumberArray(tagName, defaultTag);
        // double theta = tag[1] + swerveDrive.getGyroHeading().getDegrees();
        // return theta;
        return lastTheta;
    }

    public boolean haveTag(String tagName) {
        if(AprilTagSpeakerTag.get(defaultTag)[0] > 0) {
            return true;
        } else {
            return false;
        }
    }
}