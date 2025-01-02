package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constant {
    public static final class CanIdConstants{
        public static final int LFTurnMotor = 2;//falcon500
        public static final int LFDriveMotor = 3;//falcon500
        public static final int LFCanID = 10;
        public static final int LBTurnMotor = 4;//falcon500
        public static final int LBDriveMotor = 5;//falcon500
        public static final int LBCanID = 11;
        public static final int RBTurnMotor = 6;//falcon500
        public static final int RBDriveMotor = 7;//falcon500
        public static final int RBCanID = 13;
        public static final int RFTurnMotor = 8;//falcon500
        public static final int RFDriveMotor = 9;//falcon500
        public static final int RFCanID = 12;
        public static final int PigeonID = 14;
        public static final int PDHID = 0;
        public static final int pneumaticontroller = 15;
     }
    public static final class DriveConstants {
        public static final double LFOff = 2.36;// 0.375 * 2 * pi  CANCoder offset in radians
        public static final double RFOff = 2.33;//0.371 * 2 * pi
        public static final double LBOff = 4.499;//-1.784; //-0.284 * 2 * pi  (adding 2 pi to make it not negative.)
        public static final double RBOff = 0.182;//0.029 * 2 * pi
        public static final double trackWidth = Units.inchesToMeters(26.75);//wheel center to center width
        public static final double wheelBase = Units.inchesToMeters(22.25);//wheel center to center front / back distance
        public static final double wheelDiameter = Units.inchesToMeters(4.125);//guessing there is about 1/8" added for the tread. The wheel diameter is 4"
        public static final double wheelCircumfrence = wheelDiameter * Math.PI;//meters
        public static final double driveGearRatio = 6.75;
        public static final double maxMotorRpm = 6380;
        public static final double maxWheelRpm = maxMotorRpm / driveGearRatio;
        public static final double maxRobotSpeedmps = maxWheelRpm / 60 * wheelCircumfrence;//should be 5.1853 mps
        //Swerve locations relative to the center of the robot. Positive x values represent moving toward the front of the robot whereas positive y values
        // represent moving toward the left of the robot. Distances are in meters.
        public static Translation2d LFLocation = new Translation2d(wheelBase/2, trackWidth/2);
        public static Translation2d RFLocation = new Translation2d(wheelBase/2, -trackWidth/2);
        public static Translation2d LBLocation = new Translation2d(-wheelBase/2, trackWidth/2);
        public static Translation2d RBLocation = new Translation2d(-wheelBase/2, -trackWidth/2);
        public static double turnGearRatio = 150.0 / 7.0;
        /**Maximum angular velocity in degrees per second */
        public static double maxAngularVelocityRps = 10.0;

        public static final double speakerDistance = 6.75;
        public static final double kNominalVoltage = 0;
        public static final int kDriveCurrentLimit = 0;
        public static final int kSteerCurrentLimit = 0;
    }
    public static final class ControllerConstants{
        public static double joystickDeadband = 0.1;//a deadband that you must overcome for the joystick input, otherwise we send 0
    }
    public static final class DebugSetting{
        //set this to get more values to smart dashboard. Leave it off for competition.
        public static final DebugLevel TraceLevel = DebugLevel.Off;
    }
    public static enum DebugLevel{
        Off,
        Swerve,
        Amp,
        Climber,
        IntakeShooter,
        Helix,
        Vision,
        All
    }
    public static final class AutoConstants {
        public static final double kPTranslationController = 2;
        public static final double kPThetaController = 3;
        public static final double autoShootCloseness = 0.003;
        public static final double degreesError = 5;
        public static final double cameraAngleOffset = 5.5;
        public static final double cameraDepthOffset = 1;
        public static final double kPAutoShoot = 6;
        public static final double kPTurnAutoShoot = 12;
    }
}
