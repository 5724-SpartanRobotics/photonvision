package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constant {
    public static final double TwoPI = Math.PI * 2;
    public static final double HalfPI = Math.PI / 2;

    public static class HelixPIDController {
        public double kP, kD;
        private double kI;
        private double d, i, lastError;
        public double reference, inputRange;
        public boolean continuous;

        public HelixPIDController(double P, double I, double D) {
            this.kP = P; this.kI = I; this.kD = D;
            i = 0; lastError = 0;
            inputRange = Double.POSITIVE_INFINITY;
            continuous = false;
        }

        public double calculate(double state, double dt) {
            double error = (reference - state) % inputRange;
            if (Math.abs(error) > inputRange / 2) {
                if (error > 0) error -= inputRange;
                else error += inputRange;
            }
            if (dt > 0) {
                d = (error - lastError) / dt;
                i += error *dt;
            }
            lastError = error;
            return kP * error + kI * i + kD * d;
        }
    }

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
     public final class DriveConstants {
        // Maximum robot speed (in meters per second)
        public static final double maxRobotSpeedmps = 4.5;
        public static final double driveGearRatio = 6.75;
        public static double turnGearRatio = 150.0 / 7.0;
        public static final double wheelDiameter = Units.inchesToMeters(4.125);//guessing there is about 1/8" added for the tread. The wheel diameter is 4"
        public static final double wheelCircumfrence = wheelDiameter * Math.PI;//meters
    
        // Maximum angular velocity (in radians per second)
        public static final double maxAngularVelocityRadps = Math.PI * 2;
        public static final double maxLinearSpeed = 15.5F / 3.281F;
    
        // Wheelbase and track width (in meters)
        public static final double wheelBase = 0.6; // Distance between front and back wheels
        public static final double trackWidth = 0.6; // Distance between left and right wheels
    
        // Swerve module offsets (in radians) - adjust these based on calibration
        public static final double LFOff = 0.0;
        public static final double RFOff = 0.0;
        public static final double LBOff = 0.0;
        public static final double RBOff = 0.0; 
    
        // Module positions relative to the robot center (in meters)
        public static final Translation2d LFLocation = new Translation2d(wheelBase / 2.0, trackWidth / 2.0); // Front left
        public static final Translation2d RFLocation = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0); // Front right
        public static final Translation2d LBLocation = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0); // Back left
        public static final Translation2d RBLocation = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0); // Back right
    
        // PID Controller tuning values for trajectory tracking
        public static final double xPID = 10.0;
        public static final double yPID = 10.0;
        public static final double headingPID = 7.5;
    }
    public static final class ControllerConstants{
        public static double joystickDeadband = 0.1;//a deadband that you must overcome for the joystick input, otherwise we send 0
        public static double joystickZDeadband = 0.3;//a deadband that you must overcome for the joystick input, otherwise we send 0

        public static final class ButtonMap {
            public static final int GyroZero = 7;
            public static final int TagLockon = 1;
            public static final int TagLockonAlt = 11;
            public static final int ObjectLockon = 2;
            public static final int ObjectLockonAlt = 12;
            public static final int OuttakePiece = -1;
        }
    }
    public static final class DebugSetting{
        //set this to get more values to smart dashboard. Leave it off for competition.
        public static final DebugLevel TraceLevel = DebugLevel.Swerve;
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
