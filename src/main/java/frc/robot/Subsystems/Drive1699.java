package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.Constant.DriveConstants;
// import frc.team1699.Constants.SwerveConstants;
// import frc.team1699.Constants.VisionConstants;
// import frc.team1699.lib.vision.VisionData;
// import frc.team1699.lib.vision.VisionHandler;
// import swervelib.SwerveDrive;
// import swervelib.parser.SwerveParser;
// import swervelib.telemetry.SwerveDriveTelemetry;
// import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * {@link}https://github.com/FIRST-Team-1699/Robocats2024/blob/main/src/main/java/frc/team1699/subsystems/Drive.java
 */
public class Drive1699 {
//    private DriveState currentState = DriveState.LOCK;
//    private DriveState wantedState = DriveState.LOCK;

    private DriveState currentState = DriveState.TELEOP_DRIVE;
    private DriveState wantedState = DriveState.TELEOP_DRIVE;
    private Timer trajTimer = new Timer();
    // private PathPlannerTrajectory trajectory;
    private DriveTrainSubsystem driveController;
    // private PIDConstants translationConstants = new PIDConstants(0.01);
    // private PIDConstants rotationConstants = new PIDConstants(0.2);
    private boolean doneWithTraj = true;
    private PIDController headingLockController = new PIDController(.015, .005, 0);
    private PIDController headingAmpController = new PIDController(.025, 0.01, 0);

    private DriveTrainSubsystem swerve;
    private Joystick controller;
    // private VisionHandler visionHandler;

    private static double kDeadband = .02;

    public Drive1699(DriveTrainSubsystem dts, Joystick controller) {
        this.swerve = dts;
        this.controller = controller;
        this.driveController = dts;
        // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
        // visionHandler = new VisionHandler(VisionConstants.kCameraName, VisionConstants.kCameraPosition, VisionConstants.kAprilTagField);
    }

    private void teleopDrive() {
        // get controller inputs
       // double vX = -controller.getLeftY();
      //  double vY = controller.getLeftX();
        double vX = -controller.getRawAxis(0);
        double vY = -controller.getRawAxis(1);
        double vR = -controller.getRawAxis(2);
        // apply deadbands
        if(Math.abs(vX) < kDeadband) {
            vX = 0;
        }
        if(Math.abs(vY) < kDeadband) {
            vY = 0;
        }
        if(Math.abs(vR) < kDeadband) {
            vR = 0;
        }
        // scale outputs
        vX *= DriveConstants.maxRobotSpeedmps;
        vY *= DriveConstants.maxRobotSpeedmps;
        vR *= DriveConstants.maxAngularVelocityRadps;


        // drive swerve
        swerve.drive(new Translation2d(vY, vX), vR);
    }

    /*private void teleopDriveHeadingPID(double targetOffset) {
        // get controller inputs
        double vX = -controller.getLeftY();
        double vY = controller.getLeftX();
        double vR = headingLockController.calculate(targetOffset, 0.0);
        System.out.println(vR); 
        //System.out.println(swerve.getOdometryHeading());
        System.out.println("X:  " + vX + "   vY:   " + vY);

        // drive swerve
        swerve.drive(new Translation2d(vY, vX), vR, true, false);

    } */

     private void teleopDriveHeadingPID(double targetOffset) {
        // get controller inputs
        System.out.println("BAD IF I AM SAWED");
        double vX = controller.getRawAxis(0);
        double vY = controller.getRawAxis(1);
        double vR = headingLockController.calculate(targetOffset, 0.0);

        // apply deadbands
        if(Math.abs(vX) < kDeadband) {
            vX = 0;
        }
        if(Math.abs(vY) < kDeadband) {
            vY = 0;
        }
        // scale outputs
        vX *= DriveConstants.maxRobotSpeedmps; 
        vY *= DriveConstants.maxRobotSpeedmps;
        vR *= DriveConstants.maxAngularVelocityRadps;
        // drive swerve
        swerve.drive(new Translation2d(vX, vY), vR);
    }

   /* private void teleopDriveHeadingAmp() {
        // get controller inputs
        double vX = -controller.getLeftY();
        double vY = controller.getLeftX();
        double vR = -headingAmpController.calculate(getHeading().getDegrees(), 90); */

     private void teleopDriveHeadingAmp() {
        // get controller inputs
                System.out.println("BAD IF I AM SAWED AMP");

        double vX = controller.getRawAxis(0);
        double vY = controller.getRawAxis(1);
        double vR = headingAmpController.calculate(getHeading().getDegrees(), 90);

        System.out.println(getHeading().getDegrees());
        // apply deadbands
        if(Math.abs(vX) < kDeadband) {
            vX = 0;
        }
        if(Math.abs(vY) < kDeadband) {
            vY = 0;
        }
        // scale outputs
        vX *= DriveConstants.maxRobotSpeedmps; 
        vY *= DriveConstants.maxRobotSpeedmps;
        vR *= DriveConstants.maxAngularVelocityRadps;
        // drive swerve
        swerve.drive(new Translation2d(vX, vY), vR);
    } 

     public void setTrajectory(Object trajectory) {
        // this.trajectory = trajectory;
    }

    public void setHeading(double heading) {
        // swerve.addVisionMeasurement(getPose(), heading);
    }

    public void resetHeading() {
        Rotation2d gyroReading = swerve.getGyroHeading();
        Pose2d pose = new Pose2d(new Translation2d(), gyroReading);
        swerve.ZeroDriveSensors(pose);
    } 


    // /** Manually set the module states
    //  * @param moduleStates
    //  * FL, FR, BL, BR
    //  */
    // private void setModuleStates(SwerveModuleState[] moduleStates) {
    //     swerve.setModuleStates(moduleStates, false);
    // }

    /** Set an X to keep the swerve from moving */
    //photonvision-heading
   /* private void lock() {
        swerve.lockPose();
    }

    public void update()  */
     private void lock() {
        // swerve.lockPose();
    }

    // private void driveTraj() {
    //     if(trajTimer.get() < trajectory.getTotalTimeSeconds()) {
    //         PathPlannerTrajectory.State targetState = trajectory.sample(trajTimer.get());
    //         ChassisSpeeds targetSpeeds = driveController.calculateRobotRelativeSpeeds(swerve.getPose(), targetState);
    //         swerve.drive(targetSpeeds);
    //     } else {
    //         trajTimer.stop();
    //         doneWithTraj = true;
    //         setWantedState(DriveState.LOCK);
    //     }
    // }

    private void updateVisionData() {

        // VisionData estimatedData = visionHandler.getEstimatedPose();
        // if(estimatedData.getTimestamp() != -1.0) {
        //     Pose2d estimatedPose = estimatedData.getPose2d();
        //     swerve.addVisionMeasurement(estimatedPose, estimatedData.getTimestamp());
        //     Rotation2d estimatedRotation = estimatedPose.getRotation();
        //     swerve.setGyroOffset(new Rotation3d(0, 0, -estimatedRotation.getRadians()));
        // }

      /*  switch (currentState) {
            case FOLLOW_TRAJ:
                if(trajTimer.get() < trajectory.getTotalTimeSeconds()) {
                    PathPlannerTrajectory.State targetState = trajectory.sample(trajTimer.get());
                    ChassisSpeeds targetSpeeds = driveController.calculateRobotRelativeSpeeds(swerve.getPose(), targetState);
                    swerve.drive(targetSpeeds);
                } else {
                    trajTimer.stop();
                    doneWithTraj = true;
                    setWantedState(DriveState.LOCK);
                } */

    } 

    public void update() {
        // teleopDrive();
        updateVisionData();
        switch (currentState) {
            case FOLLOW_TRAJ:
                // driveTraj();
                break;
            case LOCK:
                // lock();
                break;
            case TELEOP_DRIVE:
                teleopDrive();
                break;
            case TELEOP_APRILTAG_TRACK:
                // if(visionHandler.getTargetID() == 6 || visionHandler.getTargetID() == -1) {
                //     teleopDriveHeadingAmp();
                // } else if(visionHandler.getTargetID() != -1) {
                //     teleopDriveHeadingPID(visionHandler.getTargetOffset());
                // }
                break;
            default:
                break;
        }
    }

    private void handleStateTransition() {
        switch (wantedState) {
            case FOLLOW_TRAJ:
                // trajTimer.reset();
                // trajTimer.start();
                // doneWithTraj = false;

             //   swerve.resetOdometry(new Pose2d(trajectory.sample(0).positionMeters, swerve.getYaw()));

                // swerve.resetOdometry(new Pose2d(trajectory.sample(0).positionMeters, swerve.getOdometryHeading()));

                break;
            case LOCK:
                break;
            case TELEOP_DRIVE:
                break;
            case TELEOP_APRILTAG_TRACK:
                break;
            default:
                break;
        }
        currentState = wantedState;
    }

    public void setWantedState(DriveState state) {
        if(this.wantedState != state) {
            wantedState = state;
            handleStateTransition();
        }
    }

    public DriveState getState() {
        return this.currentState;
    }

    /** For auto
     * @param states
     * FL, FR, BL, BR
     */
    public void setModuleStates(SwerveModuleState[] states) {
        swerve.drive(states);
    }

    public Pose2d getPose() {
        return swerve.getPose();
    }

    public Rotation2d getHeading() {
      //  return swerve.getYaw();
        return swerve.getGyroHeading();

    }

    public boolean doneWithTraj() {
        return doneWithTraj;
    }

    public enum DriveState {
        TELEOP_DRIVE,
        TELEOP_APRILTAG_TRACK,
        LOCK,
        FOLLOW_TRAJ
    }
}