package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Subsystems.Constant;
import frc.robot.Subsystems.Constant.ControllerConstants;
import frc.robot.Subsystems.Constant.HelixPIDController;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.LimelightLockonSubsystem;

public class AprilTagLimelight extends Command {
    private final DriveTrainSubsystem drive;
    private final LimelightLockonSubsystem llLockon;
    private final Joystick hid;

    private final HelixPIDController xPid, yPid, thetaPid, thetaPid2;
    private Timer _timer = new Timer();
    private double lastTime = 0;

    private final AprilTagFieldLayout tagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public AprilTagLimelight(
        DriveTrainSubsystem driveTrainSubsystem,
        LimelightLockonSubsystem limelightLockonSubsystem,
        Joystick joystick
    ) {
        super();

        addRequirements(driveTrainSubsystem, limelightLockonSubsystem);
        this.drive = driveTrainSubsystem;
        this.llLockon = limelightLockonSubsystem;
        this.hid = joystick;

        final double pidGain = 6.;

        this.xPid = new HelixPIDController(pidGain, 0, 1);
        this.yPid = new HelixPIDController(pidGain, 0, 1);
        this.thetaPid = new HelixPIDController(2*pidGain, 0, 1);
        this.thetaPid.setContinuous(true);
        this.thetaPid.setRange(Constant.TwoPI);
        this.thetaPid2 = new HelixPIDController(2*pidGain, 0, 1);
        this.thetaPid2.setContinuous(true);
        this.thetaPid2.setRange(Constant.TwoPI);

        _timer.reset();
        _timer.start();
    }

    @Override
    public void execute() {
        super.execute();

        PoseEstimate mt2 = llLockon.getPoseEstimate();
        Pose3d anotherPose = llLockon.getPoseRobotSpace();

        final double targetDistance = 3; //feet 
        // Pose2d T = llLockon.getT();
        // double kP_proportionalAim = 0.035;
        // double proportionalAim = T.getX() * kP_proportionalAim * DriveConstants.maxAngularVelocityRadps * -1.0;
        // double kP_proportionalRange = 0.1;
        // double proportionalRange = T.getY() * kP_proportionalRange * DriveConstants.maxRobotSpeedmps * -1.0;
        
        if (mt2.tagCount > 0) {
            double dist = mt2.rawFiducials[0].distToCamera;
            double angle = Math.toRadians(mt2.rawFiducials[0].txnc);
            double time = _timer.get();

            double deltaTime = time - lastTime;
            double theta = -angle + drive.getGyroHeading().getRadians();
            double deltaD = dist - targetDistance;
            double deltaX = -deltaD * Math.sin(theta);
            double deltaY = deltaD * Math.cos(theta);

            drive.ZeroDriveSensors(new Pose2d(
                new Translation2d(-deltaY, deltaX),
                drive.getGyroHeading().times(-1)
            ));

            Pose2d currentPose = drive.getPose();
            Pose2d targetPose = tagFieldLayout.getTagPose(mt2.rawFiducials[0].id).get().toPose2d();
            SmartDashboard.putNumber("target pose x", targetPose.getX());
            SmartDashboard.putNumber("target pose y", targetPose.getY());
            SmartDashboard.putNumber("target pose rotation", targetPose.getRotation().getDegrees());

            xPid.setReference(targetPose.getX()); // Target pose: 0,0
            yPid.setReference(targetPose.getY()); // Target pose: 0,0
            thetaPid.setReference(theta);
            thetaPid2.setReference(-mt2.rawFiducials[0].txnc);

            double Vx = xPid.calculate(currentPose.getX(), deltaTime);
            double Vy = yPid.calculate(currentPose.getY(), deltaTime);
            double angularSpeed = -thetaPid.calculate(drive.getGyroHeading().getRadians(), deltaTime);
                angularSpeed *= Math.toRadians(angle);
            SmartDashboard.putNumber("apriltag angular speed", angularSpeed);

            double cap = 1.5;
            int omegacap = 3;
            if(Vx > cap) Vx = cap;
            else if(Vx < -cap) Vx = -cap;
            if(Vy > cap) Vy = cap;
            else if(Vy < -cap) Vy = -cap;
            if(angularSpeed > omegacap) angularSpeed = omegacap;
            else if(angularSpeed < -omegacap) angularSpeed = -omegacap;

            angularSpeed *= -1.;

            lastTime = time;
            // if (
            //     Math.pow(currentPose.getX(), 2) + Math.pow(currentPose.getY(), 2) < targetDistance &&
            //     Math.abs(theta) - drive.getGyroHeading().getRadians() < Math.toRadians(AutoConstants.degreesError)
            // ) drive.drive(new Translation2d(Vx, Vy), angularSpeed);
            // else drive.drive();
            // drive.drive(new Translation2d(Vx, Vy), angularSpeed);
            drive.drive(anotherPose);
        } else drive.drive();

        // SmartDashboard.putNumber("apriltag tag id",
        //     mt2.tagCount > 0 ? mt2.rawFiducials[0].id : -1
        // );

        // double joystickY = -hid.getY();

        // SmartDashboard.putNumber("apriltag aim", proportionalAim);
        // SmartDashboard.putNumber("apriltag range", proportionalRange);
        // SmartDashboard.putNumber("joystick y", joystickY);
        
        
        // if (mt2.tagCount > 0) {
        //     SmartDashboard.putBoolean("apriltag candrive", true);
        //     drive.setFieldRelative(false) // ensure field relative driving (it should already be, though)
        //         .drive(new Translation2d(proportionalAim, joystickY), proportionalRange)
        //         .setFieldRelative(DriveTrainSubsystem.kFieldRelativeDefault);
        // } else {
        //     SmartDashboard.putBoolean("apriltag candrive", false);
        //     drive.drive();
        // }
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
