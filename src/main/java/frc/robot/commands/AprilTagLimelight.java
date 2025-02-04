package frc.robot.commands;

import java.util.List;

import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Subsystems.Constant;
import frc.robot.Subsystems.Constant.AutoConstants;
import frc.robot.Subsystems.Constant.ControllerConstants;
import frc.robot.Subsystems.Constant.DriveConstants;
import frc.robot.Subsystems.Constant.HelixPIDController;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.LimelightLockonSubsystem;

public class AprilTagLimelight extends Command {
    private final DriveTrainSubsystem drive;
    private final LimelightLockonSubsystem llLockon;
    private final Joystick hid;

    private final HelixPIDController xPid, yPid, thetaPid;
    private Timer _timer = new Timer();
    private double lastTime = 0;

    private boolean rejectUpdate = false;
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

        this.xPid = new HelixPIDController(AutoConstants.kPAutoShoot, 0, 0);
        this.yPid = new HelixPIDController(AutoConstants.kPAutoShoot, 0, 0);
        this.thetaPid = new HelixPIDController(AutoConstants.kPTurnAutoShoot, 0, 0);
        this.thetaPid.setContinuous(true);
        this.thetaPid.setRange(Constant.TwoPI);

        _timer.reset();
        _timer.start();
    }

    @Override
    public void execute() {
        super.execute();

        PoseEstimate mt2 = llLockon.getPoseEstimate();

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
            xPid.setReference(targetPose.getX()); // Target pose: 0,0
            yPid.setReference(targetPose.getY()); // Target pose: 0,0
            thetaPid.setReference(theta);

            double Vx = xPid.calculate(currentPose.getX(), deltaTime);
            double Vy = yPid.calculate(currentPose.getY(), deltaTime);
            double angularSpeed = -thetaPid.calculate(drive.getGyroHeading().getRadians(), deltaTime);

            double cap = 1.5;
            int omegacap = 3;
            if(Vx > cap) Vx = cap;
            else if(Vx < -cap) Vx = -cap;
            if(Vy > cap) Vy = cap;
            else if(Vy < -cap) Vy = -cap;
            if(angularSpeed > omegacap) angularSpeed = omegacap;
            else if(angularSpeed < -omegacap) angularSpeed = -omegacap;

            lastTime = time;
            // if (
            //     Math.pow(currentPose.getX(), 2) + Math.pow(currentPose.getY(), 2) < targetDistance &&
            //     Math.abs(theta) - drive.getGyroHeading().getRadians() < Math.toRadians(AutoConstants.degreesError)
            // ) drive.drive(new Translation2d(Vx, Vy), angularSpeed);
            // else drive.drive();
            drive.drive(new Translation2d(Vx, Vy), angularSpeed);
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
