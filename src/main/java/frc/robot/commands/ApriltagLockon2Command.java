package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ApriltagLockon2Subsystem;
import frc.robot.Subsystems.Constant;
import frc.robot.Subsystems.Constant.ControllerConstants;
import frc.robot.Subsystems.Constant.HelixPIDController;
import frc.robot.Subsystems.DriveTrainSubsystem;

public class ApriltagLockon2Command extends Command {
    private DriveTrainSubsystem drive;
    private ApriltagLockon2Subsystem lockonSubsystem;
    private Joystick hid;
    private int[] tagSubset;
    private Pose2d targetPose;
    private double targetDistance;
    private boolean autonomous;

    private Timer timer = new Timer();
    private HelixPIDController xPid, yPid, tPid;
    private double lastTime = 0;
    private boolean robotCanDrive;

    private final double distanceFixMulti = 4/3;
    private final double distanceFixAdd = 2;

    public ApriltagLockon2Command(
        DriveTrainSubsystem d,
        ApriltagLockon2Subsystem a,
        Joystick j,
        int[] tagsAllowed,
        double targetDistFt
    ) {
        super();
        this.drive = d;
        this.lockonSubsystem = a;
        this.hid = j;
        this.tagSubset = tagsAllowed;
        this.targetPose = new Pose2d(new Translation2d(), new Rotation2d());
        this.targetDistance = targetDistFt * distanceFixMulti + distanceFixAdd;
        this.autonomous = false;
        constructor();
    }

    public ApriltagLockon2Command(
        DriveTrainSubsystem d,
        ApriltagLockon2Subsystem a,
        Joystick j,
        int[] tagsAllowed,
        Pose2d p,
        double targetDistFt
    ) {
        super();
        this.drive = d;
        this.lockonSubsystem = a;
        this.hid = j;
        this.tagSubset = tagsAllowed;
        this.targetPose = p;
        this.targetDistance = targetDistFt * distanceFixMulti + distanceFixAdd;
        this.autonomous = false;
        constructor();
    }

    public ApriltagLockon2Command(
        DriveTrainSubsystem d,
        ApriltagLockon2Subsystem a,
        Joystick j,
        int[] tagsAllowed,
        Pose2d p,
        double targetDistFt,
        boolean autonomous
    ) {
        super();
        this.drive = d;
        this.lockonSubsystem = a;
        this.hid = j;
        this.tagSubset = tagsAllowed;
        this.targetPose = p;
        this.targetDistance = targetDistFt * distanceFixMulti + distanceFixAdd;
        this.autonomous = autonomous;
        constructor();
    }

    private void constructor() {
        timer.reset(); timer.start();

        // SmartDashboard.putNumber("ApriltagLockonXP", 6);
        // SmartDashboard.putNumber("ApriltagLockonTP", 6);
        // SmartDashboard.putNumber("ApriltagLockonTP", 3);

        xPid = new HelixPIDController(1, 0, 0);
        yPid = new HelixPIDController(1, 0, 0);
        tPid = new HelixPIDController(.5, .001, 0);
        tPid.setContinuous(true);
        tPid.setRange(Constant.TwoPI);

        lastTime = 0;
        lockonSubsystem.setDrivePosition(targetDistance, tagSubset);
        robotCanDrive = true;
    }

    // @Override
    // public void initialize() {
    //     super.initialize();
    //     // Pose2d initPose = drive.getPose();
    // }

    @Override
    public void execute() {
        super.execute();

        // xPid.setP(SmartDashboard.getNumber("ApriltagLockonXP", 6));
        // yPid.setP(SmartDashboard.getNumber("ApriltagLockonYP", 6));
        // tPid.setP(SmartDashboard.getNumber("ApriltagLockonTP", 3));

        double time = timer.get();
        double dt = time - lastTime;
        Pose2d currPose = drive.getPose();

        xPid.setReference(-targetPose.getX());
        yPid.setReference(-targetPose.getY());
        tPid.setReference(Math.toRadians(lockonSubsystem.getTheta(-1)));

        double vx = xPid.calculate(currPose.getX(), dt);
        double vy = yPid.calculate(currPose.getY(), dt);
        double omega = -tPid.calculate(drive.getGyroHeading().getRadians(), dt);

        double cap = 1.5;
        int omegacap = 3;
        if(vx > cap) vx = cap;
        else if(vx < -cap) vx = -cap;
        if(vy > cap) vy = cap;
        else if(vy < -cap) vy = -cap;
        if(omega > omegacap) omega = omegacap;
        else if(omega < -omegacap) omega = -omegacap;

        SmartDashboard.putNumber("ApriltagLockonCmd Omgea", omega);
        SmartDashboard.putNumber("ApriltagLockonCmd OmgeaCap", omegacap);

        SmartDashboard.putNumber("AprilTagLockonCmd X", vx);
        SmartDashboard.putNumber("AprilTagLockonCmd Y", vy);
        SmartDashboard.putNumber("AprilTagLockonCmd Omega", omega);
        SmartDashboard.putNumber("AprilTagLockonCmd Id", lockonSubsystem.getFiducialId());

        lastTime = time;
        if(
            Math.pow(currPose.getX(), 2) + Math.pow(currPose.getY(), 2) < 1 &&
            Math.abs(lockonSubsystem.getTheta(-1) - drive.getGyroHeading().getDegrees()) < 3
        ) {
            robotCanDrive = false;
        }

        if (robotCanDrive) {
            drive.drive(new Translation2d(vx, vy), omega);
        } else drive.drive();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        timer.stop();
        drive.brake();
    }

    @Override
    public boolean isFinished() {
        boolean r = false;
        if (autonomous) {
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
