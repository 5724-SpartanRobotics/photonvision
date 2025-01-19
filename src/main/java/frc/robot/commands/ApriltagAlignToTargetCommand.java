package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.PhotonVisionSubsystem;
import frc.robot.Subsystems.Constant.AutoConstants;

import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Subsystems.ApriltagLockonSubsystem;
import frc.robot.Subsystems.Constant;
import frc.robot.Subsystems.Constant.HelixPIDController;

import java.util.List;

public class ApriltagAlignToTargetCommand extends Command {
    private DriveTrainSubsystem driveTrain;
    private ApriltagLockonSubsystem tagLockon;
    private Pose2d targetPose;
    private double targetDistance;
    private Joystick driver;
    private boolean autoDrive;

    private Pose2d initPose;
    private Timer _timer = new Timer();
    private HelixPIDController xPID, yPID, thetaPID;
    private double lastTime = 0;
    private boolean robotCanDrive;
    private boolean isAuto;

    // TODO: Find out the units of distance (meters/ft/etc)
    public ApriltagAlignToTargetCommand(
        DriveTrainSubsystem driveTrain,
        ApriltagLockonSubsystem tagLockon,
        Pose2d targetPose,
        double distance, 
        Joystick driver,
        boolean autonomousDriving
    ) {
        addRequirements(driveTrain);
        this.driveTrain = driveTrain;
        this.tagLockon = tagLockon;
        this.targetPose = targetPose;
        this.targetDistance = distance;
        this.driver = driver;
        this.autoDrive = autonomousDriving;
    }

    @Override
    public void initialize() {
        initPose = driveTrain.getPose();
        _timer.reset();
        _timer.start();

        xPID = new HelixPIDController(AutoConstants.kPAutoShoot, 0, 0);
        yPID = new HelixPIDController(AutoConstants.kPAutoShoot, 0, 0);

        thetaPID = new HelixPIDController(AutoConstants.kPAutoShoot, 0, 0);
        thetaPID.continuous = true;
        thetaPID.inputRange = Constant.TwoPI;

        lastTime = 0;
        tagLockon._drive_2024apriltagbase(targetDistance);
        robotCanDrive = true;
    }

    @Override
    public void execute() {
        double time = _timer.get();
        double dt = time - lastTime;
        Pose2d currPose = driveTrain.getPose();
        xPID.reference = -targetPose.getX();
        yPID.reference = -targetPose.getY();
        thetaPID.reference = Units.degreesToRadians(tagLockon.getTheta());

        double vx = xPID.calculate(currPose.getX(), dt);
        double vy = yPID.calculate(currPose.getY(), dt);
        double omega = -thetaPID.calculate(driveTrain.getGyroHeading().getRadians(), dt);

        double cap = 1.5;
        int omegacap = 3;
        if(vx > cap) vx = cap;
        else if(vx < -cap) vx = -cap;
        if(vy > cap) vy = cap;
        else if(vy < -cap) vy = -cap;
        if(omega > omegacap) omega = omegacap;
        else if(omega < -omegacap) omega = -omegacap;

        lastTime = time;
        if (
            Math.pow(currPose.getX(), 2) + Math.pow(currPose.getY(), 2) < AutoConstants.autoShootCloseness &&
            Math.abs(tagLockon.getTheta() - driveTrain.getGyroHeading().getDegrees()) < AutoConstants.degreesError
        ) {
            // Intake, do another thing, etc.
            robotCanDrive = false;
        }

        if (robotCanDrive) driveTrain.drive(new Translation2d(vx, vy), omega);
        else driveTrain.drive();
    }

    @Override
    public void end(boolean interrupted) {
        _timer.stop();
        driveTrain.brake();
    }

    @Override
    public boolean isFinished() {
        boolean retVal = false;
        if (isAuto) {
            // 2024... time checks on the shooter (see if 2s had elapsed)
            return true;
        } else if (!driver.getRawButton(Constant.ControllerConstants.ButtonMap.OuttakePiece)) {
            retVal = true;
            // 2024... turn off the shooter.
        }
        return retVal;
    }
}

/*
public class ApriltagAlignToTargetCommand extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final PhotonVisionSubsystem vision;
    private final PIDController pidController;
    private final Joystick drivestick;

    private final double VISION_TURN_kP = 0.01F;

    public ApriltagAlignToTargetCommand(
    DriveTrainSubsystem driveTrain,
    PhotonVisionSubsystem vision,
    Joystick drivestick
) {
    this.driveTrain = driveTrain;
    this.vision = vision;
    this.drivestick = drivestick; // Assign drivestick
    this.pidController = new PIDController(0.02, 0.0, 0.0);

    addRequirements(driveTrain, vision);
}


    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {
        // Calculate drivetrain commands from Joystick values
        double forward = -drivestick.getRawAxis(1) * Constant.DriveConstants.maxRobotSpeedmps; // Axis 1 for forward/backward
        double strafe = -drivestick.getRawAxis(0) * Constant.DriveConstants.maxRobotSpeedmps;  // Axis 0 for left/right
        double turn = -drivestick.getRawAxis(2) * Constant.DriveConstants.maxAngularVelocityRadps; // Axis 2 for rotation

        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        if (vision.hasTarget()) {
            PhotonTrackedTarget target = vision.getBestTarget();

            // At least one target is detected by the camera
            if (target != null) {
                targetYaw = target.getYaw();
                targetVisible = true;
            }
        }

        // Auto-align when requested
        if (targetVisible) { 
            // Override the driver's turn command with an automatic one that turns toward the tag
            turn = -1.0 * targetYaw * VISION_TURN_kP * Constant.DriveConstants.maxAngularVelocityRadps;
        }

        // Command drivetrain motors based on target speeds
        Translation2d translation = new Translation2d(forward, strafe);
        driveTrain.drive(translation, turn);
    }

    @Override
    public boolean isFinished() {
        // Finish when aligned within 1 degree
        PhotonTrackedTarget bestTarget = vision.getBestTarget();
        return bestTarget != null && Math.abs(bestTarget.getYaw()) < 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        driveTrain.drive(new Translation2d(0.0, 0.0), 0.0);
    }
}
*/
