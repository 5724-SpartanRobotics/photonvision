package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.LimelightLockonSubsystem;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.Subsystems.ApriltagLockonSubsystem;
import frc.robot.Subsystems.Constant;
import frc.robot.Subsystems.Constant.AutoConstants;
import frc.robot.Subsystems.Constant.HelixPIDController;
import frc.robot.Subsystems.Constant.LockonSubsystem;

public class LimelightAlignToTargetCommand extends Command {
    private DriveTrainSubsystem driveTrain;
    private LimelightLockonSubsystem llLockon;
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
    public LimelightAlignToTargetCommand(
        DriveTrainSubsystem driveTrain,
        LimelightLockonSubsystem llLockon,
        Pose2d targetPose,
        double distance, 
        Joystick driver,
        boolean autonomousDriving
    ) {
        addRequirements(driveTrain);
        this.driveTrain = driveTrain;
        this.llLockon = llLockon;
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
        llLockon.drive(driveTrain, 1);
        robotCanDrive = true;
    }

    @Override
    public void execute() {
        LockonSubsystem.TagReading reading = llLockon.getReading();
        double time = _timer.get();
        double dt = time - lastTime;
        Pose2d currPose = driveTrain.getPose();
        xPID.reference = -targetPose.getX();
        yPID.reference = -targetPose.getY();
        thetaPID.reference = Units.degreesToRadians(reading.angle);

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
            Math.abs(reading.angle - driveTrain.getGyroHeading().getDegrees()) < AutoConstants.degreesError
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
