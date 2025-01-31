package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Constant;
import frc.robot.Subsystems.Constant.HelixPIDController;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.LimelightLockonSubsystem;

public class LimelightAlignToTargetCommand extends Command {
    private DriveTrainSubsystem driveTrain;
    private LimelightLockonSubsystem llLockon;
    private Pose2d targetPose;
    private double targetDistance;
    private Joystick hid;
    private boolean autoDrive;

    // private Pose2d initPose;
    private Timer _timer = new Timer();
    private HelixPIDController xPID, yPID, thetaPID;
    private double lastTime = 0;
    private boolean robotCanDrive;
    // private boolean isAuto;

    // TODO: Find out the units of distance (meters/ft/etc)
    public LimelightAlignToTargetCommand(
        DriveTrainSubsystem driveTrain,
        LimelightLockonSubsystem llLockon,
        Pose2d targetPose,
        double distance, 
        Joystick hid,
        boolean autonomousDriving
    ) {
        super();
        addRequirements(driveTrain, llLockon);
        this.driveTrain = driveTrain;
        this.llLockon = llLockon;
        this.targetPose = targetPose;
        this.targetDistance = distance;
        this.hid = hid;
        this.autoDrive = autonomousDriving;
        constructor();
    }

    public LimelightAlignToTargetCommand(
        DriveTrainSubsystem driveTrain,
        LimelightLockonSubsystem llLockon,
        Joystick driver,
        double distance,
        boolean autonomousDriving
    ) {
        super();
        addRequirements(driveTrain, llLockon);
        this.driveTrain = driveTrain;
        this.llLockon = llLockon;
        this.targetPose = new Pose2d();
        this.targetDistance = distance;
        this.autoDrive = autonomousDriving;
        constructor();
    }

    private void constructor() {
        // initPose = driveTrain.getPose();
        _timer.reset();
        _timer.start();

        xPID = new HelixPIDController(1, 0, 0);
        yPID = new HelixPIDController(1, 0, 0);
        thetaPID = new HelixPIDController(0.5, 0, 0);
        thetaPID.setContinuous(true);
        thetaPID.setRange(Constant.TwoPI);

        lastTime = 0;
        llLockon.drive(driveTrain, 1D);
        robotCanDrive = true;
    }

    @Override
    public void execute() {
        super.execute();
        Pose2d reading = llLockon.getT();
        double time = _timer.get();
        double dt = time - lastTime;
        Pose2d currPose = driveTrain.getPose();
        xPID.setReference(-currPose.getX());
        yPID.setReference(-currPose.getY());
        thetaPID.setReference(currPose.getRotation().getRadians());

        double vx = xPID.calculate(targetPose.getX(), dt);
        double vy = yPID.calculate(targetPose.getY(), dt);
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
        // if (
        //     // Math.pow(currPose.getX(), 2) + Math.pow(currPose.getY(), 2) < 1 &&
        //     // Math.abs(reading.getX() /* - driveTrain.getGyroHeading().getDegrees() */) < 10 &&
        //     reading.getY() <= -16
        // ) {
        //     // Intake, do another thing, etc.
        //     robotCanDrive = false;
        // }

        if (Constant.isClose(reading.getY(), -12F, 3F)) robotCanDrive = false;
        if (Constant.isClose(reading.getRotation().getRadians(), 15, 5)) robotCanDrive = false;

        SmartDashboard.putNumber("LimelightLockonCmd Count", llLockon.getTargetCount());
        // if (reading.getX() == 0F && reading.getY() == 0F) robotCanDrive = false;

        // if (Math.abs(reading.getX()) < 24) {
        //     double mv = (-reading.getX() + reading.getY()) / 2F;
        //     // vx *= mv * Constant.signum(reading.getX());
        //     vx *= mv;
        //     // vy *= Constant.signum(reading.getX());
        // }

        SmartDashboard.putNumber("LimelightLockonCmd Omgea", omega);
        SmartDashboard.putNumber("LimelightLockonCmd OmgeaCap", omegacap);

        SmartDashboard.putNumber("LimelightLockonCmd X", vx);
        SmartDashboard.putNumber("LimelightLockonCmd Y", vy);
        SmartDashboard.putNumber("LimelightLockonCmd Omega", omega);
        SmartDashboard.putBoolean("LimelightLockonCmd CanDrive", robotCanDrive);

        if (robotCanDrive) driveTrain.drive(new Translation2d(vx, vy), -reading.getX());
        else driveTrain.drive();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _timer.stop();
        driveTrain.brake();
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        boolean retVal = false;
        if (autoDrive) {
            // 2024... time checks on the shooter (see if 2s had elapsed)
            return true;
        } else if (llLockon.getT().getY() < -16) {
            // This is when the piece is about 1ft from the bumper with 
            return true;
        } else if (hid != null && !hid.getRawButton(Constant.ControllerConstants.ButtonMap.OuttakePiece)) {
            retVal = true;
            // 2024... turn off the shooter.
        }
        return retVal;
    }
}
