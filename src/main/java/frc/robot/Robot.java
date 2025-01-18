package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.auto.AutoFactory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.QFRCLib.ErrorLevel;
import frc.robot.Subsystems.ApriltagLockon;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.PhotonVisionSubsystem;
import frc.robot.Subsystems.Constant.ControllerConstants;
import frc.robot.commands.ApriltagAlignToTargetCommand;
import frc.robot.commands.LimelightAlignToTargetCommand;
import frc.robot.commands.TeleopSwerve;

public class Robot extends LoggedRobot {

    private DriveTrainSubsystem drive;
    private PhotonVisionSubsystem vision;
    private Joystick drivestick = new Joystick(0); // Replaced XboxController with Joystick
    private PowerDistribution powerDistribution;
    private Field2d field = new Field2d();
    private Timer timer = new Timer();

    private ApriltagLockon apriltagLockon;

    // Choreo trajectory support
    private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("myTrajectory");

    @Override
    public void robotInit() {
        drive = new DriveTrainSubsystem();
        vision = new PhotonVisionSubsystem("Front");
        apriltagLockon = new ApriltagLockon(drive, vision, drivestick);
        PortForwarder.add(5800, "photonvision.local", 5800);

        // Pass DriveTrainSubsystem and Joystick to TeleopSwerve
        drive.setDefaultCommand(new TeleopSwerve(drive, drivestick));

        // Initialize power distribution
        powerDistribution = new PowerDistribution(0, ModuleType.kCTRE);

        SmartDashboard.putData("Field", field);

        SmartDashboard.putNumber("joystickDeadband", 0.1);
        SmartDashboard.putNumber("joystickZDeadband", 0.5);

        // Logger setup
        Logger.recordMetadata("ProjectName", "MyRobotProject");
        Logger.recordMetadata("RobotName", "MyRobot");

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }

        trajectory.ifPresent(t -> Logger.recordOutput("Choreo/Trajectory", t.samples().toArray(new SwerveSample[0])));
        Logger.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (powerDistribution != null) {
            SmartDashboard.putNumber("PDP Voltage", powerDistribution.getVoltage());
            SmartDashboard.putNumber("PDP Total Current", powerDistribution.getTotalCurrent());
            SmartDashboard.putNumber("PDP Total Power", powerDistribution.getTotalPower());
            SmartDashboard.putNumber("PDP Temperature", powerDistribution.getTemperature());
        }

        QFRCLib.reportError(ErrorLevel.Critical, "Browned out!");
        QFRCLib.reportError(ErrorLevel.Warning, "Battery Voltage is low");
        QFRCLib.reportError(ErrorLevel.Information, "Robot is ready.");

        SwerveModuleState[] states = new SwerveModuleState[]{
                new SwerveModuleState(), 
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };
        Logger.recordOutput("SwerveModuleStates", states);

        Pose3d poseA = new Pose3d(); 
        Pose3d poseB = new Pose3d(); 
        Logger.recordOutput("PoseA", poseA);
        Logger.recordOutput("PoseArray", new Pose3d[]{poseA, poseB});
    }

    @Override
    public void autonomousInit() {
        // Reset and start the timer for auto mode
        timer.reset();
        timer.start();

        // Reset odometry if trajectory is loaded
        trajectory.ifPresent(t -> {
            Optional<Pose2d> initialPose = t.getInitialPose(isRedAlliance());
            initialPose.ifPresent(pose -> drive.resetOdometry(pose));
        });
    }

    @Override
    public void teleopPeriodic() {
        // Trigger alignment to target
        if (drivestick.getRawButton(ControllerConstants.ButtonMap.TagLockon) || drivestick.getRawButton(11)) {
            apriltagLockon.drive();
        }

        if (drivestick.getRawButton(ControllerConstants.ButtonMap.ObjectLockon)) {
            new LimelightAlignToTargetCommand(drive, drivestick).schedule();
        }

        if (drivestick.getRawButton(ControllerConstants.ButtonMap.GyroZero)) { 
            drive.setGyroZero();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }  
}
