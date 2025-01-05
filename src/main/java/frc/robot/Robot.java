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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.PhotonVisionSubsystem;
import frc.robot.commands.AlignToTargetCommand;
import frc.robot.commands.TeleopSwerve;

public class Robot extends TimedRobot {
    private DriveTrainSubsystem drive;
    private PhotonVisionSubsystem vision;
    private XboxController drivestick = new XboxController(0);
    private PowerDistribution powerDistribution;
    private Field2d field = new Field2d();
    private Timer timer = new Timer();

    // Choreo trajectory support
    private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("myTrajectory");

    @Override
    public void robotInit() {
        // Initialize subsystems
        drive = new DriveTrainSubsystem();
        vision = new PhotonVisionSubsystem("limelight");

        // Set default teleop command
        drive.setDefaultCommand(new TeleopSwerve(drive, drivestick));

        // Initialize power distribution
        powerDistribution = new PowerDistribution(0, ModuleType.kCTRE);

        // Add Field2d to SmartDashboard
        SmartDashboard.putData("Field", field);

        // Logger setup for AdvantageScope
        Logger.getInstance().recordMetadata("ProjectName", "MyRobotProject");
        Logger.getInstance().recordMetadata("RobotName", "MyRobot");

        // Add log data receivers
        if (isReal()) {
            Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // USB logging
            Logger.getInstance().addDataReceiver(new NT4Publisher()); // Live data via NetworkTables
        } else {
            setUseTiming(false); // Run as fast as possible in sim
            String logPath = LogFileUtil.findReplayLog(); // Locate replay log
            Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Replay from log
            Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save new log
        }

        // Start the logger
        Logger.getInstance().start();

        // Choreo trajectory logging
        trajectory.ifPresent(t -> Logger.getInstance().recordOutput("Choreo/Trajectory", t));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Publish power distribution data to SmartDashboard
        SmartDashboard.putNumber("PDP Voltage", powerDistribution.getVoltage());
        SmartDashboard.putNumber("PDP Total Current", powerDistribution.getTotalCurrent());
        SmartDashboard.putNumber("PDP Total Power", powerDistribution.getTotalPower());
        SmartDashboard.putNumber("PDP Temperature", powerDistribution.getTemperature());

        // Log robot states for AdvantageScope
        Pose2d robotPose = drive.getPose(); // Retrieve current robot pose
        SwerveModuleState[] moduleStates = drive.getModuleStates(); // Retrieve swerve module states

        Logger.getInstance().recordOutput("Field Pose", new Pose3d(robotPose, drive.getRotation()));
        Logger.getInstance().recordOutput("Swerve Module States", moduleStates);

        // Log vision data
        SmartDashboard.putBoolean("Vision/Has Target", vision.hasTarget());
        SmartDashboard.putNumber("Vision/Target Yaw", vision.getYaw());
        Logger.getInstance().recordOutput("Vision/Has Target", vision.hasTarget());
        Logger.getInstance().recordOutput("Vision/Target Yaw", vision.getYaw());
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
        if (drivestick.getAButtonPressed()) {
            new AlignToTargetCommand(drive, vision).schedule();
        }

        // Toggle vision camera driver mode
        if (drivestick.getLeftBumperPressed()) {
            vision.setPipeline(0); // Switch to driver mode pipeline
        } else if (drivestick.getLeftBumperReleased()) {
            vision.setPipeline(1); // Switch back to target pipeline
        }
    }
    private boolean isRedAlliance() {
      return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
  }  
}
