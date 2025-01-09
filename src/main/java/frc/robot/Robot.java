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
import frc.robot.QFRCLib.ErrorLevel;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.PhotonVisionSubsystem;
import frc.robot.commands.AlignToTargetCommand;
import frc.robot.commands.TeleopSwerve;
public class Robot extends LoggedRobot {

    

public class robot extends TimedRobot {
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
        Logger.recordMetadata("ProjectName", "MyRobotProject");
        Logger.recordMetadata("RobotName", "MyRobot");
    
        // Add log data receivers
        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // USB logging
            Logger.addDataReceiver(new NT4Publisher()); // NetworkTables
        } else {
            String logPath = LogFileUtil.findReplayLog(); // Locate replay log
            Logger.setReplaySource(new WPILOGReader(logPath)); // Replay from log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save new log
        }
    
        // Log Choreo trajectory
        trajectory.ifPresent(t -> {
            // Convert the trajectory to a supported format and log it
            SwerveSample[] samples = t.samples().toArray(new SwerveSample[0]);
            Logger.recordOutput("Choreo/Trajectory", samples);
        });
    
        // Start the logger
        Logger.start();
    }
    
    

@Override
public void robotPeriodic() {
    // Run the CommandScheduler to execute scheduled commands
    CommandScheduler.getInstance().run();

    // Log data for the power distribution panel
    if (powerDistribution != null) {
        SmartDashboard.putNumber("PDP Voltage", powerDistribution.getVoltage());
        SmartDashboard.putNumber("PDP Total Current", powerDistribution.getTotalCurrent());
        SmartDashboard.putNumber("PDP Total Power", powerDistribution.getTotalPower());
        SmartDashboard.putNumber("PDP Temperature", powerDistribution.getTemperature());
    }

    // Log errors or warnings using QFRCLib
    QFRCLib.reportError(ErrorLevel.Critical, "Browned out!");
    QFRCLib.reportError(ErrorLevel.Warning, "Battery Voltage is low");
    QFRCLib.reportError(ErrorLevel.Information, "Robot is ready.");

    // Log swerve module states
    SwerveModuleState[] states = new SwerveModuleState[]{
        new SwerveModuleState(), // Replace with actual states
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };
    Logger.recordOutput("SwerveModuleStates", states);

    // Log pose information
    Pose3d poseA = new Pose3d(); // Replace with actual pose
    Pose3d poseB = new Pose3d(); // Replace with actual pose
    Logger.recordOutput("PoseA", poseA);
    Logger.recordOutput("PoseArray", new Pose3d[]{poseA, poseB});
}


@Override
public void autonomousInit() {
    QFRCLib.setTab("Autonomous");

    // If a trajectory is loaded, get its initial pose
    if (trajectory.isPresent()) {
        Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());
        if (initialPose.isPresent()) {
            drive.resetOdometry(initialPose.get()); // Reset odometry to the start of the trajectory
        }
    }

    // Reset and start the timer
    timer.restart();
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
}