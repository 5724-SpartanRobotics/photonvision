package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.PhotonCamera;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.auto.AutoFactory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
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




public class Robot extends TimedRobot {
  private DriveTrainSubsystem drive;
  private PhotonVisionSubsystem vision;
  private XboxController drivestick = new XboxController(0);
  private PowerDistribution powerDistribution;
  private QFRCLib dashboard;
  private NetworkTable table; 
  private NetworkTableEntry streamEntry;
  private PhotonCamera camera;
  private final Field2d m_field = new Field2d();
  private final Timer timer = new Timer();
  private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("myTrajectory");
 

 public Robot(){
 PortForwarder.add(5800, "photonvision.local", 5800);

 // Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
 //if (isReal()) {
    //Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    //Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
   // new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
 // } else {
    //setUseTiming(false); // Run as fast as possible
    //String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    //Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
   // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
 }
  //Logger.start();
 


  @Override
  public void robotInit() {
    drive = new DriveTrainSubsystem();
    vision = new PhotonVisionSubsystem("limelight"); // Replace "CameraName" with the PhotonVision camera's name
    drive.setDefaultCommand(new TeleopSwerve(drive, drivestick));
    powerDistribution = new PowerDistribution(0, ModuleType.kCTRE);
    dashboard = new QFRCLib();
    SmartDashboard.putData("Field", m_field);
    Pose3d poseA = new Pose3d();
    Pose3d poseB = new Pose3d();



  }
  

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (powerDistribution != null) { // Ensure the object is initialized
      // Send general PDP data to the SmartDashboard
      SmartDashboard.putNumber("PDP Voltage", powerDistribution.getVoltage());
      SmartDashboard.putNumber("PDP Total Current", powerDistribution.getTotalCurrent());
      SmartDashboard.putNumber("PDP Total Power", powerDistribution.getTotalPower());
      SmartDashboard.putNumber("PDP Temperature", powerDistribution.getTemperature());
      QFRCLib.reportError(ErrorLevel.Critical, "Browned out!");
      QFRCLib.reportError(ErrorLevel.Warning, "Battery Voltage is low");
      QFRCLib.reportError(ErrorLevel.Information, "Robot is ready.");
    }
    SwerveModuleState[] states = new SwerveModuleState[] {
   new SwerveModuleState(),
   new SwerveModuleState(),
   new SwerveModuleState(),
    new SwerveModuleState()
    };
    Pose3d poseA = new Pose3d();
    Pose3d poseB = new Pose3d();
    Logger.recordOutput("MyStates", states);
    Logger.recordOutput("MyPose", poseA);
    Logger.recordOutput("MyPoseArray", poseA, poseB);
    Logger.recordOutput("MyPoseArray", new Pose3d[] {poseA, poseB});



  }
  @Override
  public void autonomousInit() {
   QFRCLib.setTab("Autonomous");
    if (trajectory.isPresent()) {
    // Get the initial pose of the trajectory
    Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());
    if (initialPose.isPresent()) {
    // Reset odometry to the start of the trajectory
    //DriveTrainSubsystem.resetOdometry(initialPose.get());
    }
  }
 
   // Reset and start the timer when the autonomous period begins
   timer.restart();
  }
@Override
public void teleopInit() {
  QFRCLib.setTab("Teleop");
}

  @SuppressWarnings("removal")
  @Override
  public void teleopPeriodic() {
      if (drivestick.getAButtonPressed()) {
          new AlignToTargetCommand(drive, vision).schedule();
      }
     if (drivestick.getLeftBumperPressed()) {
      camera.setDriverMode(true);
      } else if (drivestick.getLeftBumperReleased()) {
       camera.setDriverMode(false);
      } 
  }
  @Override
  public void disabledInit() {
      // Runs when robot enters disabled mode
  }

  @Override
  public void disabledPeriodic() {
      // Runs periodically when robot is disabled
  }

  @Override
  public void testInit() {
      // Runs once when the robot enters test mode
  }

  @Override
  public void testPeriodic() {
      // Runs periodically during test mode
  }
  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
   }
}  
