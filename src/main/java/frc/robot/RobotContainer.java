package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.Constant;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.LimelightLockonSubsystem;
import frc.robot.Subsystems.PhotonVisionSubsystem;
import frc.robot.Subsystems.VisionSubsystem2024;
import frc.robot.commands.GotToAPlace2024;
import frc.robot.commands.LimelightAlignToTargetCommand;
import frc.robot.commands.TeleopSwerve;

public class RobotContainer {
    private final DriveTrainSubsystem drive;
    private final PhotonVisionSubsystem photon;
    private final VisionSubsystem2024 vision;
    private final LimelightLockonSubsystem limelightLockon;
    private final Joystick drivestick;
    private final Timer timer;

    private JoystickButton jb_ZeroGyro;
    private JoystickButton jb_AlignApriltag;
    private JoystickButton jb_AlignLimelighObject;

    public RobotContainer() {
        this.drive = new DriveTrainSubsystem();
        this.photon = new PhotonVisionSubsystem("Front");
        this.vision = new VisionSubsystem2024(drive);
        this.limelightLockon = new LimelightLockonSubsystem(drive);
        this.drivestick = new Joystick(0);
        this.timer = new Timer();

        this.jb_ZeroGyro = new JoystickButton(drivestick, Constant.ControllerConstants.ButtonMap.GyroZero);
        this.jb_AlignApriltag = new JoystickButton(drivestick, Constant.ControllerConstants.ButtonMap.TagLockon);
        this.jb_AlignLimelighObject = new JoystickButton(drivestick, Constant.ControllerConstants.ButtonMap.ObjectLockon);

        configureButtonBindings();
        
        drive.setDefaultCommand(new TeleopSwerve(drive, drivestick));
    }
        
    private void configureButtonBindings() {
        jb_AlignApriltag.whileTrue(
            new GotToAPlace2024(drive, vision, new Pose2d(), 3, drivestick, false)
        );
        jb_AlignLimelighObject.whileTrue(
            new LimelightAlignToTargetCommand(drive, limelightLockon, new Pose2d(), 3, drivestick, false)
        );
        jb_ZeroGyro.onTrue(new InstantCommand(() -> {drive.setGyroZero();}));
    }
}
