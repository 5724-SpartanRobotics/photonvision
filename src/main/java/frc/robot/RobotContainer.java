package frc.robot;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.Constant;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.commands.TeleopSwerve;

public class RobotContainer {
    private final DriveTrainSubsystem drive;
    private final Joystick drivestick;

    private JoystickButton jb_ZeroGyro;
    private JoystickButton jb_AlignApriltag;
    private JoystickButton jb_AlignLimelighObject;
    private JoystickButton jb_Rotate180;

    public RobotContainer() {
        this.drive = new DriveTrainSubsystem();
        this.drivestick = new Joystick(0);

        // this.vision = new VisionSubsystem2024(drive);
        // this.vision = new ApriltagLockon2Subsystem(drive, photon);
        // this.vision = new ApriltagLockonSubsystem(drive, photon, drivestick);

        this.jb_ZeroGyro = new JoystickButton(drivestick, Constant.ControllerConstants.ButtonMap.GyroZero);
        this.jb_AlignApriltag = new JoystickButton(drivestick, Constant.ControllerConstants.ButtonMap.TagLockonAlt);
        this.jb_AlignLimelighObject = new JoystickButton(drivestick, Constant.ControllerConstants.ButtonMap.ObjectLockonAlt);
        this.jb_Rotate180 = new JoystickButton(drivestick, Constant.ControllerConstants.ButtonMap.Rotate180);

        configureButtonBindings();
        
        drive.setDefaultCommand(new TeleopSwerve(drive, drivestick));
    }
        
    private void configureButtonBindings() {
        jb_ZeroGyro.onTrue(new InstantCommand(() -> {drive.setGyroZero();}));
        jb_Rotate180.onTrue(new InstantCommand(() -> {drive.drive(-Math.PI);}));
    }
}
