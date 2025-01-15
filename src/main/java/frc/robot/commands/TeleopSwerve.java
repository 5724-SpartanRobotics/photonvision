package frc.robot.commands;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.DriveTrainInterface;
import frc.robot.Subsystems.DriveTrainSubsystem;
import frc.robot.Subsystems.Constant.ControllerConstants;
import frc.robot.Subsystems.Constant.DebugLevel;
import frc.robot.Subsystems.Constant.DebugSetting;
import frc.robot.Subsystems.Constant.DriveConstants;

public class TeleopSwerve extends Command {
    private Joystick controller;
    private DriveTrainSubsystem swerveDrive;

    /**
     * Drive Controller
     * @param swerveDrive The drive train subsystem
     * @param controller A joystick
     */
    public TeleopSwerve(DriveTrainSubsystem swerveDrive, Joystick controller){
        this.controller = controller;
        this.swerveDrive = swerveDrive;
        addRequirements((SubsystemBase)swerveDrive);
        if (TimedRobot.isSimulation() && DebugSetting.TraceLevel == DebugLevel.All)
        {
            //SmartDashboard.putNumber("JoyStickY", 0.0);
            //SmartDashboard.putNumber("JoyStickX", 0.0);
            //SmartDashboard.putNumber("JoyStickZ", 0.0);
        }
 }

    @Override
    public void execute(){
        if (DriverStation.isAutonomous()){
            return;
        } else if (controller.getRawButton(7) ) {
            return;
        } else {
            double xAxis;
            double yAxis;
            double zAxis;
            double speedMod = 1;
        // This chunk of code locks certain joystick directions if buttons are pressed

            yAxis = -controller.getY();

            xAxis = -controller.getX();

            zAxis = controller.getTwist();

        // Power Array Auto Align Code
        // Conditional is a check for having a combination of buttons pressed

        
        
            yAxis = (Math.abs(yAxis) < ControllerConstants.joystickDeadband) ? 0 : yAxis * speedMod;
            xAxis = (Math.abs(xAxis) < ControllerConstants.joystickDeadband) ? 0 : xAxis * speedMod;
            zAxis = (Math.abs(zAxis) < ControllerConstants.joystickDeadband) ? 0 : zAxis * speedMod;

            double rotation = zAxis * DriveConstants.maxAngularVelocityRadps;
            if (DebugSetting.TraceLevel == DebugLevel.Swerve) {
                SmartDashboard.putNumber("ControllerRotation", rotation);
                SmartDashboard.putNumber("ControllerX", xAxis);
                SmartDashboard.putNumber("ControllerY", yAxis);
            }
            Translation2d translation = new Translation2d(yAxis, xAxis).times(DriveConstants.maxRobotSpeedmps);
            swerveDrive.drive(translation, rotation);
            if(controller.getRawButton(3)) {
                swerveDrive.setGyroZero();
            }
        }
    }
}