package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Subsystems.Constant.DebugLevel;
import frc.robot.Subsystems.Constant.DebugSetting;
import frc.robot.Subsystems.Constant.DriveConstants;
import frc.robot.Util.CTREModuleState;
import frc.robot.Util.Conversions;

public class SwerveModule {

    private final double offset;
    private final TalonFX turnMotor;
    private final TalonFX driveMotor;
    private final CANcoder canCoder;
    private final PositionVoltage turnVoltageControl;
    private final TalonFXConfiguration turnConfiguration;

    private double driveSpeed = 0.0;
    private double driveAngle = 0.0;
    private final String moduleName;
    private final String canCoderName;

    public SwerveModule(int turnMotorID, int driveMotorID, int canCoderID, double offset, String name) {
        this.offset = offset;
        this.moduleName = name;

        // Initialize motors and encoder
        turnMotor = new TalonFX(turnMotorID);
        driveMotor = new TalonFX(driveMotorID);
        canCoder = new CANcoder(canCoderID);
        canCoderName = name + canCoderID;

        // Initialize control and configuration objects
        turnVoltageControl = new PositionVoltage(0);
        turnConfiguration = new TalonFXConfiguration();

        initializeMotors();
        resetTurnToAbsolute();
        applyTurnConfiguration();
    }

    private void initializeMotors() {
        // Configure the drive motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveMotor.getConfigurator().apply(driveConfig);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);

        // Configure the turn motor
        turnConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turnConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        turnConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        turnConfiguration.HardwareLimitSwitch.ForwardLimitEnable = false;
        turnConfiguration.HardwareLimitSwitch.ReverseLimitEnable = false;
        turnConfiguration.Slot0.kP = 0.9; // Ensure this value is tuned for your setup
        turnConfiguration.Voltage.PeakForwardVoltage = 10.0;
        turnConfiguration.Voltage.PeakReverseVoltage = -10.0;

        turnMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private void resetTurnToAbsolute() {
        double absolutePosition = Conversions.radiansToFalcon(
            (Units.degreesToRadians(canCoder.getAbsolutePosition().getValueAsDouble())) - offset
        );

        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All) {
            SmartDashboard.putNumber(moduleName + " Posn abs", absolutePosition);
        }

        // Ensure the turn motor position is set correctly
        turnMotor.getConfigurator().setPosition(-absolutePosition);

    }

    private void applyTurnConfiguration() {
        StatusCode status;
        for (int i = 0; i < 5; i++) {
            status = turnMotor.getConfigurator().apply(turnConfiguration);
            if (status.isOK()) {
                break;
            } else if (i == 4) {
                // Log error if configuration consistently fails
                SmartDashboard.putString(moduleName + " Config Error", status.getDescription());
            }
        }
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(driveMotor.getVelocity().getValueAsDouble());
        Rotation2d angle = Rotation2d.fromRadians(Conversions.falconToRadians(-turnMotor.getPosition().getValueAsDouble()));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double position = Conversions.falconToMeters(driveMotor.getPosition().getValueAsDouble());
        Rotation2d angle = Rotation2d.fromRadians(Conversions.falconToRadians(-turnMotor.getPosition().getValueAsDouble()));
        return new SwerveModulePosition(position, angle);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        driveSpeed = desiredState.speedMetersPerSecond / DriveConstants.maxRobotSpeedmps;

        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All) {
            SmartDashboard.putNumber(moduleName + " DriveRef", driveSpeed);
        }

        driveMotor.set(driveSpeed); // Ensure the motor control mode is set appropriately

        double angle = Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.maxRobotSpeedmps * 0.01)
            ? driveAngle
            : desiredState.angle.getRadians();

        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All) {
            SmartDashboard.putNumber(moduleName + " TurnRef", Units.radiansToDegrees(angle));
        }

        turnMotor.setControl(turnVoltageControl.withPosition(-Conversions.radiansToFalcon(angle)));
        driveAngle = angle;
    }

    public void periodic() {
        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All) {
            SmartDashboard.putNumber("Pos FB " + moduleName, Units.radiansToDegrees(
                Conversions.falconToRadians(turnMotor.getPosition().getValueAsDouble())
            ));
            SmartDashboard.putNumber(canCoderName, Units.radiansToDegrees(
                (Units.degreesToRadians(canCoder.getAbsolutePosition().getValueAsDouble())) - offset
            ));
            SmartDashboard.putNumber("Drive FB " + moduleName, driveMotor.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("DriveAngle " + moduleName, Units.radiansToDegrees(driveAngle));
        }
    }
}
