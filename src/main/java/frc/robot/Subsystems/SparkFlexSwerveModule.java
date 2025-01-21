package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Subsystems.Constant.DebugLevel;
import frc.robot.Subsystems.Constant.DebugSetting;
import frc.robot.Subsystems.Constant.DriveConstants;
import frc.robot.Util.CTREModuleState;
import frc.robot.Util.Conversions;

public class SparkFlexSwerveModule extends frc.lib.SwerveModule3673 {
    private String name; 

    public SparkFlexSwerveModule(int turnMotorID, int driveMotorID, int canCoderID, double offset, String name) {
        super(driveMotorID, turnMotorID, false, false, canCoderID, offset, false);
        this.name = name;

        resetTurnToAbsolute();
        applyTurnConfiguration();
    }

    private void resetTurnToAbsolute() {
        var absPos = absoluteEncoder.getAbsolutePosition().refresh();
        double absPosn = absPos.getValueAsDouble(); // rotations
        double absolutePosition = (absPosn * Constant.TwoPI) - absoluteEncoderOffset; // radians

        if (DebugSetting.TraceLevel == DebugLevel.Swerve || DebugSetting.TraceLevel == DebugLevel.All) {
            SmartDashboard.putNumber(name + " Posn abs", absolutePosition);
        }

        // Ensure the turn motor position is set correctly
        // turnMotor.getConfigurator().setPosition(-absolutePosition);
        turnMotor.set(turnPIDController.calculate(absolutePosition, 0));
    }

    private void applyTurnConfiguration() {
        REVLibError status;
        for (int i = 0; i < 5; i++) {
            status = turnMotor.configure(turnMotor.getConfig(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            if (status == REVLibError.kOk) {
                break;
            } else if (i == 4) {
                // Log error if configuration consistently fails
                SmartDashboard.putString(name + " Config Error", status.toString());
            }
        }
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.rpmToMps(driveMotor.getEncoder().getVelocity()); // rpm -> m/s
        Rotation2d angle = Rotation2d.fromRotations(turnMotor.getEncoder().getPosition());
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        double position = driveMotor.getEncoder().getPosition() * DriveConstants.wheelCircumfrence;
        Rotation2d angle = Rotation2d.fromRadians(-turnMotor.getEncoder().getPosition() * Constant.TwoPI);
        return new SwerveModulePosition(position, angle);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        
        setSpeed(desiredState);
        setAngle(desiredState);
    }
}
