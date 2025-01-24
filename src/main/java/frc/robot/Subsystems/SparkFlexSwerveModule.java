package frc.robot.Subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Constant.DebugLevel;
import frc.robot.Subsystems.Constant.DebugSetting;

public class SparkFlexSwerveModule extends frc.lib.SwerveModule3673 {
    private String name; 

    public SparkFlexSwerveModule(int turnMotorID, int driveMotorID, int canCoderID, double offset, String name) {
        super(driveMotorID, turnMotorID, false, false, canCoderID, offset, false);
        this.name = name;

        resetTurnToAbsolute();
        applyTurnConfiguration();
    }

    public void resetTurnToAbsolute() {
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

    public void applyTurnConfiguration() {
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
}
