// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Preferences;

/**
 * {@link}https://github.com/team3673/SwerveDrive_2024
 */
public class PearadoxSparkMax3673 extends SparkFlex {
    protected SparkFlexConfig cfg;
    /**
     * Creates a new CANSparkMax with the necessary configurations.
     * @param deviceId The device ID.
     * @param m The motor type (Brushed/Brushless).
     * @param mode The idle mode (kBrake/kCoast).
     * @param limit The current limit.
     * @param isInverted The invert type of the motor.
     */
    public PearadoxSparkMax3673(int deviceId, MotorType m, IdleMode mode, int limit, boolean isInverted){
        super(deviceId, m);
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.apply(new ExternalEncoderConfig());
        cfg.smartCurrentLimit(limit);
        cfg.inverted(isInverted);
        cfg.apply(new SoftLimitConfig().forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false));
        cfg.apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false));
        cfg.idleMode(mode);

        this.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        String key = "Spark " + this.getDeviceId() + " Flashes";
        Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
    }

    /**
     * Creates a new CANSparkMax with the necessary motor and PID configurations.
     * @param deviceId The device ID.
     * @param m The mo
     * 
     * or type (Brushed/Brushless).
     * @param mode The idle mode (kBrake/kCoast).
     * @param limit The current limit.
     * @param isInverted The invert type of the motor.
     * @param kP The proportional gain value.
     * @param kI The integral gain value.
     * @param kD The derivative gain value.
     * @param minOutput Reverse power minimum to allow the controller to output
     * @param maxOutput Reverse power maximum to allow the controller to output
     */
    public PearadoxSparkMax3673(int deviceId, MotorType m, IdleMode mode, int limit, boolean isInverted, 
        double kP, double kI, double kD, double minOutput, double maxOutput)
    {
        super(deviceId, m);
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.apply(new ExternalEncoderConfig());
        cfg.smartCurrentLimit(limit);
        cfg.inverted(isInverted);
        cfg.apply(new SoftLimitConfig().forwardSoftLimitEnabled(false).reverseSoftLimitEnabled(false));
        cfg.apply(new LimitSwitchConfig().forwardLimitSwitchEnabled(false).reverseLimitSwitchEnabled(false));
        cfg.apply(new ClosedLoopConfig()
            .p(kP, ClosedLoopSlot.kSlot0)
            .i(kI, ClosedLoopSlot.kSlot0)
            .d(kD, ClosedLoopSlot.kSlot0)
            .outputRange(minOutput, maxOutput, ClosedLoopSlot.kSlot0)
        );
        cfg.idleMode(mode);

        this._configure();
        String key = "Spark " + this.getDeviceId() + " Flashes";
        Preferences.setDouble(key, Preferences.getDouble(key, 0) + 1);
    }

    private REVLibError _configure() {
        return this.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public SparkBaseConfig getConfig() {
        return cfg;
    }

    public void setIdleMode(IdleMode m) {
        cfg.idleMode(m);
        _configure();
    }
}