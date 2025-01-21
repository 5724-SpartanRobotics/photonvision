// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Constant;
import frc.robot.Subsystems.Constant.DriveConstants;

/**
 * {@link}https://github.com/team3673/SwerveDrive_2024
 */
public class SwerveModule3673 extends SubsystemBase {
  protected PearadoxSparkMax3673 driveMotor;
  protected PearadoxSparkMax3673 turnMotor;

  protected RelativeEncoder driveEncoder;
  protected RelativeEncoder turnEncoder;

  protected PIDController turnPIDController;
  protected CANcoder absoluteEncoder;

  protected boolean absoluteEncoderReversed;
  protected double absoluteEncoderOffset;

  protected int driveID = 0;

  protected Rotation2d lastAngle;

  /** Creates a new SwerveModule. */
  public SwerveModule3673(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
      this.absoluteEncoderOffset = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;

      driveID = driveMotorId;
      absoluteEncoder = new CANcoder(absoluteEncoderId);

      driveMotor = new PearadoxSparkMax3673(driveMotorId, MotorType.kBrushless, IdleMode.kCoast, 45, driveMotorReversed);
      turnMotor = new PearadoxSparkMax3673(turnMotorId, MotorType.kBrushless, IdleMode.kCoast, 25, turnMotorReversed);

      driveEncoder = driveMotor.getEncoder();
      turnEncoder = turnMotor.getEncoder();

      turnPIDController = new PIDController(0.5, 0, 0);
      turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

      resetEncoders();
      lastAngle = getState().angle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Drive Distance (rot) - Motor: " + driveID, getDriveMotorPosition());
    SmartDashboard.putNumber("Wheel Position (rot) - Motor: " + driveID, getTurnMotorPosition());
    SmartDashboard.putNumber("Absolute Wheel Angle (deg) - Motor: " + driveID, absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    
  }

  public void setBrake(boolean brake){
    if(brake){
      driveMotor.setIdleMode(IdleMode.kBrake);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
    else{
      driveMotor.setIdleMode(IdleMode.kCoast);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
  }
  
  public double getDriveMotorPosition(){
    return driveEncoder.getPosition() * (DriveConstants.wheelDiameter * Math.PI * DriveConstants.driveGearRatio);
  }

  public double getDriveMotorVelocity(){
    return driveEncoder.getVelocity() * (DriveConstants.wheelDiameter * Math.PI * DriveConstants.driveGearRatio) / 60D;
  }

  public double getTurnMotorPosition(){
    return turnEncoder.getPosition() * (Constant.TwoPI / DriveConstants.turnGearRatio);
  }

  public double getTurnMotorVelocity(){
    return turnEncoder.getVelocity() * (Constant.TwoPI / DriveConstants.turnGearRatio) / 60D;
  }

  public double getAbsoluteEncoderAngle(){
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    angle -= absoluteEncoderOffset;
    angle *= (2 * Math.PI);
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
    turnEncoder.setPosition(getAbsoluteEncoderAngle() / (Constant.TwoPI / DriveConstants.turnGearRatio));
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getTurnMotorPosition()));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getTurnMotorPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState){
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
    
    setAngle(desiredState);
    setSpeed(desiredState);
    SmartDashboard.putString("Swerve [" + driveMotor.getDeviceId() + "] State", getState().toString());
    SmartDashboard.putNumber("Abs Angle " + driveMotor.getDeviceId(), getAbsoluteEncoderAngle());
  }

  public void setSpeed(SwerveModuleState desiredState){
    driveMotor.set(desiredState.speedMetersPerSecond / DriveConstants.maxRobotSpeedmps);
  }

  public void setAngle(SwerveModuleState desiredState){
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (DriveConstants.maxRobotSpeedmps * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    
    turnMotor.set(turnPIDController.calculate(getTurnMotorPosition(), desiredState.angle.getRadians()));
    lastAngle = angle;
  }

  public void stop(){
    driveMotor.set(0);
    turnMotor.set(0);
  }

  public String getAbsoluteEncoderName(String name) {
    return name + absoluteEncoder.getDeviceID();
  }
}
