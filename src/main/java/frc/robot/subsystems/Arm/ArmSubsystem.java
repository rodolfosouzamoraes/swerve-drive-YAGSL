// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax leftMotorLeader;
  private CANSparkMax rightMotorLeader;
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private DifferentialDrive myArm;

  private Encoder encoder;

  public ArmSubsystem() {
    configureMotors();
    configureEncoder();
  }

  private void configureEncoder(){
    encoder = new Encoder(Constants.ArmConstants.kChannelA,Constants.ArmConstants.kChannelB,true);
    encoder.setDistancePerPulse(Constants.ArmConstants.kFactorConvertionPosition);
    resetEncoder();
  }

  private void configureMotors(){
    leftMotorLeader = new CANSparkMax(Constants.ArmConstants.kLeftMotorLeader, MotorType.kBrushless);
    leftMotorLeader.restoreFactoryDefaults();
    leftMotorLeader.setInverted(false);
    leftMotorLeader.setIdleMode(IdleMode.kBrake);
    leftMotorLeader.setOpenLoopRampRate(0);
    leftMotorLeader.setSmartCurrentLimit(60,80);

    leftMotor = new CANSparkMax(Constants.ArmConstants.kLeftMotor, MotorType.kBrushless);
    leftMotor.restoreFactoryDefaults();
    leftMotor.setInverted(false);
    leftMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setOpenLoopRampRate(0);
    leftMotor.setSmartCurrentLimit(60,80);
    leftMotor.follow(leftMotorLeader);

    rightMotorLeader = new CANSparkMax(Constants.ArmConstants.kRightMotorLeader, MotorType.kBrushless);
    rightMotorLeader.restoreFactoryDefaults();
    rightMotorLeader.setInverted(true);
    rightMotorLeader.setIdleMode(IdleMode.kBrake);
    rightMotorLeader.setOpenLoopRampRate(0);
    rightMotorLeader.setSmartCurrentLimit(60,80);

    rightMotor = new CANSparkMax(Constants.ArmConstants.kRightMotor, MotorType.kBrushless);
    rightMotor.restoreFactoryDefaults();
    rightMotor.setInverted(true);
    rightMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setOpenLoopRampRate(0);
    rightMotor.setSmartCurrentLimit(60,80);
    rightMotor.follow(rightMotorLeader);

    leftMotorLeader.burnFlash();
    leftMotor.burnFlash();
    rightMotorLeader.burnFlash();
    rightMotor.burnFlash();

    myArm = new DifferentialDrive(leftMotorLeader,rightMotorLeader);
    myArm.setSafetyEnabled(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Arm",getEncoderArm());
    // This method will be called once per scheduler run
  }

  public void moveManualArm(double speed){
    myArm.tankDrive(speed, speed);
  }

  public void moveArmVoltage(double voltage){
    leftMotorLeader.setVoltage(voltage);
    rightMotorLeader.setVoltage(voltage);
  }

  public void stopArm(){
    myArm.stopMotor();
  }

  public void setMaxOutput(double maxOutPut){
    myArm.setMaxOutput(maxOutPut);
  }

  public double getEncoderArm(){
    if(encoder.getDistance() < 0){
      resetEncoder();
    }
    return encoder.getDistance();
  }

  public void resetEncoder(){
    encoder.reset();
  }
}
