// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private CANSparkMax leftMotorLeader;
  private CANSparkMax rightMotorLeader;
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private DifferentialDrive myArm;

  public ArmSubsystem() {
    configureMotors();
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
    // This method will be called once per scheduler run
  }

  public void moveManualArm(double speed){
    myArm.tankDrive(speed, speed);
  }

  public void stopArm(){
    myArm.stopMotor();
  }

  public void setMaxOutput(double maxOutPut){
    myArm.setMaxOutput(maxOutPut);
  }
}
