// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax leftMotorShooter;
  private CANSparkMax rightMotorShooter;
  private CANSparkMax leftMotorCollect;
  private CANSparkMax rightMotorCollect;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    leftMotorShooter = new CANSparkMax(Constants.IntakeConstants.kLeftMotorShooter, MotorType.kBrushless);
    leftMotorShooter.restoreFactoryDefaults();
    leftMotorShooter.setInverted(true);
    leftMotorShooter.setIdleMode(IdleMode.kBrake);
    leftMotorShooter.setOpenLoopRampRate(0);
    leftMotorShooter.setSmartCurrentLimit(60,80);

    rightMotorShooter = new CANSparkMax(Constants.IntakeConstants.kRightMotorShooter, MotorType.kBrushless);
    rightMotorShooter.restoreFactoryDefaults();
    rightMotorShooter.setInverted(true);
    rightMotorShooter.setIdleMode(IdleMode.kBrake);
    rightMotorShooter.setOpenLoopRampRate(0);
    rightMotorShooter.setSmartCurrentLimit(60,80);

    leftMotorCollect = new CANSparkMax(Constants.IntakeConstants.kLeftMotorCollect, MotorType.kBrushless);
    leftMotorCollect.restoreFactoryDefaults();
    leftMotorCollect.setInverted(false);
    leftMotorCollect.setIdleMode(IdleMode.kBrake);
    leftMotorCollect.setOpenLoopRampRate(0);
    leftMotorCollect.setSmartCurrentLimit(60,80);

    rightMotorCollect = new CANSparkMax(Constants.IntakeConstants.kRightMotorCollect, MotorType.kBrushless);
    rightMotorCollect.restoreFactoryDefaults();
    rightMotorCollect.setInverted(false);
    rightMotorCollect.setIdleMode(IdleMode.kBrake);
    rightMotorCollect.setOpenLoopRampRate(0);
    rightMotorCollect.setSmartCurrentLimit(60,80);

    leftMotorCollect.burnFlash();
    leftMotorShooter.burnFlash();
    rightMotorShooter.burnFlash();
    rightMotorCollect.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shooterNote(double value){
    leftMotorShooter.set(value);
    rightMotorShooter.set(value);
  }

  public void collectNote(double value){
    leftMotorCollect.set(value);
    rightMotorCollect.set(value);
  }

  public void stopIntake(){
    leftMotorCollect.stopMotor();
    leftMotorShooter.stopMotor();
    rightMotorCollect.stopMotor();
    rightMotorShooter.stopMotor();
  }
}
