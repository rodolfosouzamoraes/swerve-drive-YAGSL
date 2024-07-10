// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));



  final CommandXboxController driverXbox = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
    /*
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);


/*     AbsoluteDrive absoluteDrive = new AbsoluteDrive(drivebase,
                                  () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                              OperatorConstants.LEFT_Y_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                              OperatorConstants.LEFT_X_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                              OperatorConstants.RIGHT_X_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                              OperatorConstants.RIGHT_X_DEADBAND));
 */
/*
    AbsoluteFieldDrive absoluteFieldDrive = new AbsoluteFieldDrive(drivebase,
                                  () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                              OperatorConstants.LEFT_Y_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                              OperatorConstants.LEFT_X_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                              OperatorConstants.RIGHT_X_DEADBAND));
*/
                                                              /*
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX(),
      () -> driverXbox.getRightY());
    
       */

      Command baseDriveCommand = drivebase.driveCommand(        
        () -> MathUtil.applyDeadband(driverXbox.getLeftY()*-1, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX()*-1, OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightX()*0.85*-1,.1));

    drivebase.setDefaultCommand(baseDriveCommand); //padrão
    //drivebase.setDefaultCommand(absoluteFieldDrive); // Anda somente para frente/atrás e lados, não gira no eixo
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void setDriveMode() {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
