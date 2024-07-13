// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Arm.ArmMoveManualUpCmd;
import frc.robot.commands.Arm.ArmPositionCmd;
import frc.robot.commands.Intake.IntakeCollectCmd;
import frc.robot.commands.Intake.IntakeShooterCmd;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.subsystems.Arm.ArmPidSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem _drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  
  private final IntakeSubsystem _intakeSubsystem = new IntakeSubsystem();
  //private final ArmSubsystem _armSubsystem = new ArmSubsystem();
  private final ArmPidSubsystem _armPidSubsystem = new ArmPidSubsystem();
                                                                         
  private final XboxController _driverXbox = new XboxController(0);
  private final XboxController _armIntakeXbox = new XboxController(1);

  public RobotContainer() {
    configureBindings();

    configureSwerve();
    configureIntake();
    configureArm();
  }

  private void configureArm(){
    //_armSubsystem.setDefaultCommand(new InstantCommand(() -> _armSubsystem.stopArm(), _armSubsystem));
  }

  private void configureSwerve(){
    Command baseDriveCommand = _drivebase.driveCommand(        
        () -> MathUtil.applyDeadband(_driverXbox.getLeftY()*-1, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(_driverXbox.getLeftX()*-1, OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(_driverXbox.getRightX()*0.85*-1,.1));

    _drivebase.setDefaultCommand(baseDriveCommand);
  }

  private void configureIntake(){
    _intakeSubsystem.setDefaultCommand(new InstantCommand(() -> _intakeSubsystem.stopIntake(), _intakeSubsystem));
  }

  private void configureBindings() {
    initializeIntakeXboxController();
    initializeArmXboxController();
    initializeArmToPositionXboxController();
  }

  private void initializeIntakeXboxController(){
    Trigger shooterNote = new Trigger(() -> Math.abs(_armIntakeXbox.getRightTriggerAxis())>0.1);
    shooterNote.whileTrue(new IntakeShooterCmd(_intakeSubsystem));

    Trigger shooterAmp = new Trigger(() -> Math.abs(_armIntakeXbox.getLeftTriggerAxis())>0.1);
    shooterAmp.whileTrue(new IntakeShooterCmd(_intakeSubsystem,0.5));

    JoystickButton collectNote = new JoystickButton(_armIntakeXbox,Button.kA.value);
    collectNote.whileTrue(new IntakeCollectCmd(_intakeSubsystem));
  }

  private void initializeArmXboxController(){
    //JoystickButton armMoveUp = new JoystickButton(_armIntakeXbox,Button.kX.value);
    //armMoveUp.whileTrue(new ArmMoveManualUpCmd(_armSubsystem));
  }

  private void initializeArmToPositionXboxController(){
    //JoystickButton armMoveArmToPosition = new JoystickButton(_armIntakeXbox,Button.kB.value);
    //armMoveArmToPosition.onTrue(new ArmPositionCmd(_armSubsystem,30,0.6)
    //.withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
    JoystickButton armMoveArmTo20Position = new JoystickButton(_armIntakeXbox,Button.kB.value);
    armMoveArmTo20Position.onTrue(new ArmPositionCmd(_armPidSubsystem,15,1));

    JoystickButton armMoveArmTo5Position = new JoystickButton(_armIntakeXbox,Button.kX.value);
    armMoveArmTo5Position.onTrue(new ArmPositionCmd(_armPidSubsystem,5,1));

    JoystickButton armMoveArmTo30Position = new JoystickButton(_armIntakeXbox,Button.kY.value);
    armMoveArmTo30Position.onTrue(new ArmPositionCmd(_armPidSubsystem,36,1));

    JoystickButton armMoveToDownPosition = new JoystickButton(_armIntakeXbox,Button.kRightBumper.value);
    armMoveToDownPosition.onTrue(new InstantCommand(() -> _armPidSubsystem.stopFeeder(),_armPidSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void setDriveMode() {

  }

  public void setMotorBrake(boolean brake) {
    _drivebase.setMotorBrake(brake);
  }

  private void trashCode(){
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
     //drivebase.setDefaultCommand(absoluteFieldDrive); // Anda somente para frente/atrás e lados, não gira no eixo
  }
}
