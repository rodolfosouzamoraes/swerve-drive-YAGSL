// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Auto_NoteMiddle extends Command {
  // Swerve Variáveis
  private final SwerveSubsystem _drivebase;
  private final double _speed = 0.5;
  private boolean _isFinish = false;
  private double _poseX;
  private double _poseY;

  // Collect Variáveis
  private final IntakeSubsystem _intakeSubsystem;
  /** Creates a new Auto_NoteMiddle. */
  public Auto_NoteMiddle(SwerveSubsystem drivebase, IntakeSubsystem intakeSubsystem, double poseX, double poseY) {
    _drivebase = drivebase;
    _intakeSubsystem = intakeSubsystem;
    _poseX = poseX;
    _poseY = poseY;

    addRequirements(_drivebase,_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _isFinish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Pose Swerve",_drivebase.getPose().getX());
    if(_intakeSubsystem.getSensorCollect() == false){
      _drivebase.drive(new Translation2d(_poseX, _poseY),0,false);
    }
    else{
      if(_drivebase.getPose().getX() >= 0){
        _drivebase.drive(new Translation2d(-1, 0),0,false);
      }
      else{
        _drivebase.drive(new Translation2d(0, 0),0,false);
        _isFinish = true;
      }      
    }
    _intakeSubsystem.collectNote(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _isFinish;
  }
}
