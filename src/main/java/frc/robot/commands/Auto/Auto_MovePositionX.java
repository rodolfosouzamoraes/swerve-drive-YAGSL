// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Auto_MovePositionX extends Command {
  private final SwerveSubsystem _drivebase;
  private double _speed;
  private double _distanceX;
  private boolean _isFinish = false;
  /** Creates a new Auto_MovePosition. */
  public Auto_MovePositionX(SwerveSubsystem drivebase, double speed, double distanceX) {
    _drivebase = drivebase;
    _speed = speed;
    _distanceX = distanceX;
    addRequirements(_drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _isFinish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double positionX = _drivebase.getPose().getX()*100;
    if(_speed>0){
      if(positionX<=_distanceX){
        _drivebase.drive(new Translation2d(_speed, 0),0,false);
      }
      else{
        _drivebase.drive(new Translation2d(0, 0),0,false);
        _isFinish = true;
      }
    }
    else{
      if(positionX>=_distanceX){
        _drivebase.drive(new Translation2d(_speed, 0),0,false);
      }
      else{
        _drivebase.drive(new Translation2d(0, 0),0,false);
        _isFinish = true;
      }
    }   
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
