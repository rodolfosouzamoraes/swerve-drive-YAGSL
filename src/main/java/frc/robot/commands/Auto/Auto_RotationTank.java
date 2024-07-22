// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Auto_RotationTank extends Command {
  private final SwerveSubsystem _drivebase;
  private double _speed;
  private double _target;
  private boolean isFinish = false;
  /** Creates a new Auto_RotationTank. */
  public Auto_RotationTank(SwerveSubsystem drivebase, double speed, double angle) {
    _drivebase = drivebase;
    _speed = speed;
    _target = angle;
    addRequirements(_drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = _drivebase.getHeading().getDegrees(); 
    if(_speed>0){
      if(angle < _target){ //-58 -6
      _drivebase.drive(new Translation2d(0, 0),_speed,false);
      }
      else{
        _drivebase.drive(new Translation2d(0, 0),0,false);
        isFinish = true;
      }
    }
    else{
      if(angle > _target){
        _drivebase.drive(new Translation2d(0, 0),_speed,false);
      }
      else{
          _drivebase.drive(new Translation2d(0, 0),0,false);
          isFinish = true;
      }
    }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinish;
  }
}
