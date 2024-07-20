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
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Auto_NoteMiddle extends Command {
  private final SwerveSubsystem _drivebase;
  private final Timer _timer;
  private final double _timerMoveForward = 1;
  private final double _speed = 0.5;
  /** Creates a new Auto_NoteMiddle. */
  public Auto_NoteMiddle(SwerveSubsystem drivebase) {
    _drivebase = drivebase;
    _timer = new Timer();

    addRequirements(_drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_drivebase.getPose().getX()<= 1){
      SmartDashboard.putNumber("Pose Swerve",_drivebase.getPose().getX());
      _drivebase.drive(new Translation2d(1, 0),0,false);
    }
    else{
      _drivebase.drive(new Translation2d(0, 0),0,false);
    }
  }

  private void MoveY(double direction){
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
