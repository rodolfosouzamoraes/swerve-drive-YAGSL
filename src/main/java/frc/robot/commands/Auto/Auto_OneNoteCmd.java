// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.ArmPidSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class Auto_OneNoteCmd extends Command {
  // Arm Variáveis
  private final ArmPidSubsystem _armPidSubsystem;
  private final double _setPosition;
  private final double _maxOutput;
  private final Timer _timer;
  private final double _timerArmWait = 1.25;
  
  // Intake Variáveis
  private final IntakeSubsystem _intakeSubsystem;
  private double _speedShooter;
  private double _timerWait = 2.25;
  private double _timerShooter = 0;
  private double _timerFinishShooter = 0;
  private double _timerWaitFinish = 0.75;
  private boolean _isShooter = false;
  /** Creates a new ArmPositionCmd. */
  public Auto_OneNoteCmd(ArmPidSubsystem armPidSubsystem, double setPoint, double maxOutPut, IntakeSubsystem intakeSubsystem) {
    _armPidSubsystem = armPidSubsystem;
    _setPosition = setPoint;
    _maxOutput = maxOutPut;
    _timer = new Timer();

    _intakeSubsystem = intakeSubsystem;
    _speedShooter = 1;
    addRequirements(_armPidSubsystem,_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _timer.restart();

    _timerShooter = _timer.get() + _timerWait;
    _timerFinishShooter = _timerShooter + _timerWaitFinish;
    _isShooter = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_timer.get()<= _timerArmWait){
      _armPidSubsystem.useOutput(_maxOutput, _setPosition);   
    }
    else{
      ShooterSpeake();
    }
      
  }

  private void ShooterSpeake(){
    if(_isShooter == false){
      _intakeSubsystem.shooterNoteSpeaker(_speedShooter);   
      if(_timerShooter < _timer.get()){
        _intakeSubsystem.pushNote(0.3);
        if(_timerFinishShooter < _timer.get()){
          _isShooter = true;
          _armPidSubsystem.stopFeeder();
          _intakeSubsystem.stopIntake();
        }
      }
    }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _armPidSubsystem.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
