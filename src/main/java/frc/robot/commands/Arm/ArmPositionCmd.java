// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.ArmPidSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class ArmPositionCmd extends Command {
  private final ArmPidSubsystem _armPidSubsystem;
  private final double _setPosition;
  private final double _maxOutput;
  /** Creates a new ArmPositionCmd. */
  public ArmPositionCmd(ArmPidSubsystem armSubsystem, double setPoint, double maxOutPut) {
    _armPidSubsystem = armSubsystem;
    _setPosition = setPoint;
    _maxOutput = maxOutPut;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   _armPidSubsystem.useOutput(_maxOutput, _setPosition);      
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
