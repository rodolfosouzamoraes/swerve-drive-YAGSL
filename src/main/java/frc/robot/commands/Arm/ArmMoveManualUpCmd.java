// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class ArmMoveManualUpCmd extends Command {
  private final ArmSubsystem _armSubsystem;
  private double speed = 0.5;
  /** Creates a new ArmMoveCmd. */
  public ArmMoveManualUpCmd(ArmSubsystem armSubsystem) {
    _armSubsystem = armSubsystem;
    addRequirements(_armSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _armSubsystem.setMaxOutput(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _armSubsystem.moveManualArm(-1*speed);
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