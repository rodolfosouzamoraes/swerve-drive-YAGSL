// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sequentials;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto.Auto_CollectNoteArena;
import frc.robot.commands.Auto.Auto_MovePositionX;
import frc.robot.commands.Auto.Auto_RotationTank;
import frc.robot.commands.Auto.Auto_ShooterNoteCmd;
import frc.robot.subsystems.Arm.ArmPidSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SequentialSkeakerRight extends SequentialCommandGroup {
  /** Creates a new SequentialSkeakerRight. */
  public SequentialSkeakerRight(IntakeSubsystem intakeSubsystem, ArmPidSubsystem armPidSubsystem, SwerveSubsystem drivebase) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new Auto_RotationTank(drivebase, 0.5,45)
      new InstantCommand(() -> drivebase.resetPose(),drivebase),
      new Auto_ShooterNoteCmd(armPidSubsystem,10.3,1,intakeSubsystem),
      new Auto_MovePositionX(drivebase, 0.5, 37),
      new Auto_RotationTank(drivebase, 0.5, -9.5),
      new InstantCommand(() -> drivebase.resetPose(),drivebase),
      new Auto_CollectNoteArena(drivebase, intakeSubsystem, 0.5, 0, 0),
      new Auto_RotationTank(drivebase, -0.5, -50),
      new InstantCommand(() -> drivebase.resetPose(),drivebase),
      new Auto_MovePositionX(drivebase, -0.5, -37),
      new Auto_ShooterNoteCmd(armPidSubsystem,10.3,1,intakeSubsystem),
      new InstantCommand(() -> drivebase.resetPose(),drivebase)
      //new Auto_CollectNoteArena(drivebase, intakeSubsystem,0.45,1,0)
    );
  }
}
