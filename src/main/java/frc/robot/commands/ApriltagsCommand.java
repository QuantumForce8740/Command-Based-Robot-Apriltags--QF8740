// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Apriltags;

public class ApriltagsCommand extends Command {
  /** Creates a new ApriltagsCommand. */
  private final Apriltags apriltags_subsystem;
  public ApriltagsCommand(Apriltags apriltags_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.apriltags_subsystem = apriltags_subsystem;
    addRequirements(apriltags_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    apriltags_subsystem.startVision();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
