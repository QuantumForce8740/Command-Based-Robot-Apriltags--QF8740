// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import java.util.function.Supplier;
public class ArcadeDriveCmd extends Command {
  /** Creates a new ArcadeDriveCmd. */
  private final Drive drivesubsystem;
  private final Supplier<Double> speedfunction, turnfunction;
  public ArcadeDriveCmd(Drive drivesubsystem, Supplier<Double> speedfunction, Supplier<Double> turnfunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivesubsystem = drivesubsystem;
    this.speedfunction = speedfunction;
    this.turnfunction = turnfunction;
    addRequirements(drivesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double realTimeSpeed = speedfunction.get();
    double realTimeTurn = turnfunction.get();

    double left = realTimeSpeed + realTimeTurn;
    double right = realTimeSpeed - realTimeTurn;
    drivesubsystem.set_motors(left, right);
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
