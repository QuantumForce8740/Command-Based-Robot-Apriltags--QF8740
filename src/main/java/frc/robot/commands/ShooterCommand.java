// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {
  private final Shooter shooter;
  private final double speed;
  
  /** Creates a new ShooterCommand. */
  public ShooterCommand(Shooter shooter, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.speed = speed;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shoot started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.shoot(speed);
    // if (controller.getRawButton(buttonA)){
    //   shooter.shoot(-0.3);
    // } else if (controller.getRawButton(buttonB)){
    //   shooter.shoot(1);
    // }else {
    //   shooter.shoot(0);
    // }

    // if (controller.getRawButton(buttonX)){
    //   shooter.middle_shoot(0.5);
    // }else {
    //   shooter.middle_shoot(0);
    // }

    // if (controller.getRawButton(buttonY)){
    //   shooter.intake(1);
    // }else {
    //   shooter.intake(0);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shoot(0);
    System.out.println("Shoot ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
