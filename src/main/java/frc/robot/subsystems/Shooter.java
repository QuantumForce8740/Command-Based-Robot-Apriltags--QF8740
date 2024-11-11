// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final CANSparkMax upshooter = new CANSparkMax(5, MotorType.kBrushed);
  private final CANSparkMax downshooter = new CANSparkMax(6, MotorType.kBrushed);
  private final CANSparkMax middle_shooter = new CANSparkMax(3, MotorType.kBrushed);
  private final CANSparkMax intakeupper = new CANSparkMax(2, MotorType.kBrushed);
  private final CANSparkMax intakeup = new CANSparkMax(1, MotorType.kBrushed);
  /** Creates a new Shooter. */
  public Shooter() {
    upshooter.setInverted(true);
  }

  public void shoot(double speed){
    upshooter.set(speed);
    downshooter.set(speed);
  }

  public void middle_shoot(double speed){
    middle_shooter.set(speed);
  }

  public void intake(double speed){
    intakeup.set(speed);
    intakeupper.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
