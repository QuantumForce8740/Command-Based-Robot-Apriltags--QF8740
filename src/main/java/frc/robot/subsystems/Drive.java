// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  private final PWMSparkMax left_back_m = new PWMSparkMax(4);
  private final PWMSparkMax left_front_m = new PWMSparkMax(3);
  private final PWMSparkMax right_back_m = new PWMSparkMax(2);
  private final PWMSparkMax right_front_m = new PWMSparkMax(1);
  // private final DifferentialDrive drive = new DifferentialDrive(
  //   (double output) -> {
  //     left_front_m.set(output);
  //     left_back_m.set(output);
  //   },
  //   (double output) ->{
  //     right_front_m.set(output);
  //     right_back_m.set(output);
  //   });
  
  /** Creates a new Drive. */
  public Drive() {
    left_back_m.setInverted(true);
    left_front_m.setInverted(true);
  }

  public void set_motors(double leftSpeed, double rightSpeed){
    left_front_m.set(leftSpeed);
    left_back_m.set(leftSpeed);
    right_front_m.set(-rightSpeed);
    right_back_m.set(-rightSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
