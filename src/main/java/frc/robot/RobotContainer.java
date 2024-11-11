// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterMiddleCmd;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final GenericHID controller = new GenericHID(0);
  private final Joystick control = new Joystick(0);
  private final Drive drivesubsystem = new Drive();
  private final Shooter shooter = new Shooter();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivesubsystem.setDefaultCommand(new ArcadeDriveCmd(drivesubsystem, 
    () -> -control.getRawAxis(1), () -> control.getRawAxis(2)));
    shooter.setDefaultCommand(new ShooterCommand(shooter, 0));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    new JoystickButton(control, 1).whileTrue(new ShooterCommand(shooter, 1)).whileFalse(new ShooterCommand(shooter, 0));
    new JoystickButton(control, 2).whileTrue(new ShooterMiddleCmd(shooter,0.5)).whileFalse(new ShooterMiddleCmd(shooter, 0));
    new JoystickButton(control, 3).whileTrue(new IntakeCmd(shooter, 0.5)).whileFalse(new IntakeCmd(shooter, 0));

  }

  public Command getAutonomousCommand(){
    return new SequentialCommandGroup(
      new DriveForwardCommand(drivesubsystem).withTimeout(3),
      new ParallelCommandGroup(
        new IntakeCmd(shooter, 0.8).withTimeout(3),
        new ShooterCommand(shooter,1).withTimeout(6),
        new ShooterMiddleCmd(shooter, 0.8).withTimeout(1),
        new ShooterCommand(shooter, 1).withTimeout(2)
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   // return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
