package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final XboxController xboxController = new XboxController(0);
  private final JoystickButton aButton = new JoystickButton(xboxController, XboxController.Button.kA.value);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    aButton.onTrue(new InstantCommand(() -> swerveSubsystem.resetGyro()));
  }
}
