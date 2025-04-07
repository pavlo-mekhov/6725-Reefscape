// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shootersubsystem;
  private final I2C.Port i2cPort = I2C.Port.kMXP;
  // private final DigitalInput limitSwitch = new DigitalInput(0);
  // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  double proximity;



  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(ShooterSubsystem shooterSubsystem) {
    m_shootersubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shootersubsystem.runMotor(0.15);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    proximity = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Proximity", proximity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shootersubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (proximity > 500) {
      return true;
    }
    return false;
  }
}
