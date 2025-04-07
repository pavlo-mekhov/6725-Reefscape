// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterCommandScore extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shootersubsystem;

    private final I2C.Port i2cPort = I2C.Port.kMXP;
      private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
      double proximity = m_colorSensor.getProximity();



  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommandScore(ShooterSubsystem shooterSubsystem) {
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
    proximity = m_colorSensor.getProximity();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shootersubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (proximity < 300) {
      try {
        Thread.sleep(900);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
      return true;
    }
    return false;
  }
}
