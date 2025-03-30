// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import frc.robot.Configs;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax elevatorMotorLeft;
  private final SparkMax elevatorMotorRight;
  private final RelativeEncoder m_elevatorEncoderLeft;
  private final RelativeEncoder m_elevatorEncoderRight;
  // Creates a PIDController with gains kP, kI, and kD
  private SparkClosedLoopController m_elevatorControllerRight;
  
  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {

    elevatorMotorLeft = new SparkMax(ElevatorConstants.elevatorMotorLeftCanId, MotorType.kBrushless);
    elevatorMotorRight = new SparkMax(ElevatorConstants.elevatorMotorRightCanId, MotorType.kBrushless);
    
    elevatorMotorLeft.configure(Configs.ElevatorConfigs.elevatorFollowerConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    elevatorMotorRight.configure(Configs.ElevatorConfigs.elevatorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    
    m_elevatorEncoderLeft = elevatorMotorLeft.getEncoder();
    m_elevatorEncoderRight = elevatorMotorRight.getEncoder();

    // m_elevatorControllerLeft = elevatorMotorLeft.getClosedLoopController();
    m_elevatorControllerRight = elevatorMotorRight.getClosedLoopController();

    // m_absoluteEncoder.setPosition(0);
  }
    //probably should make this work based of a height variable in the future
  public void elevatorRotatePID(double position){
    double metersToAngle = (position * (ElevatorConstants.discGearRatio / ElevatorConstants.discCircumferenceMeter)) * 180;// 360/2 becasue 2:1 carrige ratio
    // m_elevatorControllerLeft.setReference(-metersToAngle, ControlType.kPosition);
    m_elevatorControllerRight.setReference(metersToAngle, ControlType.kPosition);
  }

  public double getElevatorHeight(){
    return (m_elevatorEncoderRight.getPosition() * (ElevatorConstants.discCircumferenceMeter / (180 * ElevatorConstants.discGearRatio)));
  }

  public void restartEncoder(){
    m_elevatorEncoderRight.setPosition(0);
    // printMotorPosition();
  }

  public void printMotorPosition(){
    double relativeEncoderMetersLeft = (m_elevatorEncoderLeft.getPosition() * (ElevatorConstants.discGearRatio / ElevatorConstants.discCircumferenceMeter)) * 180;
    double relativeEncoderMetersRight = (m_elevatorEncoderRight.getPosition() * (ElevatorConstants.discGearRatio / ElevatorConstants.discCircumferenceMeter)) * 180;
    SmartDashboard.putNumber("Left Motor", (relativeEncoderMetersLeft ));
    SmartDashboard.putNumber("Right Motor", (relativeEncoderMetersRight));
  }

  public void setElevatorSpeed(double speed){
    // elevatorMotorLeft.set(speed);
    elevatorMotorRight.set(-speed*4);
  }
  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
    // SmartDashboard.putNumber("Elevator Encoder", m_elevatorEncoderLeft.getPosition());
    // SmartDashboard.putNumber("Elevator Output", elevatorMotorRight.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void resetEncoders() {
    // m_absoluteEncoder.setPosition(0);
  }
}