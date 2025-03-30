// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToReefTagRelative;
import frc.robot.commands.ElevatorHeight;
import frc.robot.commands.L1;
import frc.robot.commands.L2;
import frc.robot.commands.L3;
import frc.robot.commands.L4;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterCommandInMan;
import frc.robot.commands.ShooterCommandOut;
import frc.robot.commands.ShooterCommandScore;
import frc.robot.commands.inTake;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_driverController1 =
      new CommandXboxController(1); 
  

  private double elevatorOffset = 0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    
    NamedCommands.registerCommand("L1", new L1(elevatorSubsystem));
    NamedCommands.registerCommand("L2", new L2(elevatorSubsystem));
    NamedCommands.registerCommand("L3", new L3(elevatorSubsystem));
    NamedCommands.registerCommand("L4", new L4(elevatorSubsystem));
    NamedCommands.registerCommand("Intake", new inTake(elevatorSubsystem));


    NamedCommands.registerCommand("Shooter", new ShooterCommandOut(shooterSubsystem));
    NamedCommands.registerCommand("ShooterScore", new ShooterCommandScore(shooterSubsystem));
    NamedCommands.registerCommand("ShooterIn", new ShooterCommand(shooterSubsystem));




    // CameraServer.startAutomaticCapture(1);
    // Thread m_visionThread =
    //     new Thread(
    //         () -> {
    //           // Get the UsbCamera from CameraServer
    //           UsbCamera camera = CameraServer.startAutomaticCapture();
    //           // Set the resolution-
    //           camera.setResolution(64, 48);
    //           camera.setFPS(5);

    //           // Get a CvSink. This will capture Mats from the camera
    //           CvSink cvSink = CameraServer.getVideo();
    //           // Setup a CvSource. This will send images back to the Dashboard
    //           CvSource outputStream = CameraServer.putVideo("0", 100, 60);

    //           // Mats are very memory expensive. Lets reuse this Mat.
    //           Mat mat = new Mat();

    //           // This cannot be 'true'. The program will never exit if it is. This
    //           // lets the robot stop this thread when restarting robot code or
    //           // deploying.
    //           while (!Thread.interrupted()) {
    //             // Tell the CvSink to grab a frame from the camera and put it
    //             // in the source mat.  If there is an error notify the output.
    //             if (cvSink.grabFrame(mat) == 0) {
    //               // Send the output the error.
    //               outputStream.notifyError(cvSink.getError());
    //               // skip the rest of the current iteration
    //               continue;
    //             }
    //             // Put a rectangle on the image
    //             Imgproc.rectangle(
    //                 mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
    //             // Give the output stream a new image to display
    //             outputStream.putFrame(mat);
    //           }
    //         });
    // m_visionThread.setDaemon(true);
    // m_visionThread.start();

    Thread m_visionThread1 =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution-
              camera.setResolution(64, 48);
              camera.setFPS(20);
          
  

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("0", 100, 60);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 1) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                // Put a rectangle on the image
                Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
    m_visionThread1.setDaemon(true);
    m_visionThread1.start();
    

  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), 
                                                                () -> m_driverController.getLeftY() * -1, 
                                                                () -> m_driverController.getLeftX() * -1)
                                                                .withControllerRotationAxis(m_driverController::getRightX)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .cubeTranslationControllerAxis(true)
                                                                .allianceRelativeControl(false);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX, 
                                                                                            m_driverController::getRightY)
                                                                                            .headingWhile(true);                                                              
  
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

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
    

    m_driverController.povDown().whileTrue(drivebase.gyroResetCommand());
    // m_driverController.y().onTrue(new FastSpeed(m_exampleSubsystem));
    // m_driverController.a().onTrue(new SlowSpeed(m_exampleSubsystem));

    //Elevator Controls 

    m_driverController1.a().onTrue(new ElevatorHeight(elevatorSubsystem, 0.09 + elevatorOffset)); //L1
    m_driverController1.b().onTrue(new ElevatorHeight(elevatorSubsystem, 0.25 + elevatorOffset)); //L2
    m_driverController1.x().onTrue(new ElevatorHeight(elevatorSubsystem, 0.75 + elevatorOffset)); //L3
    m_driverController1.y().onTrue(new ElevatorHeight(elevatorSubsystem, 1.50 + elevatorOffset)); //L4

    m_driverController1.a().and(m_driverController1.povUp()).onTrue(new ElevatorHeight(elevatorSubsystem, 0.10));
    m_driverController1.b().and(m_driverController1.povUp()).onTrue(new ElevatorHeight(elevatorSubsystem, 0.28));
    m_driverController1.x().and(m_driverController1.povUp()).onTrue(new ElevatorHeight(elevatorSubsystem, 0.79));
    m_driverController1.y().and(m_driverController1.povUp()).onTrue(new ElevatorHeight(elevatorSubsystem, 1.60));

    m_driverController1.a().and(m_driverController1.povDown()).onTrue(new ElevatorHeight(elevatorSubsystem, 0.08));
    m_driverController1.b().and(m_driverController1.povDown()).onTrue(new ElevatorHeight(elevatorSubsystem, 0.23));
    m_driverController1.x().and(m_driverController1.povDown()).onTrue(new ElevatorHeight(elevatorSubsystem, 0.70));
    m_driverController1.y().and(m_driverController1.povDown()).onTrue(new ElevatorHeight(elevatorSubsystem, 1.55));

    m_driverController1.leftBumper().onTrue(new inTake(elevatorSubsystem));
    m_driverController1.rightTrigger().whileTrue(new ShooterCommandOut(shooterSubsystem));
    m_driverController1.leftTrigger().onTrue(new ShooterCommand(shooterSubsystem));
    m_driverController1.rightBumper().whileTrue(new ShooterCommandInMan(shooterSubsystem));


    // m_driverController.povRight().onTrue(new AlignToReefTagRelative(true, drivebase).withTimeout(3));
		// m_driverController.povLeft().onTrue(new AlignToReefTagRelative(false, drivebase).withTimeout(3));
    // m_driverController1.povUp().onTrue(new ShooterCommandScore(shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Right");
  }
}
