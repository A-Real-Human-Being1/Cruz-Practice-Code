// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.FeetPerSecond;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import first.generated.CompTunerConstants;
import first.robot.Constants.DriveConstants;
import first.robot.Constants.Mode;
import first.robot.commands.drive.DriveCommands;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.GyroIOPigeon2;
import first.robot.subsystems.drive.ModuleIOTalonFX;
import first.robot.subsystems.vision.AprilTagVision;
import first.robot.subsystems.vision.CameraIO;
import first.robot.subsystems.vision.CameraIOLimelight4;
import java.util.Arrays;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;

public class RobotContainer {
  private final CommandGamepad gamepad_ = new CommandGamepad(0);

  private Drive drivebase_;
  private AprilTagVision vision_;

  public RobotContainer() {
    buildRobot();
    createDefaultSubsystems();

    configureBindings();
    configureDriveBindings();
  }

  private void configureBindings() {}

  private void configureDriveBindings() {
    DriveCommands.configure(
        drivebase_,
        () -> -gamepad_.getLeftY(),
        () -> -gamepad_.getLeftX(),
        () -> -gamepad_.getRightX());

    // Default command, normal field-relative drive
    drivebase_.setDefaultCommand(DriveCommands.joystickDrive().withName("JoystickDrive"));

    // Slow Mode, during left bumper
    gamepad_
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDrive(
                drivebase_,
                () -> -gamepad_.getLeftY() * DriveConstants.slowModeJoystickMultiplier,
                () -> -gamepad_.getLeftX() * DriveConstants.slowModeJoystickMultiplier,
                () -> -gamepad_.getRightX() * DriveConstants.slowModeJoystickMultiplier));

    // Switch to X pattern / brake while the West face button (X) is pressed
    gamepad_.faceLeft().whileTrue(drivebase_.stopWithXCmd());

    // Robot Relative
    gamepad_
        .dpadUp()
        .whileTrue(
            drivebase_.runVelocityCmd(
                FeetPerSecond.one(), MetersPerSecond.of(0), RadiansPerSecond.zero()));

    gamepad_
        .dpadDown()
        .whileTrue(
            drivebase_.runVelocityCmd(
                FeetPerSecond.one().unaryMinus(), MetersPerSecond.of(0), RadiansPerSecond.zero()));

    gamepad_
        .dpadLeft()
        .whileTrue(
            drivebase_.runVelocityCmd(
                MetersPerSecond.zero(), FeetPerSecond.one(), RadiansPerSecond.zero()));

    gamepad_
        .dpadRight()
        .whileTrue(
            drivebase_.runVelocityCmd(
                MetersPerSecond.zero(), FeetPerSecond.one().unaryMinus(), RadiansPerSecond.zero()));


    // Reset gyro to 0° when the North (Y) & East (B) face buttons are pressed
    gamepad_.faceUp().and(gamepad_.faceRight()).onTrue(drivebase_.resetGyroCmd());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private void buildRobot() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case SIMBOT:
          buildSimBot();
          break;

        case COMPETITION:
          buildComp();
          break;
      }
    } else {

    }
  }

  private void buildSimBot() {
    // TODO: build the drivebase and vision here using the simulation IO classes
    // from the SystemcoreTesting drivebase and vision code.
  }

  private void buildComp() {
    drivebase_ =
        new Drive(
            new GyroIOPigeon2(
                CompTunerConstants.DrivetrainConstants.Pigeon2Id, CompTunerConstants.kCANBus),
            ModuleIOTalonFX::new,
            CompTunerConstants.FrontLeft,
            CompTunerConstants.FrontRight,
            CompTunerConstants.BackLeft,
            CompTunerConstants.BackRight,
            CompTunerConstants.kCANBus,
            CompTunerConstants.kSpeedAt12Volts);

    vision_ =
        new AprilTagVision(
            drivebase_::addVisionMeasurement,
            new CameraIOLimelight4("limelight-front", drivebase_::getRotation),
            new CameraIOLimelight4("limelight-bl", drivebase_::getRotation),
            new CameraIOLimelight4("limelight-br", drivebase_::getRotation));
  }

  private void createDefaultSubsystems() {
    if (drivebase_ == null) {}

    if (vision_ == null) {
      int numCams =
          switch (Constants.getRobot()) {
            default -> 3;
          };

      CameraIO[] cams = new CameraIO[numCams];
      Arrays.setAll(cams, i -> new CameraIO() {});

      vision_ = new AprilTagVision(drivebase_::addVisionMeasurement, cams);
    }
  }
}
