// Copyright (c) 2026 FRC 1425 Error Code Xero
// http://github.com/errorcodexero
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.FeetPerSecond;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.limelightvision.Limelight;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;
import first.generated.CompTunerConstants;
import first.robot.Constants.DriveConstants;
import first.robot.commands.drive.DriveCommands;
import first.robot.subsystems.colorSensor.ColorSensor;
import first.robot.subsystems.colorSensor.ColorSensorIOHardware;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.GyroIOPigeon2;
import first.robot.subsystems.drive.ModuleIOReplay;
import first.robot.subsystems.drive.ModuleIOTalonFX;
import first.robot.subsystems.motor.Motor;
import first.robot.subsystems.motor.MotorIO;
import first.robot.subsystems.motor.MotorIOHardware;
import first.robot.subsystems.vision.AprilTagVision;
import first.robot.subsystems.vision.CameraIO;
import first.robot.subsystems.vision.CameraIOLimelight;
import first.robot.subsystems.vision.CameraIOLimelight4;
import java.io.IOException;
import java.util.List;
import org.json.simple.parser.ParseException;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Rotation2d;

public class RobotContainer {

  private Drive drive;
  private AprilTagVision vision;
  private Motor motor;
  private ColorSensor colorSensor;

  private final CommandXboxController gamepad = new CommandXboxController(0);

  public RobotContainer() {
    switch (Constants.getRobot()) {
      case COMPETITION:
        drive =
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

        motor = new Motor(new MotorIOHardware(4, new CANBus(CANPort.CAN_S4)));

        colorSensor = new ColorSensor(new ColorSensorIOHardware());

        vision =
            new AprilTagVision(
                drive::addVisionMeasurement,
                new CameraIOLimelight4("limelight", drive::getRotation),
                new CameraIOLimelight(Limelight.SYSTEMCORE_USB1, drive::getRotation));
        break;
      case SIMBOT:
        drive =
            new Drive(
                new GyroIO() {},
                ModuleIOReplay::new,
                CompTunerConstants.FrontLeft,
                CompTunerConstants.FrontRight,
                CompTunerConstants.BackLeft,
                CompTunerConstants.BackRight,
                CompTunerConstants.kCANBus,
                CompTunerConstants.kSpeedAt12Volts);
        break;
    }

    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              ModuleIOReplay::new,
              CompTunerConstants.FrontLeft,
              CompTunerConstants.FrontRight,
              CompTunerConstants.BackLeft,
              CompTunerConstants.BackRight,
              CompTunerConstants.kCANBus,
              CompTunerConstants.kSpeedAt12Volts);
    }

    if (motor == null) {
      motor = new Motor(new MotorIO() {});
    }

    if (colorSensor == null) {
      colorSensor = new ColorSensor(new ColorSensorIOHardware() {});
    }

    if (vision == null) {
      vision = new AprilTagVision(drive::addVisionMeasurement, new CameraIO() {});
    }

    DriveCommands.configure(
        drive, () -> -gamepad.getLeftY(), () -> -gamepad.getLeftX(), () -> -gamepad.getRightX());

    configureBindings();
    configureDriveBindings();
  }

  // Bind robot actions to commands here.
  private void configureBindings() {
    gamepad.rightTrigger().and(gamepad.leftTrigger()).whileTrue(motor.runVoltage(Volts.of(6.0)));
  }

  private void configureDriveBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(DriveCommands.joystickDrive().withName("JoystickDrive"));

    // Slow Mode, during left bumper
    gamepad
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -gamepad.getLeftY() * DriveConstants.slowModeJoystickMultiplier,
                () -> -gamepad.getLeftX() * DriveConstants.slowModeJoystickMultiplier,
                () -> -gamepad.getRightX() * DriveConstants.slowModeJoystickMultiplier));

    // Switch to X pattern / brake while X button is pressed
    gamepad.x().whileTrue(drive.stopWithXCmd());

    // Robot Relative
    gamepad
        .dpadUp()
        .whileTrue(
            drive.runVelocityCmd(
                FeetPerSecond.one(), MetersPerSecond.of(0), RadiansPerSecond.zero()));

    gamepad
        .dpadDown()
        .whileTrue(
            drive.runVelocityCmd(
                FeetPerSecond.one().unaryMinus(), MetersPerSecond.of(0), RadiansPerSecond.zero()));

    gamepad
        .dpadLeft()
        .whileTrue(
            drive.runVelocityCmd(
                MetersPerSecond.zero(), FeetPerSecond.one(), RadiansPerSecond.zero()));

    gamepad
        .dpadRight()
        .whileTrue(
            drive.runVelocityCmd(
                MetersPerSecond.zero(), FeetPerSecond.one().unaryMinus(), RadiansPerSecond.zero()));

    // Reset gyro to 0° when Y & B button is pressed
    gamepad.y().and(gamepad.b()).onTrue(drive.resetGyroCmd());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Command followTestPath() throws FileVersionException, IOException, ParseException {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath testPath = PathPlannerPath.fromPathFile("testPath");
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(testPath);
  }

  /**
   * <p>waypoints is a list of the following format: The rotation component of the pose should be the
   * direction of travel. Do not use holonomic rotation. 
   * List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses( 
   * new Pose2d(x, y, rotation),
   * new Pose2d(x, y, rotation)
   * );
   *
   * <p>endRotation should be a rotation2d of your intended ending rotation
   *
   * <p>endVelo is ending velocity in meters per second!!!
   *
   * @param waypoints
   * @param endRotation
   * @param endVelo
   * @return
   */
  public Command makeAndFollowPath(
      List<Waypoint> waypoints, double endVelo, Rotation2d endRotation) {

    // Create the path using the waypoints passed in
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            PathConstraints.unlimitedConstraints(12.0), // no constraints
            null, // irrelavant, so its null
            new GoalEndState(endVelo, endRotation) // Goal end state. assumed you want to stop
            );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }
}
