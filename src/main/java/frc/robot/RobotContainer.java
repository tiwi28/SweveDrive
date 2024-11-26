// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Hashtable;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

//import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PieceType;
//import frc.robot.autos.*;
import frc.robot.commands.*;
// import frc.robot.commands.leds.Blink;
// import frc.robot.commands.leds.SetColor;
// import frc.robot.commands.presets.ConeHP;
// import frc.robot.commands.presets.ConeL1;
// import frc.robot.commands.presets.ConeL2;
// import frc.robot.commands.presets.ConeL2Score;
// import frc.robot.commands.presets.ConeL3;
// import frc.robot.commands.presets.ConeL3Score;
// import frc.robot.commands.presets.ConeShelf;
// import frc.robot.commands.presets.ConeStanding;
// import frc.robot.commands.presets.ConeTipped;
// import frc.robot.commands.presets.CubeHP;
// import frc.robot.commands.presets.CubeIntake;
// import frc.robot.commands.presets.CubeL1;
// import frc.robot.commands.presets.CubeL2;
// import frc.robot.commands.presets.CubeL3;
// import frc.robot.commands.presets.Rest;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.PoseEstimator;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(Constants.Operators.driver);

  /* Subsystems */
  final Swerve s_Swerve = new Swerve();
  final PoseEstimator poseEstimator = new PoseEstimator(s_Swerve);
  public RobotContainer() {

    // arm.setDefaultCommand(
    //     new ArmManualControl(arm, wrist, operator::getLeftY, operator::getRightY));

    // Configure the button bindings
    configureButtonBindings();
    configureAutoCommands();
    configureTestCommands();
    sendAutoCommands();
  }

  /**
   * Actions that we want to do when the robot is disabled.
   */
  public void disabledActions() {
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /* Driver Buttons */
    s_Swerve.setName("Drive");
    s_Swerve.setDefaultCommand(new TeleopSwerve(
        s_Swerve,
        () -> -driver.getLeftX(), //was getLeftY
        () -> -driver.getLeftY(), //wasGetleftX
        () -> -driver.getRightX(),
        () -> driver.leftBumper().getAsBoolean(),
        () -> driver.rightBumper().getAsBoolean(),
        () -> driver.y().getAsBoolean(),
        () -> driver.b().getAsBoolean(),
        () -> driver.a().getAsBoolean(),
        () -> driver.x().getAsBoolean()
        )
        );

    driver.back().onTrue(new InstantCommand(s_Swerve::zeroGyro));
    //driver.start().whileTrue(new Balance(s_Swerve, leds));
    driver.leftStick().whileTrue(new FancyRotation(s_Swerve));

    // driver.leftTrigger().whileTrue(new ParallelCommandGroup(
    //     new ConeTipped(arm, wrist, leds),
    //     new IntakeIn(arm, this.wrist, PieceType.CONE, leds)));

    // driver.rightTrigger().whileTrue(new ParallelCommandGroup(
    //     new CubeIntake(arm, wrist, leds),
    //     new IntakeIn(arm, this.wrist, PieceType.CUBE, leds)));
  }

  public void configureAutoCommands() {
    // this.autoCommands.put("L2 Link Farside", new FarsideL2Link(s_Swerve,
    // poseEstimator, wrist, arm, leds));
    // this.autoCommands.put("L2 2.5 Farside + Balance", new Farside25Balance(s_Swerve, poseEstimator, wrist, arm, leds));
    // this.autoCommands.put("Cable Side L2", new CableSideL2(s_Swerve, poseEstimator, arm, wrist, leds));
    // this.autoCommands.put("centerbalance", new CenterChargeStation(s_Swerve, poseEstimator, arm, wrist, leds));
    // this.autoCommands.put("2056 Link", new Link2056(s_Swerve, poseEstimator, wrist, arm, leds));
    // this.autoCommands.put("Cable Side L3", new CableSideL3(s_Swerve,
    // poseEstimator, arm, wrist, leds));

    // this.autoCommands.put("Two cone auto", new TwoConeAuto(s_Swerve,
    // poseEstimator, arm, wrist, leds));
    // this.autoCommands.put("Cone and Cube L2", new ConeAndCube(s_Swerve,
    //     poseEstimator, arm, wrist, leds));
    // this.autoCommands.put("Cone and Cube L3", new ConeAndCubeL3(s_Swerve,
    // poseEstimator, arm, wrist, leds));
    // this.autoCommands.put("L2 Cone Charge Station",
    // new L2ChargeStationCone(s_Swerve, poseEstimator, arm, wrist, leds));
    // this.autoCommands.put("L3 Cone Farside",
    // new FarsideConeL3(s_Swerve, poseEstimator, arm, wrist, leds));
    // this.autoCommands.put("L2 Cone Center Balance",
    // new CenterChargeStation(s_Swerve, poseEstimator, arm, wrist, leds));
    // this.autoCommands.put("Cone L2 Stationary", new StationaryConeL2(arm, wrist,
    //     leds));
    // this.autoCommands.put("Cone L3 Stationary", new StationaryConeL3(arm, wrist,
    //     leds));
    // this.autoCommands.put("Cube L2 Stationary", new StationaryCubeL2(arm, wrist,
    //     leds));
    // this.autoCommands.put("Cube L3 Stationary", new StationaryCubeL3(arm, wrist,
    //     leds));
  }

  public void configureTestCommands() {
    SmartDashboard.putData("Reset Pose Estimator", new InstantCommand(this.poseEstimator::resetFieldPosition));
    SmartDashboard.putData(poseEstimator);
    SmartDashboard.putData("Go to Position", new GoToPosition(s_Swerve, poseEstimator,
        new Transform3d(new Translation3d(FieldConstants.aprilTags.get(1).getX() - 0.5,
            FieldConstants.aprilTags.get(1).getY(), 0), new Rotation3d(0, 3.142, 0))));

    driver.povLeft().whileTrue(new InstantCommand(() -> {
      s_Swerve.retrieveAutoConstants();
    }));
  }

  public void sendAutoCommands() {
    SmartDashboard.putString("selectedAuto", "");
    JSONObject autoJSON = new JSONObject();

    // this.autoCommands.forEach((key, value) -> {
    //   JSONArray translation = new JSONArray();
    //   Pose2d initialHolonomicPose;

    //   if (value.getInitialHolonomicPose().getX() != 0) {
    //     initialHolonomicPose = value.pathGroup.get(0).getInitialHolonomicPose();
    //   } else {
    //     initialHolonomicPose = value.getInitialHolonomicPose();

    //   }
    //   translation.add(initialHolonomicPose.getX());
    //   translation.add(initialHolonomicPose.getY());

    //   autoJSON.put(key, translation);
    // });

    SmartDashboard.putString("autos", autoJSON.toJSONString());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   //@TODO remove void from function below

  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   String selectedAuto = SmartDashboard.getString("selectedAuto", "default");
  //   //if (selectedAuto.equals("default") || !this.autoCommands.containsKey(selectedAuto)) {
  //     // return new DefaultAuto(arm, wrist, leds).getCommand();
  //     //return new Tuning(s_Swerve, poseEstimator, arm, wrist, leds).getCommand();
  //  // }

  //   return this.autoCommands.get(selectedAuto).updateAutoBuilder().getCommand();
  // }
}
