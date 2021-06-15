/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Autonomous
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Cameras
  private UsbCamera camera1;
  private UsbCamera camera2;

  // Victor
  private WPI_VictorSPX uptakeOne;
  private WPI_VictorSPX shooter;
  
  // Talon
  // hood is a talon, but uses Motor Postion class
  private WPI_TalonSRX leftMotor;
  private WPI_TalonSRX rightMotor;
  private WPI_TalonSRX pigeonTalon;


  // Spark
  private Spark intake;
  private Spark uptakeTwo;
  private Spark uptakeThree;
  private Spark drawBridge;

  // WPI
  private DifferentialDrive chassis;
  private DigitalInput drawBridgeDown;

  // primitives
  private boolean index;
  private double indexStart;

  // gamepad
  private Joystick gamepad;
  private boolean aButton;
  private boolean bButton;
  private boolean xButton;
  private boolean yButton;
  private boolean leftBumper;
  private boolean rightBumper;
  private double leftXAxis; // left negative, right positive
  private double leftYAxis; // up negative, down positive
  private double leftTrigger; // no negative
  private double rightTrigger; // no negative
  private double rightXAxis; // left negative, right positive
  private double rightYAxis; // up negative, down positive
  private double leftXAxisWDeadzone;
  private double leftYAxisWDeadzone;
  private double rightXAxisWDeadzone;
  private double rightYAxisWDeadzone;

  // PID Motor ControlMode
  private MotorPosition hood;

  // Odemetry work
  //DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(23));
  // other
  private PigeonIMU pigeon;

  @Override
  public void robotInit() {
    // Autonomous
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Cameras
    camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    camera2 = CameraServer.getInstance().startAutomaticCapture(1);

    // Gamepad
    gamepad = new Joystick(0);

    // Victors
    uptakeOne = new WPI_VictorSPX(0);
    shooter = new WPI_VictorSPX(1);
    
    // Talon
    leftMotor = new WPI_TalonSRX(2);
    rightMotor = new WPI_TalonSRX(3);

    pigeonTalon = new WPI_TalonSRX(10);

    // Spark
    intake = new Spark(0);
    uptakeTwo = new Spark(1);
    uptakeThree = new Spark(2);
    drawBridge = new Spark(3);

    // WPI
    chassis = new DifferentialDrive(leftMotor, rightMotor);
    drawBridgeDown = new DigitalInput(0);

    // primitives

    // gamepad

    // odometry

    // other
    pigeon = new PigeonIMU(pigeonTalon);
    pigeon.setYaw(0.0);

    //PID Controls
    hood = new MotorPosition(3);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // lower drawbridge until down
      do {
        drawBridge.set(-1);
      } while (!drawBridgeDown.get());
      // STOP WHEN DOWN
      drawBridge.set(0);

      break;
    case kDefaultAuto:
    default:
      break;
    }

  }

  @Override
  public void teleopPeriodic() {
   // if(!hood.getSetUp())
    //hood.setStart();
    // Pigeon
    //getAndSendPigeonStatus();

    // Odometry

    //odometry



    // gamepad & motors
    updateGamepadStatus();
    drawbridge(yButton, xButton); // drawbridge up, drawbridge down
    indexBall(bButton, aButton, rightBumper, leftBumper, leftTrigger); // startIndex, intake, uptakeTwo, uptakeThree,
                                                                       // backout
    shooter(rightTrigger, 1); // analogControl, divider
    //hood.move(rightXAxisWDeadzone, .25); // analogControl, divider
    arcadeDrive(leftYAxisWDeadzone, leftXAxisWDeadzone, .5); // drive, rotate, divider
  }

  private void arcadeDrive(double drive, double rotate, double divider) {
    chassis.arcadeDrive(drive * divider, rotate * divider);
  }

  private void shooter(double analogControl, double divider) {
    shooter.set(ControlMode.PercentOutput, analogControl * divider);
  }

  public void indexBall(boolean startIndexButton, boolean intakeButton, boolean manualUptakeTwoButton,
      boolean manualUptakeThreeButton, double analogBackoutButton) {
    // intake, part of indexing, but not controlled by timer.
    if (intakeButton) {
      uptakeOne.set(ControlMode.PercentOutput, 1);
      intake.set(-.25);
    } else {
      uptakeOne.set(ControlMode.PercentOutput, 0);
      intake.set(0);
    }

    // if you press the index button, and not currently index, start indexing and
    // set the initial time to now.
    if (!index && startIndexButton) {
      index = true;
      indexStart = Timer.getFPGATimestamp();
    }

    // uptakeTwo
    if (index || manualUptakeTwoButton) {
      uptakeTwo.set(-1);
    } else {
      uptakeTwo.set(-1 * analogBackoutButton);
    }

    // UptakeThree
    if (index || manualUptakeThreeButton) {
      uptakeThree.set(-1);
    } else {
      uptakeThree.set(analogBackoutButton);
    }

    // stop index if time is up
    double curTime = Timer.getFPGATimestamp();
    if (curTime > indexStart + 4 && index) {
      index = false;
    }
  }

  private void drawbridge(boolean upButton, boolean downButton) {
    if (upButton) {
      drawBridge.set(1);
    } else if (downButton && drawBridgeDown.get()) {
      drawBridge.set(-1);
    } else {
      drawBridge.set(0);
    }
  }

  private void getAndSendPigeonStatus() {
    // setting up values
    double[] ypr = new double[3];
    double[] accelxyz = new double[3];
    double[] gyroxyz = new double[3];
    double compass;
    // setting values
    pigeon.getYawPitchRoll(ypr);
    pigeon.getAccelerometerAngles(accelxyz);
    pigeon.getRawGyro(gyroxyz);
    compass = pigeon.getCompassFieldStrength();

    // send values to SmartDashboard, both as an array and values
    // ypr
    SmartDashboard.putNumberArray("Yaw Pitch Roll", ypr);
    SmartDashboard.putNumber("Yaw", ypr[0]);
    SmartDashboard.putNumber("Pitch", ypr[1]);
    SmartDashboard.putNumber("Roll", ypr[2]);
    // accel
    SmartDashboard.putNumberArray("Accelerometer", accelxyz);
    SmartDashboard.putNumber("X Acceleration", accelxyz[0]);
    SmartDashboard.putNumber("Y Acceleration", accelxyz[1]);
    SmartDashboard.putNumber("Z Acceleration", accelxyz[2]);
    // gyro
    SmartDashboard.putNumberArray("Gyroscope", gyroxyz);
    SmartDashboard.putNumber("X Rotation", gyroxyz[0]);
    SmartDashboard.putNumber("Y Rotation", gyroxyz[1]);
    SmartDashboard.putNumber("Z Rotation", gyroxyz[2]);
    // compass
    SmartDashboard.putNumber("Compass", compass);
  }

  private void updateGamepadStatus() {
    aButton = gamepad.getRawButton(1);
    bButton = gamepad.getRawButton(2);
    xButton = gamepad.getRawButton(3);
    yButton = gamepad.getRawButton(4);
    leftBumper = gamepad.getRawButton(5);
    rightBumper = gamepad.getRawButton(6);
    leftXAxis = gamepad.getRawAxis(0); // left negitive, right positive
    leftYAxis = gamepad.getRawAxis(1); // up negitive, down positive
    leftTrigger = gamepad.getRawAxis(2); // no negitive
    rightTrigger = gamepad.getRawAxis(3); // no negitive
    rightXAxis = gamepad.getRawAxis(4); // left negitive, right positive
    rightYAxis = gamepad.getRawAxis(5); // up negitive, down positive

    setDeadzoneValues();
  }

  private void setDeadzoneValues() {
    if (leftXAxis < .1 && leftXAxis > -.1) {
      leftXAxisWDeadzone = 0;
    } else {
      leftXAxisWDeadzone = leftXAxis;
    }
    if (leftYAxis < .1 && leftYAxis > -.1) {
      leftYAxisWDeadzone = 0;
    } else {
      leftYAxisWDeadzone = leftYAxis;
    }
    if (rightXAxis < .1 && rightXAxis > -.1) {
      rightXAxisWDeadzone = 0;
    } else {
      rightXAxisWDeadzone = rightXAxis;
    }
    if (rightYAxis < .1 && rightYAxis > -.1) {
      rightYAxisWDeadzone = 0;
    } else {
      rightYAxisWDeadzone = rightYAxis;
    }
  }

  public Rotation2d pigeonGetHeading() {
    double[] gyro = new double[2];
    pigeon.getRawGyro(gyro);
    return Rotation2d.fromDegrees(gyro[2]);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
