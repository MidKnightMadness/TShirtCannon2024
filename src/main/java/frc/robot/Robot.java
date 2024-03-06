// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  //Terminology
  /*
   * Channel: the electrical port that the hardware goes into (either in computer or robot)
   * Axis: number that represents a control on the gamepad that isn't just on/off. Driver station shows which one is which (ex. x axis on joystick)
   * Solenoid: Thing that makes the t shirt cannon shoot. Time released determines power of shot
   */

  //joystick
  private PS4Controller gamepad;
  private static final int gamepadChannel = 0;
  private boolean armed  = false;

  //drivetrain
  private MecanumDrive robotDrive;

    TalonFX frontLeft = new TalonFX(frontLeftChannel);
    TalonFX rearLeft = new TalonFX(rearLeftChannel);
    TalonFX frontRight = new TalonFX(frontRightChannel);
    TalonFX rearRight = new TalonFX(rearRightChannel);

  double [] RPMMultipliers = {1.0, 1.0, 1.0, 1.0};

  public static final double [] FORWARD = {-1.0, 1.0, -1.0, -1.0};
  public static final double [] RIGHT = {-1.0, -1.0, 1.0, -1.0};
  public static final double [] CLOCKWISE = {-1.0, -1.0, -1.0, 1.0};
  public static final double POWER_MULTIPLIER = 1;

  private double [] motorInputs;

  private static final int frontLeftChannel = 15;
  private static final int rearLeftChannel = 0;
  private static final int frontRightChannel = 14;
  private static final int rearRightChannel = 1;
  private static final int driveAxisX = 0;
  private static final int driveAxisY = 1;
  private static final int driveAxisTurn = 4;

  //cannon angler
  /*
  private TalonFX cannonAngler;
  private static final int kCannonAnglerChannel = 13;
  private static final int kCannonAnglerAxis1 = 3;
  private static final int kCannonAnglerAxis2 = 2;
  private double m_cannonAnglerPos = 0;
  */

  //solenoid
  private Solenoid solenoidL;
  private Solenoid solenoidR;
  private int solenoidDuration = 3;
  private int LsolenoidCountdown = 0;
  private int RsolenoidCountdown = 0;
  private static final int solenoidLChannel = 0;
  private static final int solenoidRChannel = 1;


  @Override
  public void robotInit() {
    System.out.println("initializing...");

    //joystick
    gamepad = new PS4Controller(gamepadChannel);

    //mechanum
    TalonFX frontLeft = new TalonFX(frontLeftChannel);
    TalonFX rearLeft = new TalonFX(rearLeftChannel);
    TalonFX frontRight = new TalonFX(frontRightChannel);
    TalonFX rearRight = new TalonFX(rearRightChannel);

    robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    //cannon angler
    //cannonAngler = new TalonFX(kCannonAnglerChannel);
    //cannonAngler.config_kP(0, 0.2);

    //solenoid
    solenoidL = new Solenoid(PneumaticsModuleType.CTREPCM,solenoidLChannel);
    solenoidR = new Solenoid(PneumaticsModuleType.CTREPCM,solenoidRChannel);

    motorInputs = new double [4];

    System.out.println("initialized. Ready to run");
    CameraServer.startAutomaticCapture();
    
  }


  @Override
  public void teleopPeriodic() {
    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
    //robotDrive.driveCartesian(gamepad.getRawAxis(driveAxisX)/3, -gamepad.getRawAxis(driveAxisY)/3, -gamepad.getRawAxis(driveAxisTurn)/3);
    //robotDrive.drivePolar(LsolenoidCountdown, null, kDefaultPeriod);


    double xPad = gamepad.getRawAxis(driveAxisX)/3;
    double yPad = -gamepad.getRawAxis(driveAxisY)/3;
    double rotPad = -gamepad.getRawAxis(driveAxisTurn)/3;

    double maxPowerLevel = 0.0;
  
        // Linear combination of drive vectors
        for(int i = 0; i < 4; i++){
            motorInputs [i] = ((FORWARD [i] * yPad) + (RIGHT [i] * xPad) + (CLOCKWISE [i] * rotPad));
            motorInputs[i] *= POWER_MULTIPLIER * RPMMultipliers[i] * 1;

            if(Math.abs(motorInputs [i]) > maxPowerLevel){
                maxPowerLevel = Math.abs(motorInputs [i]);
            }
        }

        // Normalize power inputs within envelope, dead zone 0.2
        double powerEnvelope = Math.sqrt(motorInputs[0]*motorInputs[0] + motorInputs[1]*motorInputs[1] + motorInputs[2]*motorInputs[2] + motorInputs[3]*motorInputs[3]) / 2.0;
        if(powerEnvelope > 0.2 && maxPowerLevel > 1.0){
            for(int i = 0; i < 4; i++){
                motorInputs [i] /= maxPowerLevel;
            }
        }

        frontLeft.set( motorInputs [0]);
        frontRight.set( motorInputs [1]);
        rearLeft.set( motorInputs [2]);
        rearRight.set( motorInputs [3]);


    //Shoot from left and right barrels individually
    if(gamepad.getCircleButton()){
      armed = true;
      if(gamepad.getL1ButtonPressed()&&LsolenoidCountdown<=0){
        LsolenoidCountdown = solenoidDuration;
      }
      if(gamepad.getR1ButtonPressed()&&RsolenoidCountdown<=0){
        RsolenoidCountdown = solenoidDuration;
      }
    }else{
      armed = false;
    }

    //Adjust solenoid power
    if(gamepad.getTriangleButtonPressed()){
      solenoidDuration++;
    }
    if(gamepad.getCrossButtonPressed()){
      solenoidDuration--;
    }

    //Solenoid Control
    if(LsolenoidCountdown>0){
      solenoidL.set(true);
      LsolenoidCountdown--;
    }else{
      solenoidL.set(false);
    }
    if(RsolenoidCountdown>0){
      solenoidR.set(true);
      RsolenoidCountdown--;
    }else{
      solenoidR.set(false);
    }

    //Display telemetry
    SmartDashboard.putNumber("Power", solenoidDuration);
    SmartDashboard.putBoolean("Armed", armed);
    SmartDashboard.updateValues();

  }
}