// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ReefTargetMap;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.*;
/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final PassPlanner pP = new PassPlanner();
  public final ClientSubsystem clientSubsystem= new ClientSubsystem(drivetrain);
  private final CommandXboxController joystick1 = new CommandXboxController(0);

  // ==========================================
  // 🚀 可扩展的 Auto 战术枚举
  // ==========================================
  public enum AutoMode {
      kDoNothing,
      AutoReef,
      // 💡 以后有新的 Auto，直接在这里加名字！比如：
      // AutoSteal,
      // AutoMaxPoint
  }

  // ==========================================
  // 🚀 Auto Chooser & Preload Variables
  // ==========================================
  private final SendableChooser<AutoMode> m_autoChooser = new SendableChooser<>();
  private Command m_preloadedAuto = null;
  private double currentPlace = -1; // 0: 左侧, 1: 右侧
  private double previousPlace = -1;
  private boolean wasRed = false;   
  private AutoMode m_lastSelectedAuto = AutoMode.kDoNothing; // 侦测面板切换
  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
   private final double MaxAngularRate = edu.wpi.first.math.util.Units.rotationsToRadians(0.95);

   private final Telemetry logger = new Telemetry(MaxSpeed);
   private final SwerveRequest.FieldCentric driveClosed = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.04)
            .withRotationalDeadband(MaxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);
  /** The container for the robot. */
  public RobotContainer() {
    pP.setDrivetrain(drivetrain);
    drivetrain.setPassPlanner(pP);
    
    // 配置 UI 选择器
    m_autoChooser.setDefaultOption("Auto Reef (Data-Driven)", AutoMode.AutoReef);
    m_autoChooser.addOption("Do Nothing (Safe)", AutoMode.kDoNothing);
    // m_autoChooser.addOption("Steal Enemy Balls", AutoMode.AutoSteal); // 以后在这里加选项
    SmartDashboard.putData("Auto Mode", m_autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double rawVx = 0;
                double rawVy = 0;
                double rawRot = 0;
                
              
                    rawVx = -joystick1.getLeftY();
                    rawVy = -joystick1.getLeftX();
                    rawRot = -joystick1.getRightX() * MaxAngularRate * OperatorConstants.AngularSpeedRate;
                
                
                double vXCmd = rawVx * MaxSpeed * OperatorConstants.SpeedRate;
                double vYCmd = rawVy * MaxSpeed * OperatorConstants.SpeedRate;
                
                
                return driveClosed
                        .withVelocityX(vXCmd)
                        .withVelocityY(vYCmd)
                        .withRotationalRate(rawRot);
            })
        );
    drivetrain.registerTelemetry(state -> logger.telemeterize(state));
    joystick1.leftBumper().onTrue(
            drivetrain.passPlanner.changeReefID(1)
        );
        joystick1.rightBumper().onTrue(
            drivetrain.passPlanner.changeReefID(-1)
        );
    joystick1.leftTrigger().whileTrue(
      Commands.either(
        Commands.sequence(
            new AutoMoveCircle(drivetrain, 1.9, 3, false).onlyIf(() -> Math.abs(drivetrain.passPlanner.calcShortestReefPath()) > 1),
            drivetrain.passPlanner.gDPCommand(1, 3, 0.7).onlyIf(() -> Math.abs(drivetrain.passPlanner.calcShortestReefPath()) > 0),
            new AutoMoveComplex(drivetrain, null, null, null, null, false, false, false, 6, 0.1, 0.1).onlyIf(() -> Math.abs(drivetrain.passPlanner.calcShortestReefPath()) > 0),
            new AutoMoveClosed(drivetrain, () -> 1, 2.8)
        ),
        Commands.sequence(
            Commands.runOnce(()->{
              Pose2d endPose=new Pose2d(ReefTargetMap.BLUE_ALGAE_PREP_X[drivetrain.passPlanner.targetReefID], ReefTargetMap.BLUE_ALGAE_PREP_Y[drivetrain.passPlanner.targetReefID],Rotation2d.fromDegrees(drivetrain.passPlanner.getFaceCenterAngle(drivetrain.passPlanner.targetReefID)-180));
              drivetrain.passPlanner.generateUniversalBezier(null, endPose, drivetrain.passPlanner.getFaceCenterAngle(drivetrain.passPlanner.targetReefID)-180, 3.2, 0.7, 5, true, 0.6, 0.6, true, true);
            }),
            new AutoMoveComplex(drivetrain, null, null, null, null, false, false, false, 6, 0.1, 0.1),
            new AutoMoveClosed(drivetrain, () -> 1, 2.8)
          ),
        ()->{return drivetrain.passPlanner.calcShortestReefPath()!=0;}
      
      ));

    joystick1.rightTrigger().whileTrue(
      Commands.either(
      Commands.sequence(
            new AutoMoveCircle(drivetrain, 1.9, 3, false).onlyIf(() -> Math.abs(drivetrain.passPlanner.calcShortestReefPath()) > 1),
            drivetrain.passPlanner.gDPCommand(0, 3, 0.7).onlyIf(() -> Math.abs(drivetrain.passPlanner.calcShortestReefPath()) > 0),
            new AutoMoveComplex(drivetrain, null, null, null, null, false, false, false, 6, 0.1, 0.1).onlyIf(() -> Math.abs(drivetrain.passPlanner.calcShortestReefPath()) > 0),
            new AutoMoveClosed(drivetrain, () -> 0, 2.8)
        ),
      Commands.sequence(
            Commands.runOnce(()->{
              Pose2d endPose=new Pose2d(ReefTargetMap.BLUE_ALGAE_PREP_X[drivetrain.passPlanner.targetReefID], ReefTargetMap.BLUE_ALGAE_PREP_Y[drivetrain.passPlanner.targetReefID],Rotation2d.fromDegrees(drivetrain.passPlanner.getFaceCenterAngle(drivetrain.passPlanner.targetReefID)-180));
              drivetrain.passPlanner.generateUniversalBezier(null, endPose, drivetrain.passPlanner.getFaceCenterAngle(drivetrain.passPlanner.targetReefID)-180, 2.8, 0.7, 5, true, 0.6, 0.6, true, true);
            }),
            new AutoMoveComplex(drivetrain, null, null, null, null, false, false, false, 6, 0.1, 0.1),
            new AutoMoveClosed(drivetrain, () -> 0, 2.8)
          ),
        ()->{return drivetrain.passPlanner.calcShortestReefPath()!=0;}));

    joystick1.x().whileTrue(Commands.sequence(
            Commands.runOnce(()->{drivetrain.passPlanner.supplyProxyId=1;}),
            new AutoMoveCircle(drivetrain, 2, 3, false).onlyIf(()->{return drivetrain.passPlanner.calcShortestReefPath(1)>1 || drivetrain.passPlanner.calcShortestReefPath(1)<-1;}),
            drivetrain.passPlanner.CreatePathToLeftSupply(),
            new AutoMoveComplex(drivetrain, null, null, null, null, true, false, false, 4, 0.1, 0.1)
        ));
    joystick1.b().whileTrue(Commands.sequence(
            Commands.runOnce(()->{drivetrain.passPlanner.supplyProxyId=5;}),
            new AutoMoveCircle(drivetrain, 2, 3, false).onlyIf(()->{return drivetrain.passPlanner.calcShortestReefPath(5)>1 || drivetrain.passPlanner.calcShortestReefPath(5)<-1;}),
            drivetrain.passPlanner.CreatePathToRightSupply(),
            new AutoMoveComplex(drivetrain, null, null, null, null, true, false, false, 4, 0.1, 0.1)
        ));
    joystick1.a().whileTrue(Commands.sequence(
            new AutoMoveCircle(drivetrain, 1.9, 3, false).onlyIf(()->{return Math.abs(drivetrain.passPlanner.calcShortestReefPath())>1;}),
            drivetrain.passPlanner.gDPCommand(0, 3, 2).onlyIf(()->{return Math.abs(drivetrain.passPlanner.calcShortestReefPath())>0;}),
            new AutoMoveComplex(drivetrain, null, null, null, null, false, false, false, 6, 0.1, 0.1).until(()->{return drivetrain.passPlanner.calcShortestReefPath()==0;}).onlyIf(()->{return Math.abs(drivetrain.passPlanner.calcShortestReefPath())>0;}),
            Commands.defer(()->{
                boolean isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;  
                return Commands.sequence(
                    new AutoMoveClosed(drivetrain, ReefTargetMap.BLUE_ALGAE_PREP_X[drivetrain.passPlanner.targetReefID], ReefTargetMap.BLUE_ALGAE_PREP_Y[drivetrain.passPlanner.targetReefID], drivetrain.passPlanner.getFaceCenterAngle(drivetrain.passPlanner.targetReefID)-180, 2.4, isRed, isRed),
                    new AutoMoveClosed(drivetrain, ReefTargetMap.BLUE_ALGAE_X[drivetrain.passPlanner.targetReefID], ReefTargetMap.BLUE_ALGAE_Y[drivetrain.passPlanner.targetReefID], drivetrain.passPlanner.getFaceCenterAngle(drivetrain.passPlanner.targetReefID)-180, 1.5, isRed, isRed)
                );
            }, Set.of(drivetrain))
        ));
  }

  // ==========================================
  // 🏭 自动阶段工厂 (Auto Factory)
  // ==========================================
  /**
   * 根据传入的战术模式和镜像参数，分发给 Autos.java 里的不同方法
   */
  private Command generateAutoCommand(AutoMode mode, boolean invertA, boolean invertD) {
      switch (mode) {
          case AutoReef:
              
              return Autos.autoReef(drivetrain, invertA, invertD);

          case kDoNothing:
          default:
              return Commands.none();
      }
  }

  // ==========================================
  // 🚀 自动阶段智能预加载逻辑
  // ==========================================
  
  public void refreshAutoMode() {
      // 1. 获取当前 UI 选中的模式
      AutoMode currentSelection = m_autoChooser.getSelected();
      if (currentSelection == null) {
        currentSelection = AutoMode.kDoNothing;
      }
      // 2. 获取联盟颜色 (决定 invertA)
      Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
      boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

      // 3. 获取摆放位置 (决定 invertD)
      double y = drivetrain.getState().Pose.getY();
      currentPlace = (y >= OperatorConstants.fieldWidth / 2.0) ? 0 : 1;

      // 4. 如果发生了任何状态变化（被搬动、改了颜色、切了战术面板） -> 重新生成！
      if (currentPlace != previousPlace || isRed != wasRed || currentSelection != m_lastSelectedAuto || m_preloadedAuto == null) {
          SmartDashboard.putString("Auto Status", "GENERATING...");

          boolean invertA = isRed;
          boolean invertD = (currentPlace == 1); 

          // 传入工厂生成指令
          m_preloadedAuto = generateAutoCommand(currentSelection, invertA, invertD);

          SmartDashboard.putString("Auto Status", "READY!");
          previousPlace = currentPlace;
          wasRed = isRed;
          m_lastSelectedAuto = currentSelection;
      }
  }

  public Command getAutonomousCommand() {
      if (m_preloadedAuto != null) {
          Command autoCmd = m_preloadedAuto;
          m_preloadedAuto = null; 
          return autoCmd; 
      }
      return Commands.none();
  }
}