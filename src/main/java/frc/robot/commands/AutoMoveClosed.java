// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ReefTargetMap; // 请根据你的实际路径调整
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoMoveClosed extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;

    private final double targetVelocity;
    private double x;
    private double y;
    private double d; // Heading (degrees)
    private boolean invertA;
    private boolean invertD;

    private double realHead = 0.0;
    private Pose2d m_targetWaypoint = new Pose2d();

    // 🚨 恢复你的双轴独立 PID 控制器
    private final PIDController m_MovePIDx = new PIDController(5., 0.1, 0.1);
    private final PIDController m_MovePIDy = new PIDController(5., 0.1, 0.1);
    
    private final SwerveRequest.FieldCentricFacingAngle driveClosed = new SwerveRequest.FieldCentricFacingAngle();

    // 容差常量
    private static final double kTranslationTolerance = 0.05; // 允许 10 厘米误差
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
    
    // 动态对位专用标志位与晚绑定变量
    private boolean aimAtReef = false;
    private Supplier<Integer> m_branchSupplier;

    // ==========================================
    // 🏗️ 原有 Constructor (静态指令模式)
    // ==========================================
    public AutoMoveClosed(CommandSwerveDrivetrain drivetrain, double targetX, double targetY, 
                          double targetHeading, double targetVel, boolean invert1, boolean invert2) {
        this.m_drivetrain = drivetrain;
        this.x = targetX;
        this.y = targetY;
        this.d = targetHeading;
        this.targetVelocity = targetVel;
        this.invertA = invert1;
        this.invertD = invert2;
        this.aimAtReef = false;

        driveClosed.HeadingController.setPID(8.0, 0.0, 0.1);
        driveClosed.withDeadband(MaxSpeed * 0.05)
                   .withRotationalDeadband(0.1)
                   .withDriveRequestType(DriveRequestType.Velocity)
                   .withSteerRequestType(SteerRequestType.Position);

        addRequirements(m_drivetrain);
    }

    // ==========================================
    // 🚀 新增 Constructor (动态 Reef 瞄准模式)
    // ==========================================
    public AutoMoveClosed(CommandSwerveDrivetrain drivetrain, Supplier<Integer> branchSupplier, 
                          double targetVel) {
        this.m_drivetrain = drivetrain;
        this.m_branchSupplier = branchSupplier;
        this.targetVelocity = targetVel;
        this.invertA = false;
        this.invertD = false;
        this.aimAtReef = true; 

        driveClosed.HeadingController.setPID(8.0, 0.0, 0.1);
        driveClosed.withDeadband(MaxSpeed * 0.05)
                   .withRotationalDeadband(0.1)
                   .withDriveRequestType(DriveRequestType.Velocity)
                   .withSteerRequestType(SteerRequestType.Position);

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        if (aimAtReef) {
            int branchIndex = m_branchSupplier.get();
            int currentTargetReefID = m_drivetrain.passPlanner.targetReefID;

            // 提取蓝方绝对坐标
            this.x = ReefTargetMap.BLUE_REEF_X[currentTargetReefID][branchIndex];
            this.y = ReefTargetMap.BLUE_REEF_Y[currentTargetReefID][branchIndex];
            this.d = getFaceCenterAngle(currentTargetReefID) + 180.0;
            
            // 将翻转标志位激活，交给下方的逻辑统一处理
            invertA = isRed;
            invertD = isRed;
        }

        // ==========================================
        // 🟥 统一红方坐标系镜像折叠
        // ==========================================
        if (invertD) {
            this.d = 0.0 - this.d;
            this.y = OperatorConstants.fieldWidth- this.y;
        }
        if (invertA) {
            this.d = 180.0 - this.d;
            while (this.d > 180.0) this.d -= 360.0;
            while (this.d < -180.0) this.d += 360.0;
            this.x = OperatorConstants.fieldHeight - this.x;
        }
        
        realHead = this.d;
        
        // 🚨 抵消 Phoenix 6 角度透视
        if (isRed) {
            realHead -= 180.0;
        }

        m_targetWaypoint = new Pose2d(this.x, this.y, Rotation2d.fromDegrees(realHead));
        
        m_MovePIDx.reset();
        m_MovePIDy.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_drivetrain.getState().Pose;
        
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        
        // 🎯 1. 独立双轴 PID 计算速度
        double speedX = m_MovePIDx.calculate(currentX, this.x);
        double speedY = m_MovePIDy.calculate(currentY, this.y);
        
        // 🎯 2. 合成速度向量模长
        double speedT = Math.hypot(speedX, speedY);
        
        // 🎯 3. Clamp 限速处理
        if (speedT >= targetVelocity) {
            double coeff = speedT / targetVelocity;
            speedX /= coeff;
            speedY /= coeff;
        }

        // 🚨 4. 抵消 Phoenix 6 底层速度透视反转
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            speedX = -speedX;
            speedY = -speedY;
        }

        // 5. 目标朝向判定
        double targetRot = m_drivetrain.autoShooting ? m_drivetrain.autoRot : realHead;

        m_drivetrain.autoSpeedX = speedX;
        m_drivetrain.autoSpeedY = speedY;

        m_drivetrain.setControl(
            driveClosed.withVelocityX(speedX)
                       .withVelocityY(speedY)
                       .withTargetDirection(Rotation2d.fromDegrees(targetRot))
        );
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = m_drivetrain.getState().Pose;
        double distance = currentPose.getTranslation().getDistance(m_targetWaypoint.getTranslation());
        return distance <= kTranslationTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(m_drivetrain.Idle);
    }

    private double getFaceCenterAngle(int faceID) {
        switch (faceID) {
            case 0: return 180.0;
            case 1: return 120.0;
            case 2: return 60.0;
            case 3: return 0.0;
            case 4: return 300.0;
            case 5: return 240.0;
            default: return 0.0;
        }
    }
}