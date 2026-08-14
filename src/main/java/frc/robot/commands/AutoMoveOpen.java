// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoMoveOpen extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;

    private final double targetVelocity;
    private double x;
    private double y;
    private double d; // Heading (degrees)
    private final boolean invertA;
    private final boolean invertD;
    private final double endSpeed;

    private double targetRot = 0.0;
    private double initialDis = 0.0;
    private Pose2d m_targetWaypoint = new Pose2d();
    private double angleToTarget = 0.0;

    private final PIDController m_PID = new PIDController(3.0, 0.0, 0.1);
    private final SwerveRequest.FieldCentricFacingAngle driveClosed = new SwerveRequest.FieldCentricFacingAngle();

    // 容差常量 (到达多近算作“完成”)
    private static final double kTranslationTolerance = 0.1; // 允许 10 厘米的误差
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);

    /**
     * @param drivetrain 你的底盘子系统指针
     * @param targetX 目标点 X
     * @param targetY 目标点 Y
     * @param targetHeading 期望的朝向角
     * @param targetVel 目标速度
     */
    public AutoMoveOpen(CommandSwerveDrivetrain drivetrain, double targetX, double targetY, 
                        double targetHeading, double targetVel, boolean invert1, boolean invert2) {
        this(drivetrain, targetX, targetY, targetHeading, targetVel, invert1, invert2, targetVel);
    }

    public AutoMoveOpen(CommandSwerveDrivetrain drivetrain, double targetX, double targetY, 
                        double targetHeading, double targetVel, boolean invert1, boolean invert2, double endSpeed) {
        this.m_drivetrain = drivetrain;
        this.targetVelocity = targetVel;
        this.x = targetX;
        this.y = targetY;
        this.d = targetHeading;
        this.invertA = invert1;
        this.invertD = invert2;
        this.endSpeed = endSpeed;

        driveClosed.HeadingController.setPID(8.0, 0.0, 0.1);
        driveClosed.withDeadband(MaxSpeed * 0.05)
                   .withRotationalDeadband(0.1)
                   // .withMaxAbsRotationalRate(3.14 * 1.5) // (如果在你的 Phoenix 版本中支持)
                   .withDriveRequestType(DriveRequestType.Velocity)
                   .withSteerRequestType(SteerRequestType.Position);

        // 声明底盘依赖，防止指令冲突
        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        if (invertD) {
            d = 0.0 - d;
            y = OperatorConstants.fieldWidth - y;
        }
        if (invertA) {
            x = OperatorConstants.fieldHeight - x;
            d = 180.0 - d;
            while (d > 180.0) d -= 360.0;
            while (d < -180.0) d += 360.0;
        }

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        double realHead = d;
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            realHead -= 180.0;
        }

        // 指令开始时重置 PID 状态
        m_PID.reset();
        
        m_targetWaypoint = new Pose2d(x, y, Rotation2d.fromDegrees(realHead));
        Translation2d nowTrans = m_drivetrain.getState().Pose.getTranslation();
        
        // 转换到360坐标系
        angleToTarget = m_targetWaypoint.getTranslation().minus(nowTrans).getAngle().getDegrees();
        if (angleToTarget < 0) {
            angleToTarget += 360.0;
        }

        Pose2d currentPose = m_drivetrain.getState().Pose;
        initialDis = currentPose.getTranslation().getDistance(m_targetWaypoint.getTranslation());
    }

    @Override
    public void execute() {
        // 1. 获取机器人当前位置
        Pose2d currentPose = m_drivetrain.getState().Pose; 
        
        // 2. 计算当前位置到目标点的相对向量 (Translation2d)
        Translation2d difference = m_targetWaypoint.getTranslation().minus(currentPose.getTranslation());
        
        // 3. 极其优雅：直接获取这个向量的绝对朝向角 (Rotation2d)
        Rotation2d angleTo = difference.getAngle();
        
        // 4. 直接把标量速度沿着这个直线角度分解为 X 和 Y 的分量
        // Rotation2d 自带 getCos() 和 getSin()，极其安全且不会有弧度转换的 Bug
        double distance = currentPose.getTranslation().getDistance(m_targetWaypoint.getTranslation());
        
        double realTargetVel = (distance / initialDis) * targetVelocity + (1.0 - distance / initialDis) * endSpeed;
        double suppX = realTargetVel * angleTo.getCos();
        double suppY = realTargetVel * angleTo.getSin();
        
        // 5. 保留你的红蓝方反转逻辑
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            suppX = -suppX;
            suppY = -suppY;
        }

        if (m_drivetrain.autoShooting) {
            targetRot = m_drivetrain.autoRot;
        } else {
            targetRot = m_targetWaypoint.getRotation().getDegrees();
        }
        
        // 接下来你就可以直接把 suppX 和 suppY 喂给底盘的 driveClosed 
        m_drivetrain.autoSpeedX = suppX;
        m_drivetrain.autoSpeedY = suppY;
        
        // 5. 应用到底盘
        // FieldCentricFacingAngle 会自动帮你把机器人车头（Rotation）闭环转到 TargetDirection
        m_drivetrain.setControl(
            driveClosed.withVelocityX(suppX)
                       .withVelocityY(suppY)
                       .withTargetDirection(Rotation2d.fromDegrees(targetRot)) // 闭环锁定目标点的期望朝向
        );
    }

    @Override
    public boolean isFinished() {
        // 获取当前位置并计算到目标点的直线距离
        Pose2d currentPose = m_drivetrain.getState().Pose;
        double distance = currentPose.getTranslation().getDistance(m_targetWaypoint.getTranslation());
        
        Rotation2d angleNow = m_targetWaypoint.getTranslation().minus(currentPose.getTranslation()).getAngle();
        Rotation2d angleT = Rotation2d.fromDegrees(angleToTarget);

        // 当距离小于等于容差（10厘米）时或角度出现大变换（冲过头），指令完成
        return distance <= kTranslationTolerance || Math.abs(angleNow.minus(angleT).getDegrees()) >= 80.0;
    }

    @Override
    public void end(boolean interrupted) {
        //ChassisSpeeds currentSpeeds = m_drivetrain.getState().Speeds;

        // 把这些速度赋给底盘身上那个绝对安全的滑行 Request
        if (interrupted && !DriverStation.isAutonomousEnabled()) {
            m_drivetrain.setControl(
                m_drivetrain.m_safeCoastRequest
                            .withVelocityX(0.0)
                            .withVelocityY(0.0)
            );
        } else {
            m_drivetrain.setControl(m_drivetrain.Idle);
        }
    }
}