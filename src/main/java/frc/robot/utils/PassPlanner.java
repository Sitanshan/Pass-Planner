package frc.robot.utils;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ReefTargetMap;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * 🌌 Pass Planner | FRC 自动轨迹与战术计算引擎
 * 负责所有的贝塞尔曲线生成、Reef 目标追踪、蓝方宇宙坐标绝对映射。
 */
public class PassPlanner {

    // 🔗 底盘引用 (通过 setDrivetrain 注入，实现架构解耦)
    private CommandSwerveDrivetrain m_drivetrain;

    // 🚀 运行期动态生成的路径缓存 (供外部 Command 读取)
    public double[] dynamicXArray = new double[0];
    public double[] dynamicYArray = new double[0];
    public double[] dynamicSpeedArray = new double[0];

    // 🎯 战术状态变量
    public int targetReefID = 0;
    public int currentRegionID = 0;
    public boolean aimAtLeftSupply = false;
    public boolean aimAtRightSupply = false;
    public int supplyProxyId = 0;

    public PassPlanner() {
        // 构造函数保持干净，等待外部通过 setDrivetrain() 注入底盘
    }

    /**
     * 依赖注入：将底盘实例绑定到引擎中
     */
    public void setDrivetrain(CommandSwerveDrivetrain drivetrain) {
        this.m_drivetrain = drivetrain;
    }

    // ==========================================
    // 🎯 目标管理 (Target Management)
    // ==========================================

    public Command changeReefID(int i) {
        return Commands.runOnce(() -> {
            targetReefID += i;
            if (targetReefID < 0) {
                targetReefID = 5;
            } else if (targetReefID > 5) {
                targetReefID = 0;
            }
        });
    }

    public Command SetReefID(int i) {
        return Commands.runOnce(() -> {
            targetReefID = i;
            if (targetReefID < 0) {
                targetReefID = 5;
            } else if (targetReefID > 5) {
                targetReefID = 0;
            }
        });
    }

    public double getFaceCenterAngle(int faceID) {
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

    // ==========================================
    // 🌍 场地区域与最短路径拓扑计算
    // ==========================================

    /**
     * 实时更新当前所处的 Reef 扇区 (建议在 Drivetrain 的 periodic 中调用)
     */
    public void updateReefRegion() {
        if (m_drivetrain == null) return;

        Pose2d currentPose = m_drivetrain.getState().Pose;
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();

        // 蓝方宇宙映射：红方时将坐标反转到蓝方进行雷达计算
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            currentX = OperatorConstants.fieldHeight - currentX;
            currentY = OperatorConstants.fieldWidth - currentY;
        }

        double dx = currentX - 4.489;
        double dy = currentY - 4.026;
        double angleToReef = Math.toDegrees(Math.atan2(dy, dx));

        if (angleToReef < 0) {
            angleToReef += 360.0;
        }

        if (angleToReef >= 330 || angleToReef < 30) currentRegionID = 3;
        else if (angleToReef >= 30 && angleToReef < 90) currentRegionID = 2;
        else if (angleToReef >= 90 && angleToReef < 150) currentRegionID = 1;
        else if (angleToReef >= 150 && angleToReef < 210) currentRegionID = 0;
        else if (angleToReef >= 210 && angleToReef < 270) currentRegionID = 5;
        else if (angleToReef >= 270 && angleToReef < 330) currentRegionID = 4;
        
        // ==========================================
        // 📊 Elastic Dashboard 兼容层 (原封不动还原旧 Key)
        // ==========================================
        SmartDashboard.putBoolean("reefdown", targetReefID == 0);
        SmartDashboard.putBoolean("reefdownleft", targetReefID == 1);
        SmartDashboard.putBoolean("reefupleft", targetReefID == 2);
        SmartDashboard.putBoolean("reefup", targetReefID == 3);
        SmartDashboard.putBoolean("reefupright", targetReefID == 4);
        SmartDashboard.putBoolean("reefdownright", targetReefID == 5);

        SmartDashboard.putNumber("Current Reef Region", currentRegionID);
        SmartDashboard.putNumber("Current Dis to MidReef", Math.sqrt(dx * dx + dy * dy));
        SmartDashboard.putNumber("ShortestReefPath", calcShortestReefPath());  
    }

    public int calcShortestReefPath() {
        return calcShortestReefPath(this.targetReefID);
    }

    public int calcShortestReefPath(int tar) {
        if (this.currentRegionID == tar) return 0;

        Pose2d currentPose = m_drivetrain.getState().Pose;
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            currentX = OperatorConstants.fieldHeight - currentX;
            currentY = OperatorConstants.fieldWidth - currentY;
        }

        double dx = currentX - 4.489;
        double dy = currentY - 4.026;
        double currentExactAngle = Math.toDegrees(Math.atan2(dy, dx));
        if (currentExactAngle < 0) currentExactAngle += 360.0;

        int forwardSteps = (tar - currentRegionID + 6) % 6;
        int backwardSteps = (currentRegionID - tar + 6) % 6;

        // 等距奇点打断 (Tie-breaker)
        if (forwardSteps == 3) {
            double targetCenterAngle = getFaceCenterAngle(tar);
            double deltaAngle = targetCenterAngle - currentExactAngle;
            
            while (deltaAngle <= -180.0) deltaAngle += 360.0;
            while (deltaAngle >  180.0) deltaAngle -= 360.0;

            if (deltaAngle < 0) return 3;  // 顺时针更近
            else return -3;                // 逆时针更近
        } 
        else if (forwardSteps < backwardSteps) {
            return forwardSteps;
        } else {
            return -backwardSteps;
        }
    }

    // ==========================================
    // 🧮 底层贝塞尔引擎 (Bezier Engine)
    // ==========================================

    private void generateBezierSegment(double startX, double startY, double headingOut, 
                                       double endX, double endY, double headingIn, 
                                       ArrayList<Double> xList, ArrayList<Double> yList) {
        double dist = Math.hypot(endX - startX, endY - startY);
        double weight = dist * 0.4; 

        double cp0X = startX + weight * Math.cos(Math.toRadians(headingOut));
        double cp0Y = startY + weight * Math.sin(Math.toRadians(headingOut));
        double cp1X = endX - weight * Math.cos(Math.toRadians(headingIn));
        double cp1Y = endY - weight * Math.sin(Math.toRadians(headingIn));

        // 动态切割优化
        int numSteps = 100; 
        double accumulatedDist = 0;
        double lastX = startX;
        double lastY = startY;
        
        if (xList.isEmpty()) {
            xList.add(startX);
            yList.add(startY);
        }

        for (int step = 1; step <= numSteps; step++) {
            double t = (double) step / numSteps;
            double omt = 1.0 - t;

            double curX = Math.pow(omt, 3) * startX + 3 * Math.pow(omt, 2) * t * cp0X + 3 * omt * Math.pow(t, 2) * cp1X + Math.pow(t, 3) * endX;
            double curY = Math.pow(omt, 3) * startY + 3 * Math.pow(omt, 2) * t * cp0Y + 3 * omt * Math.pow(t, 2) * cp1Y + Math.pow(t, 3) * endY;

            accumulatedDist += Math.hypot(curX - lastX, curY - lastY);

            // 每凑满 15cm，切割出一个航点
            if (accumulatedDist >= 0.15 || step == numSteps) {
                xList.add(curX);
                yList.add(curY);
                accumulatedDist = 0;
            }
            lastX = curX;
            lastY = curY;
        }
    }

    /**
     * 全局通用三阶贝塞尔曲线生成器 (蓝方宇宙映射版)
     */
    public void generateUniversalBezier(Pose2d startPose, Pose2d endPose, 
                                        double targetDirectionDeg, 
                                        double startSpeed, double endSpeed,
                                        boolean invertXOnRed, boolean invertYOnRed) {
        if (m_drivetrain == null) return;
        
        double startX, startY, headingOut;
        double realStartSpeed = startSpeed;
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        if (startPose != null) {
            startX = startPose.getX();
            startY = startPose.getY();
            headingOut = startPose.getRotation().getDegrees();
        } else {
            Pose2d currentPose = m_drivetrain.getState().Pose;
            startX = currentPose.getX();
            startY = currentPose.getY();

            // 获取绝对 Field-Centric 速度方向
            double robotVx = m_drivetrain.getState().Speeds.vxMetersPerSecond;
            double robotVy = m_drivetrain.getState().Speeds.vyMetersPerSecond;
            Rotation2d currentRot = currentPose.getRotation();

            double fieldVx = robotVx * currentRot.getCos() - robotVy * currentRot.getSin();
            double fieldVy = robotVx * currentRot.getSin() + robotVy * currentRot.getCos();

            double currentActualSpeed = Math.hypot(fieldVx, fieldVy);
            double currentMovingAngle = Math.toDegrees(Math.atan2(fieldVy, fieldVx));

            // 🌟 空间向量映射：将红方物理状态强行投影回蓝方
            if (isRed) {
                if (invertXOnRed) startX = 17.55 - startX;
                if (invertYOnRed) startY = 8.05 - startY;
                
                if (invertXOnRed && invertYOnRed) {
                    currentMovingAngle -= 180.0;
                } else if (invertXOnRed && !invertYOnRed) {
                    currentMovingAngle = 180.0 - currentMovingAngle;
                } else if (!invertXOnRed && invertYOnRed) {
                    currentMovingAngle = -currentMovingAngle;
                }
                
                while (currentMovingAngle > 180.0) currentMovingAngle -= 360.0;
                while (currentMovingAngle < -180.0) currentMovingAngle += 360.0;
            }

            // 动量保留
            if (currentActualSpeed > 1.0) {
                headingOut = currentMovingAngle;
                realStartSpeed = Math.max(startSpeed, currentActualSpeed);
            } else {
                headingOut = Math.toDegrees(Math.atan2(endPose.getY() - startY, endPose.getX() - startX));
            }
        }

        double endX = endPose.getX();
        double endY = endPose.getY();
        double headingIn = targetDirectionDeg; 

        ArrayList<Double> xList = new ArrayList<>();
        ArrayList<Double> yList = new ArrayList<>();
        
        generateBezierSegment(startX, startY, headingOut, endX, endY, headingIn, xList, yList);

        // 存入底层数组缓存
        int totalPoints = xList.size();
        this.dynamicXArray = new double[totalPoints];
        this.dynamicYArray = new double[totalPoints];
        this.dynamicSpeedArray = new double[totalPoints];

        for (int i = 0; i < totalPoints; i++) {
            this.dynamicXArray[i] = xList.get(i);
            this.dynamicYArray[i] = yList.get(i);
            if (totalPoints > 1) {
                double progress = (double) i / (totalPoints - 1);
                this.dynamicSpeedArray[i] = realStartSpeed + (endSpeed - realStartSpeed) * progress;
            } else {
                this.dynamicSpeedArray[i] = endSpeed;
            }
        }
    }

    /**
     * 动态生成前往目标 Reef 的打分路线 (45° 侧切接管)
     */
    public void generateDynamicPath(int branchIndex, double targetSpeedStart, double targetSpeed) {
        if (m_drivetrain == null) return;
        
        Pose2d currentPose = m_drivetrain.getState().Pose;
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        Rotation2d currentRotation = currentPose.getRotation(); 

        double robotVx = m_drivetrain.getState().Speeds.vxMetersPerSecond;
        double robotVy = m_drivetrain.getState().Speeds.vyMetersPerSecond;
        
        double fieldVx = robotVx * currentRotation.getCos() - robotVy * currentRotation.getSin();
        double fieldVy = robotVx * currentRotation.getSin() + robotVy * currentRotation.getCos();

        double currentMovingAngle = Math.toDegrees(Math.atan2(fieldVy, fieldVx));
        double currentActualSpeed = Math.hypot(fieldVx, fieldVy);

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            currentX = OperatorConstants.fieldHeight - currentX;
            currentY = OperatorConstants.fieldWidth - currentY;
            currentMovingAngle += 180.0;
        }

        int step = calcShortestReefPath();

        if (step == 0) {
            this.dynamicXArray = new double[0];
            this.dynamicYArray = new double[0];
            this.dynamicSpeedArray = new double[0];
            return; 
        }

        ArrayList<Double> xList = new ArrayList<>();
        ArrayList<Double> yList = new ArrayList<>();

        double centerX = 4.489;
        double centerY = 4.026;
        double targetCenterAngle = getFaceCenterAngle(this.targetReefID);

        // 获取 Reef 打分点坐标
        double baseX = ReefTargetMap.BLUE_REEF_X[this.targetReefID][branchIndex];
        double baseY = ReefTargetMap.BLUE_REEF_Y[this.targetReefID][branchIndex];
        double outwardOffset = 0.5;
        double lateralOffset = (step > 0) ? 0.5 : -0.5; 
        
        // 投影到 45 度斜角线上
        double p2X = baseX + outwardOffset * Math.cos(Math.toRadians(targetCenterAngle)) + lateralOffset * Math.cos(Math.toRadians(targetCenterAngle + 90.0));
        double p2Y = baseY + outwardOffset * Math.sin(Math.toRadians(targetCenterAngle)) + lateralOffset * Math.sin(Math.toRadians(targetCenterAngle + 90.0));
        double p2HeadingIn = targetCenterAngle + (step > 0 ? 230.0 : 130.0); 

        // 大机动路由分配
        if (Math.abs(step) > 1) {
            double boundaryAngle = targetCenterAngle + (step > 0 ? 90.0 : -90.0);
            double p0X = centerX + 1.9 * Math.cos(Math.toRadians(boundaryAngle));
            double p0Y = centerY + 1.9 * Math.sin(Math.toRadians(boundaryAngle));
            double p0HeadingOut = boundaryAngle + (step > 0 ? -90.0 : 90.0);
            generateBezierSegment(p0X, p0Y, p0HeadingOut, p2X, p2Y, p2HeadingIn, xList, yList);
        } else if (Math.abs(step) == 1) {
            double p1X = centerX + 2.3 * Math.cos(Math.toRadians(targetCenterAngle + (step > 0 ? 30.0 : -30.0)));
            double p1Y = centerY + 2.3 * Math.sin(Math.toRadians(targetCenterAngle + (step > 0 ? 30.0 : -30.0)));
            double p0HeadingOut = Math.toDegrees(Math.atan2(p1Y - currentY, p1X - currentX));
            
            if (currentActualSpeed > 1.0) p0HeadingOut = currentMovingAngle;
            generateBezierSegment(currentX, currentY, p0HeadingOut, p2X, p2Y, p2HeadingIn, xList, yList);
        }

        int totalPoints = xList.size();
        this.dynamicXArray = new double[totalPoints];
        this.dynamicYArray = new double[totalPoints];
        this.dynamicSpeedArray = new double[totalPoints];

        for (int i = 0; i < totalPoints; i++) {
            this.dynamicXArray[i] = xList.get(i);
            this.dynamicYArray[i] = yList.get(i);
            
            if (totalPoints > 1) {
                double progress = (double) i / (totalPoints - 1);
                this.dynamicSpeedArray[i] = targetSpeedStart + (targetSpeed - targetSpeedStart) * progress;
            } else {
                this.dynamicSpeedArray[i] = targetSpeed;
            }
        }
    }

    public Command gDPCommand(int branchIndex, double tS, double tE){
        return Commands.runOnce(() -> generateDynamicPath(branchIndex, tS, tE));
    }

    // ==========================================
    // 🛠️ 预设通用路线 Command
    // ==========================================

    public Command CreatePathToLeftSupply() {
        return Commands.runOnce(() -> {
            aimAtLeftSupply = true;
            generateUniversalBezier(
                null, 
                new Pose2d(1.6, 7.4, Rotation2d.fromDegrees(-53)), 
                127, 3, 3, 
                true, true // 中心对称
            );
        });
    }

    public Command CreatePathToRightSupply() {
        return Commands.runOnce(() -> {
            aimAtRightSupply = true;
            generateUniversalBezier(
                null, 
                new Pose2d(1.6, 0.65, Rotation2d.fromDegrees(53)), 
                -127, 3, 3, 
                true, true // 中心对称
            );
        });
    }
}