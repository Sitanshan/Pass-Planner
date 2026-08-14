// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.PassPlannerReader;
import frc.robot.utils.PassPlannerReader.RobotPath;

@SuppressWarnings("unused")
public final class Autos {

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * 🚀 纯 PassPlanner 数据驱动的自动阶段路线 (仅依赖 Drivetrain)
     */
    public static Command autoReef(CommandSwerveDrivetrain drivetrain, boolean invertA, boolean invertD) {
        
        // 1. 从 JSON 加载路线
        RobotPath myPath = PassPlannerReader.loadPath("reefPath");

        // 2. 防呆检测：确保文件里真的有路线
        if (myPath.segments.length < 1) {
            SmartDashboard.putString("noPath", "no");
            return new WaitCommand(1.0); // 随便返回个发呆指令保命
        }

        // 3. 提取全局刹车 PID
        double p = myPath.brakePID.p;
        double i = myPath.brakePID.i;
        double di = myPath.brakePID.d;

        // 4. 计算红蓝方镜像时的左右 Branch 分支映射
        int branch1 = ((!invertA && !invertD) || (invertA && invertD)) ? 1 : 0;
        int branch2 = ((!invertA && !invertD) || (invertA && invertD)) ? 0 : 1;

        // 5. 组装成连贯的动作流
        return Commands.sequence(
            
            // ============== Segment 0 ==============
            Commands.either(drivetrain.passPlanner.SetReefID(2), drivetrain.passPlanner.SetReefID(4), () -> (!invertA && !invertD) || (invertA && invertD)),
            new AutoMoveClosed(drivetrain, () -> branch1, 3), // 初始或前置动作
            new AutoMoveComplex(
                drivetrain,
                myPath.segments[0].xArray,
                myPath.segments[0].yArray,
                myPath.segments[0].rotArray,
                myPath.segments[0].speedArray,
                myPath.segments[0].stopAtEnd,
                invertA, invertD, 
                p, i, di       
            ),
            Commands.waitSeconds(0.5),

            // ============== Segment 1 ==============
            Commands.either(drivetrain.passPlanner.SetReefID(1), drivetrain.passPlanner.SetReefID(5), () -> (!invertA && !invertD) || (invertA && invertD)),
            new AutoMoveComplex(
                drivetrain,
                myPath.segments[1].xArray,
                myPath.segments[1].yArray,
                myPath.segments[1].rotArray,
                myPath.segments[1].speedArray,
                myPath.segments[1].stopAtEnd,
                invertA, invertD,
                p, i, di
            ),
            new AutoMoveClosed(drivetrain, () -> branch1, 3),
            
            // ============== Segment 2 ==============
            new AutoMoveComplex(
                drivetrain,
                myPath.segments[2].xArray,
                myPath.segments[2].yArray,
                myPath.segments[2].rotArray,
                myPath.segments[2].speedArray,
                myPath.segments[2].stopAtEnd,
                invertA, invertD,
                p, i, di
            ),
            Commands.waitSeconds(0.5),

            // ============== Segment 3 ==============
            new AutoMoveComplex(
                drivetrain,
                myPath.segments[3].xArray,
                myPath.segments[3].yArray,
                myPath.segments[3].rotArray,
                myPath.segments[3].speedArray,
                myPath.segments[3].stopAtEnd,
                invertA, invertD,
                p, i, di
            ),
            new AutoMoveClosed(drivetrain, () -> branch2, 3),

            // ============== Segment 4 ==============
            new AutoMoveComplex(
                drivetrain,
                myPath.segments[4].xArray,
                myPath.segments[4].yArray,
                myPath.segments[4].rotArray,
                myPath.segments[4].speedArray,
                myPath.segments[4].stopAtEnd,
                invertA, invertD,
                p, i, di
            ),
            Commands.waitSeconds(0.5),

            // ============== Segment 1 (Repeat) ==============
            new AutoMoveComplex(
                drivetrain,
                myPath.segments[1].xArray,
                myPath.segments[1].yArray,
                myPath.segments[1].rotArray,
                myPath.segments[1].speedArray,
                myPath.segments[1].stopAtEnd,
                invertA, invertD,
                p, i, di
            ),
            new AutoMoveClosed(drivetrain, () -> branch1, 3),
            
            // ============== Segment 2 (Repeat) ==============
            new AutoMoveComplex(
                drivetrain,
                myPath.segments[2].xArray,
                myPath.segments[2].yArray,
                myPath.segments[2].rotArray,
                myPath.segments[2].speedArray,
                myPath.segments[2].stopAtEnd,
                invertA, invertD,
                p, i, di
            ),
            Commands.waitSeconds(0.5),

            // ============== Segment 3 (Repeat) ==============
            new AutoMoveComplex(
                drivetrain,
                myPath.segments[3].xArray,
                myPath.segments[3].yArray,
                myPath.segments[3].rotArray,
                myPath.segments[3].speedArray,
                myPath.segments[3].stopAtEnd,
                invertA, invertD,
                p, i, di
            ),
            new AutoMoveClosed(drivetrain, () -> branch2, 3),

            // ============== Segment 4 (Repeat) ==============
            new AutoMoveComplex(
                drivetrain,
                myPath.segments[4].xArray,
                myPath.segments[4].yArray,
                myPath.segments[4].rotArray,
                myPath.segments[4].speedArray,
                myPath.segments[4].stopAtEnd,
                invertA, invertD,
                p, i, di
            )
        );
    }
}