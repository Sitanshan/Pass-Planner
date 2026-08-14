// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final double SpeedRate = 0.75;
        public static final double AngularSpeedRate = 0.95;
        public static final double fieldWidth=8.05;
        public static final double fieldHeight=17.55;

    }

    public static final class DriveConstants {
        public static final double kPTranslation = 1.0;  // 位置到速度的比例系数
        public static final double kPRotation = 1.0;     // 角度到角速度的比例系数
    }

    
    public class ReefTargetMap {
        
    // 铁血法则：[0] = 机器人面向 Reef 时的右侧 (Right) 
    //           [1] = 机器人面向 Reef 时的左侧 (Left)

    public static final double[][] BLUE_REEF_X = {
        /* Face 0 (180° - 联盟墙面) */ { 3.148, 3.148 },
        /* Face 1 (120° - 西北面)   */ { 3.676, 3.961 }, // 🚨 修复了 X/Y 错位
        /* Face 2 (60°  - 东北面)   */ { 5.017, 5.302 },
        /* Face 3 (0°   - 中场面)   */ { 5.830, 5.830 },
        /* Face 4 (300° - 东南面)   */ { 5.302, 5.017 },
        /* Face 5 (240° - 西南面)   */ { 3.961, 3.676 }
    };

    public static final double[][] BLUE_REEF_Y = {
        /* Face 0 (180° - 联盟墙面) */ { 3.861, 4.191 }, // 🚨 修复了左右视角反转
        /* Face 1 (120° - 西北面)   */ { 5.105, 5.270 },
        /* Face 2 (60°  - 东北面)   */ { 5.270, 5.105 },
        /* Face 3 (0°   - 中场面)   */ { 4.191, 3.861 }, // 🚨 修复了左右视角反转
        /* Face 4 (300° - 东南面)   */ { 2.947, 2.782 },
        /* Face 5 (240° - 西南面)   */ { 2.782, 2.947 }
    };
    // ==========================================
    // 🌿 ALGAE (海藻) 抓取绝对坐标 (位于两个 Branch 正中间)
    // ==========================================
    public static final double[] BLUE_ALGAE_X = {
        /* Face 0 (180° - 联盟墙面) */ 3.148,
        /* Face 1 (120° - 西北面)   */ 3.819,
        /* Face 2 (60°  - 东北面)   */ 5.160,
        /* Face 3 (0°   - 中场面)   */ 5.830,
        /* Face 4 (300° - 东南面)   */ 5.160,
        /* Face 5 (240° - 西南面)   */ 3.819
    };

    public static final double[] BLUE_ALGAE_Y = {
        /* Face 0 (180° - 联盟墙面) */ 4.026,
        /* Face 1 (120° - 西北面)   */ 5.188,
        /* Face 2 (60°  - 东北面)   */ 5.188,
        /* Face 3 (0°   - 中场面)   */ 4.026,
        /* Face 4 (300° - 东南面)   */ 2.865,
        /* Face 5 (240° - 西南面)   */ 2.865
    };

    // ==========================================
    // 🎯 ALGAE PREPARE (预备位) 坐标 (垂直向外推 0.5 米)
    // ==========================================
    public static final double[] BLUE_ALGAE_PREP_X = {
        /* Face 0 (180° - 联盟墙面) */ 2.648,
        /* Face 1 (120° - 西北面)   */ 3.569,
        /* Face 2 (60°  - 东北面)   */ 5.410,
        /* Face 3 (0°   - 中场面)   */ 6.330,
        /* Face 4 (300° - 东南面)   */ 5.410,
        /* Face 5 (240° - 西南面)   */ 3.569
    };

    public static final double[] BLUE_ALGAE_PREP_Y = {
        /* Face 0 (180° - 联盟墙面) */ 4.026,
        /* Face 1 (120° - 西北面)   */ 5.620,
        /* Face 2 (60°  - 东北面)   */ 5.620,
        /* Face 3 (0°   - 中场面)   */ 4.026,
        /* Face 4 (300° - 东南面)   */ 2.432,
        /* Face 5 (240° - 西南面)   */ 2.432
    };
    public static final double[] FACE_TARGET_ROTATION = {
        0.0,    // Face 0: 面对联盟墙时车头朝向 0°
        -60.0,  // Face 1: 面向西北时车头朝向 -60°
        -120.0, // Face 2: 面向东北时车头朝向 -120°
        180.0,  // Face 3: 面向中场时车头朝向 180°
        120.0,  // Face 4: 面向东南时车头朝向 120°
        60.0    // Face 5: 面向西南时车头朝向 60°
    };
}
}