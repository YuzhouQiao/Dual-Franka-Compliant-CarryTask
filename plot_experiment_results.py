#!/usr/bin/env python3
import csv
import sys
import glob
import os
import matplotlib
# Use a non-interactive backend by default to avoid issues when ending the task,
# can also be changed to 'TkAgg' if we want it to block and show. 
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    # 找到最新生成的 csv 文件
    csv_files = glob.glob('/home/wusiala/mujoco_franka/franka_ws/experiment_data/*.csv')
    if not csv_files:
        print("❌ 未找到实验数据CSV文件！图表生成失败。")
        sys.exit(1)

    latest_csv = max(csv_files, key=os.path.getctime)
    print(f"📊 正在读取数据并绘制全阶段性能图表: {latest_csv}")

    times = []
    left_y = []
    right_y = []
    stress = []
    accel = []
    k_scale = []
    d_scale = []

    try:
        with open(latest_csv, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)
            for row in reader:
                if not row: continue
                times.append(float(row[0]))
                left_y.append(float(row[1]))
                right_y.append(float(row[2]))
                stress.append(float(row[3]))
                accel.append(float(row[4]))
                if len(row) >= 7:
                    k_scale.append(float(row[5]))
                    d_scale.append(float(row[6]))
                else:
                    k_scale.append(100.0)
                    d_scale.append(100.0)
    except Exception as e:
        print(f"读取 CSV 出错: {e}")
        sys.exit(1)

    if not times:
        print("CSV 文件内容为空数据！不生成图表。")
        sys.exit(0)

    # 开始离线高质量绘图
    plt.figure(figsize=(10, 12))
    plt.rcParams["font.sans-serif"] = ["WenQuanYi Zen Hei", "Noto Sans CJK SC", "Microsoft YaHei"]
    plt.rcParams["axes.unicode_minus"] = False

    # =============== 顶部子图：双臂Y轴接触力 ===============
    plt.subplot(4, 1, 1)
    plt.plot(times, left_y, label='左臂 Y向受力', color='g')
    plt.plot(times, right_y, label='右臂 Y向受力', color='b')
    plt.title("全局双臂Y轴接触力 (N)")
    plt.xlabel("任务完整时间流 (s)")
    plt.ylabel("受力值 (N)")
    plt.grid(True)
    plt.legend(loc='upper right')

    # =============== 子图2：内应力 ===============
    plt.subplot(4, 1, 2)
    plt.plot(times, stress, label='夹击内应力', color='r', linewidth=2)
    plt.axhline(y=5.0, color='grey', linestyle='--', label='应力安全阈值参考')
    plt.title("全局内应力评估 [柔顺性核心指标 - 值越低、突变越少越好]")
    plt.xlabel("任务完整时间流 (s)")
    plt.ylabel("内应力大小 (N)")
    plt.grid(True)
    plt.legend(loc='upper right')
    
    # 根据最大值动态调整一下Y轴，避免顶部贴边
    max_stress = max(stress) if stress else 0
    plt.ylim(0, max_stress + 5.0)

    # =============== 子图3：平滑度 ===============
    plt.subplot(4, 1, 3)
    plt.plot(times, accel, label='关节加速度范数', color='m', linewidth=2)
    plt.title("全阶段运动平滑度 [CHOMP规划评估指标 - 起伏越小越平顺]")
    plt.xlabel("任务完整时间流 (s)")
    plt.ylabel("加速度范数强度")
    plt.grid(True)
    plt.legend(loc='upper right')

    # =============== 底部子图：自适应阻抗缩放 ===============
    plt.subplot(4, 1, 4)
    plt.plot(times, k_scale, label='刚度 K 缩放 (%)', color='k', linewidth=2)
    plt.plot(times, d_scale, label='阻尼 D 缩放 (%)', color='orange', linewidth=2)
    plt.title("全局自适应阻抗参数变化率")
    plt.xlabel("任务完整时间流 (s)")
    plt.ylabel("参数缩放率 (%)")
    plt.grid(True)
    plt.legend(loc='upper right')

    plt.tight_layout()
    
    # 替换后缀保存成同一目录下的图片
    png_name = latest_csv.replace('.csv', '.png')
    plt.savefig(png_name, dpi=300)
    print(f"✅ 【完美】全阶段数据高精度分析曲线已生成，并保存至: {png_name}")
    print( "   你现在可以前往 experiment_data 文件夹打开查阅该图！")
    
    # 退出前释放内存
    plt.close()

if __name__ == "__main__":
    main()
