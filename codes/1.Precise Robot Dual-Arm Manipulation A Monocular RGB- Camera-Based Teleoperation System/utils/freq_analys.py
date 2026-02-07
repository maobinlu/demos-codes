import numpy as np
import matplotlib.pyplot as plt


def analyze_frequency(temp_pred, sample_freq=30):
    t = np.arange(0, len(temp_pred), 1)
    freq1 = np.fft.fftfreq(len(temp_pred), d=1/sample_freq)[:len(temp_pred)//2]  # 单边频率轴

    # 计算误差
    error = temp_pred - np.mean(temp_pred)

    # 快速傅里叶变换
    E_f = np.fft.fft(error)  # 计算傅里叶变换
    magnitude = np.abs(E_f)[:len(E_f)//2]  # 只取正频率部分

    # 绘制频谱图
    plt.figure(figsize=(10, 6))
    plt.plot(freq1, magnitude, label="Frequency Spectrum")
    plt.title("Frequency Spectrum of the Signal")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")
    plt.grid(True)
    plt.legend()
    plt.show()


# 使用示例数据
sample_freq = 300
# 示例带周期性噪声的信号
temp_pred = np.sin(np.linspace(0, 10, 300)) + 0.5 * np.sin(np.linspace(0, 10, 300) * 10) + 0.5 * np.random.randn(300)
analyze_frequency(temp_pred, sample_freq)
