# 红带特征对比实验

此目录只比较视觉特征，不修改或启动主循迹程序。

对一张包含红带的 BGR 图像，脚本比较：

- `R`：裸红色通道；
- `red_excess = R - max(G, B)`：红色优势；
- `gray_minus_g = gray - G`：灰度亮度减绿色通道。

红带的参考标签仅用于离线量化，由 HSV 红色范围生成；运行时不依赖该标签。

```bash
python3 analyze_red_features.py --image /path/to/frame.jpg
```

输出每个特征在红带/非红带区域的分位数、中位间隔和最佳单阈值平衡准确率。

