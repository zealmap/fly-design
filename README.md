# 无人机航线规划工具

一个基于Web的无人机航线规划工具，帮助用户设计高效的无人机飞行路径，特别适合于航拍、测绘和区域巡检等场景。

## 功能特点

- 支持自定义多边形目标区域
- 灵活设置起飞点和降落点
- 可调整的飞行路径间距
- 支持水平和垂直扫描方向切换
- 提供航线长度和航点数量统计
- 模拟飞行功能，直观展示飞行路径
- 支持导出航线数据为GeoJSON格式
- 支持道路地图和卫星影像切换

## 在线演示

[访问在线演示](https://open.zealmap.com/app/run?url=https://apps.zealmap.com/flydesign/&title=%E6%97%A0%E4%BA%BA%E6%9C%BA%E9%A3%9E%E8%A1%8C%E8%B7%AF%E7%BA%BF%E8%A7%84%E5%88%92)

## 使用说明

1. **区域设置**：
   - 通过GeoJSON数据定义目标区域
   - (TODO)交互在地图上绘制区域

2. **路线配置**：
   - 设置起飞点和降落点
   - 调整航线间距（100米-1000米）
   - 选择扫描方向（水平或垂直）

3. **查看结果**：
   - 显示生成的航线和航点
   - 查看航线统计信息（总长度、航点数量等）
   - 模拟飞行以预览实际飞行路径

4. **导出数据**：
   - 将规划好的航线导出为GeoJSON格式，可用于导入无人机飞控系统

## 技术实现

- 前端：纯HTML/CSS/JavaScript实现
- 地图服务：基于高德地图API
- 航线计算：自定义算法实现多边形区域的均匀覆盖


## 贡献指南

欢迎提交问题报告、功能请求和Pull Request：

1. Fork本仓库
2. 创建您的特性分支 (`git checkout -b feature/amazing-feature`)
3. 提交您的更改 (`git commit -m 'Add some amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 创建一个Pull Request

## 许可证

本项目采用MIT许可证 - 详情请参阅 [LICENSE](LICENSE) 文件
