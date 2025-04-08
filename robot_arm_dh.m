% 创建DH参数表（单位：毫米和弧度）
% 格式：[theta, d, a, alpha]
DH = [0    0    37.5    0;      % 关节1
      -pi/2 0    160.1   0;     % 关节2（θ2偏移-π/2）
      0    142.15 15     -pi/2; % 关节3
      0    0      0      pi/2;  % 关节4
      0    33     0      0];     % 关节5

% 创建连杆对象（Revolute表示旋转关节）
links = [
    Revolute('d', DH(1,2), 'a', DH(1,3), 'alpha', DH(1,4)), ...
    Revolute('d', DH(2,2), 'a', DH(2,3), 'alpha', DH(2,4)), ...
    Revolute('d', DH(3,2), 'a', DH(3,3), 'alpha', DH(3,4)), ...
    Revolute('d', DH(4,2), 'a', DH(4,3), 'alpha', DH(4,4)), ...
    Revolute('d', DH(5,2), 'a', DH(5,3), 'alpha', DH(5,4))
];

% 构建机器人模型
robot = SerialLink(links, 'name', '5DOF-Robot-Arm');

% 显示模型参数
robot.display();

% 可视化模型（支持交互式调整关节角度）
robot.teach();