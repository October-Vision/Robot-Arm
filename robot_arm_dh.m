% ����DH��������λ�����׺ͻ��ȣ�
% ��ʽ��[theta, d, a, alpha]
DH = [0    0    37.5    0;      % �ؽ�1
      -pi/2 0    160.1   0;     % �ؽ�2����2ƫ��-��/2��
      0    142.15 15     -pi/2; % �ؽ�3
      0    0      0      pi/2;  % �ؽ�4
      0    33     0      0];     % �ؽ�5

% �������˶���Revolute��ʾ��ת�ؽڣ�
links = [
    Revolute('d', DH(1,2), 'a', DH(1,3), 'alpha', DH(1,4)), ...
    Revolute('d', DH(2,2), 'a', DH(2,3), 'alpha', DH(2,4)), ...
    Revolute('d', DH(3,2), 'a', DH(3,3), 'alpha', DH(3,4)), ...
    Revolute('d', DH(4,2), 'a', DH(4,3), 'alpha', DH(4,4)), ...
    Revolute('d', DH(5,2), 'a', DH(5,3), 'alpha', DH(5,4))
];

% ����������ģ��
robot = SerialLink(links, 'name', '5DOF-Robot-Arm');

% ��ʾģ�Ͳ���
robot.display();

% ���ӻ�ģ�ͣ�֧�ֽ���ʽ�����ؽڽǶȣ�
robot.teach();