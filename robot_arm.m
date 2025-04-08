L(1)=Link('revolute','d',1.06,'a',0,'alpha',0,'modified');
L(2)=Link('revolute','d',0.74,'a',0.375,'alpha',pi/2,'offset',pi/2,'modified');
L(3)=Link('revolute','d',-0.32,'a',1.6,'alpha',0,'offset',0,'modified');
L(4)=Link('revolute','d',1.3,'a',0.05,'alpha',-pi*3/2,'offset',pi/2,'modified');
L(5)=Link('revolute','d',0,'a',0,'alpha',pi/2,'offset',pi/2,'modified');
Five_dof_mod=SerialLink(L,'name','5-dof');
Five_dof_mod.teach
Five_dof_mod.base = transl(0,0,0.28); 
L(1).qlim=[-150,150]/180*pi;
L(2).qlim=[-100,90]/180*pi;
L(3).qlim=[-90,90]/180*pi;
L(4).qlim=[-180,180]/180*pi;
L(5).qlim=[-90,90]/180*pi;
num = 3000;
P= zeros(num,3);
for i=1:num
q1 = L(1).qlim(1) + rand*(L(1).qlim(2) - L(1).qlim(1));
q2 = L(2).qlim(1) + rand*(L(2).qlim(2) - L(2).qlim(1)); 
q3 = L(3).qlim(1) + rand*(L(3).qlim(2) - L(3).qlim(1));
q4 = L(4).qlim(1) + rand*(L(4).qlim(2) - L(4).qlim(1));
q5 = L(5).qlim(1) + rand*(L(5).qlim(2) - L(5).qlim(1));
q = [q1 q2 q3 q4 q5];
T = Five_dof_mod.fkine(q); 
P(i, :) = transl(T);
end
plot3(P(:,1), P(:,2), P(:,3), 'b.', 'MarkerSize', 1, 'LineStyle', 'none');
hold on
grid on
daspect([1 1 1]);
view([45 45]);
Five_dof_mod.plot([0 0 0 0 0]); 