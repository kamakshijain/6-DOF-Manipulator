syms o1 o2 o3 o4 o5 o6;

dh_table=[0 200 0 pi/2; 0 0 100 -pi/2; 0 0 100 pi/2; 0 0 0 pi/2; 0 150 0 -pi/2; 0 0 60 0]; %Dh table for the manipulator
robot=SerialLink(dh_table)%converting the DH table into a stick file which describes our manipulator

o=[0 0 0 pi/2 0 -pi/2];
%e=tr2eul(r.fkine(o))

%r.ikunc(r.fkine(o))
robot.plot(o);
%j0 = r.jacob0(o);

%r.teach;
T1=robot.fkine(o);% forward kinematics for a given set of joint angles
j0 = robot.jacob0(o)% getting the jacobian for a particular joint angles

T2=robot.fkine([pi/2 0 pi/2 0 pi/2 0]);%Storing the homogeneous transformation matix
%r.ikine(T2)

i=[1:1:50];
t=robot.jtraj(T1, T2, i);
%r.plot(t)


for i=1:1:50
    g=robot.fkine(t(i,:));
    g=double(g);
    q(i,1)=g(1,4); q(i,2)=g(2,4); q(i,3)=g(3,4);    
end
for z=1:1:50
    robot.plot(t(z,:))
    hold on
    plot3(q(z,1),q(z,2),q(z,3),'r.');
end
hold on

T3=robot.fkine([0 0 0 pi/4 0 0]);
robot.ikine(T3)

i=[1:1:50];
t=robot.jtraj(T2, T3, i);
%r.plot(t)


for i=1:1:50
    g=robot.fkine(t(i,:));
    g=double(g);
    q(i,1)=g(1,4); q(i,2)=g(2,4); q(i,3)=g(3,4);    
end
for z=1:1:50
    robot.plot(t(z,:))
    hold on
    plot3(q(z,1),q(z,2),q(z,3),'b.');
end

T4=robot.fkine([0 pi/6 0 pi/2 0 0]);
robot.ikine(T3)

i=[1:1:50];
t=robot.jtraj(T3, T4, i);
%r.plot(t)


for i=1:1:50   
    g=robot.fkine(t(i,:));
    g=double(g);
    q(i,1)=g(1,4); q(i,2)=g(2,4); q(i,3)=g(3,4);    
end                %to get colour for the trajectory moved
for z=1:1:50
    robot.plot(t(z,:))
    hold on
    plot3(q(z,1),q(z,2),q(z,3),'g.');
end

T5=robot.fkine([0 -pi/4 0 pi/2 pi/2 0]);
robot.ikine(T3)

i=[1:1:50];
t=robot.jtraj(T4, T5, i);
%r.plot(t)


for i=1:1:50
    g=robot.fkine(t(i,:));
    g=double(g);
    q(i,1)=g(1,4); q(i,2)=g(2,4); q(i,3)=g(3,4);    
end
for z=1:1:50
    robot.plot(t(z,:))
    hold on
    plot3(q(z,1),q(z,2),q(z,3),'b.');
end