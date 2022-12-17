%% 1-Forward kinemtics
clear
clc
%sympolic DH matrix  
syms th d alp a
A =[cos(th) -sin(th)*cos(alp) sin(th)*sin(alp) a*cos(th)
   sin(th) cos(th)*cos(alp) -cos(th)*sin(alp) a*sin(th)
   0       sin(alp)                  cos(alp)         d
   0 0 0 1];

%insert dh parameter for each link for exmple of 6 DOF robot arm

%Link 1
syms q1 L1
A1=subs(A,{a,alp,d,th},{0,pi/2,L1,q1});

%link 1 CG
A1C=subs(A,{a,alp,d,th},{0,pi/2,L1/2,q1});


%Link 2
syms q2 L2
A2=subs(A,{a,alp,d,th},{L2,0,0,q2});

%link 2 CG
A2C=subs(A,{a,alp,d,th},{L2/2,0,0,q2});


%Link 3
syms q3 L3
A3=subs(A,{a,alp,d,th},{L3,0,0,q3});


%2-Transformatiom matrix
%link 3 CG
A3C=subs(A,{a,alp,d,th},{L3/2,0,0,q3});
%Link 3 matrix
At=simplify(A1*A2*A3C);
%Link 2 matrix
At2=simplify(A1*A2C);


%% 2-Jacobian (for links with graphicl method)

%1-Linear jacobian

%Center of grafity
Oc3=[At(1,4), At(2,4), At(3,4)];
Oc2=[At2(1,4), At2(2,4), At2(3,4)];
Oc1=[A1C(1,4), A1C(2,4), A1C(3,4)];

%Final matrices
JD1=[diff(Oc1(1,1),q1), diff(Oc1(1,1),q2), diff(Oc1(1,1),q3);
    diff(Oc1(1,2),q1), diff(Oc1(1,2),q2), diff(Oc1(1,2),q3);
     diff(Oc1(1,3),q1), diff(Oc1(1,3),q2), diff(Oc1(1,3),q3);];
 
 JD2=[diff(Oc2(1,1),q1), diff(Oc2(1,1),q2), diff(Oc2(1,1),q3);
    diff(Oc2(1,2),q1), diff(Oc2(1,2),q2), diff(Oc2(1,2),q3);
     diff(Oc2(1,3),q1), diff(Oc2(1,3),q2), diff(Oc2(1,3),q3);];
 
 JD3=[diff(Oc3(1,1),q1), diff(Oc3(1,1),q2), diff(Oc3(1,1),q3);
    diff(Oc3(1,2),q1), diff(Oc3(1,2),q2), diff(Oc3(1,2),q3);
     diff(Oc3(1,3),q1), diff(Oc3(1,3),q2), diff(Oc3(1,3),q3);];
 


%2- Angular jacobian

%Rotation matrices
R0=[1, 0, 0;
    0, 1, 0;
    0, 0, 1;];

R1=[A1(1,1), A1(1,2), A1(1,3);
    A1(2,1), A1(2,2), A1(2,3);
    A1(3,1), A1(3,2), A1(3,3);];

AR2=simplify(A1*A2);

R2=[AR2(1,1), AR2(1,2), AR2(1,3);
    AR2(2,1), AR2(2,2), AR2(2,3);
    AR2(3,1), AR2(3,2), AR2(3,3);];


R3=[At(1,1), At(1,2), At(1,3);
    At(2,1), At(2,2), At(2,3);
    At(3,1), At(3,2), At(3,3);];


Re=[0; 0; 1;];
Pr=[0; 0; 0;];

Z0=R0*Re;
Z1=R1*Re;
Z2=R2*Re;

%Z3=R3*Re;
%Z4=R4*Re;
%Z5=R5*Re;

%no joint
N=[0; 0; 0;];

%Final matrices
JR1=[Z0 N N];
JR2=[Z0 Z1 N];
JR3=[Z0 Z1 Z2];

%% Dynamics

%1-Ineriat matrix (D)

 syms m1 m2 m3;

   syms r1; %radious of frist link
  
  %Inertia of links
   I11=[0 0 0; 0 (m1*r1^2)/2 0; 0 0 0];
   I22=[0 0 0; 0 (m2*L2^2)/12 0; 0 0 (m2*L2^2)/12];
   I33=[0 0 0; 0 (m3*L3^2)/12 0; 0 0 (m3*L3^2)/12];
   
   %Inertia of links with referance to the base
   I1=R1*I11*transpose(R1);
   I2=R2*I22*transpose(R2);
   I3=R3*I33*transpose(R3);
   
   
   D1=transpose(JD1)*m1*JD1;
   D2=transpose(JR1)*I1*JR1;
   D3=simplify(transpose(JD2)*m2*JD2);
   D4=simplify(transpose(JR2)*I2*JR2);
   D5=simplify(transpose(JD3)*m3*JD3);
   D6=simplify(transpose(JR3)*I3*JR3);
   
   D=simplify(D1+D2+D3+D4+D5+D6)
   
   
   
   
   %3-Gravity matrx (G)
   
   syms g;
   g0=[0 0 g];
   
   
   G1=m1*g0*JD1(:,1)+m1*g0*JD2(:,1)+m3*g0*JD3(:,1);
   G2=m1*g0*JD1(:,2)+m1*g0*JD2(:,2)+m3*g0*JD3(:,2);
   G3=m1*g0*JD1(:,3)+m1*g0*JD2(:,3)+m3*g0*JD3(:,3);
   
   G=[G1; G2; G3]
   
    %3-Christoffel matrx (H)
    
   %Frist joint
   H111=simplify(diff(D(1,1),q1)-(1/2)*diff(D(1,1),q1));
   H112=simplify(diff(D(1,1),q2)-(1/2)*diff(D(1,2),q1));
   H121=simplify(diff(D(1,2),q1)-(1/2)*diff(D(2,1),q1));
   H122=simplify(diff(D(1,2),q2)-(1/2)*diff(D(2,2),q1));
   %3DOF
   H113=diff(D(1,1),q3)-(1/2)*diff(D(1,3),q1);
   H131=simplify(diff(D(1,3),q1)-(1/2)*diff(D(3,1),q1));
   H123=simplify(diff(D(1,2),q3)-(1/2)*diff(D(2,3),q1));
   H132=simplify(diff(D(1,3),q2)-(1/2)*diff(D(3,2),q1));
   H133=simplify(diff(D(1,3),q3)-(1/2)*diff(D(3,3),q1));
 
   %Second joint
   H211=simplify(diff(D(2,1),q1)-(1/2)*diff(D(1,1),q2));
   H212=simplify(diff(D(2,1),q2)-(1/2)*diff(D(1,2),q2));
   H221=simplify(diff(D(2,2),q1)-(1/2)*diff(D(2,1),q2));
   H222=simplify(diff(D(2,2),q2)-(1/2)*diff(D(2,2),q2));
   %3DOF
   H213=simplify(diff(D(2,1),q3)-(1/2)*diff(D(1,3),q2));
   H231=simplify(diff(D(2,3),q1)-(1/2)*diff(D(3,1),q2));
   H223=simplify(diff(D(2,2),q3)-(1/2)*diff(D(2,3),q2));
   H232=simplify(diff(D(2,3),q2)-(1/2)*diff(D(3,2),q2));
   H233=simplify(diff(D(2,3),q3)-(1/2)*diff(D(3,3),q2));

   
  %Third joint
   H311=simplify(diff(D(3,1),q1)-(1/2)*diff(D(1,1),q3)); 
   H312=simplify(diff(D(3,1),q2)-(1/2)*diff(D(1,2),q3));
   H321=simplify(diff(D(3,2),q1)-(1/2)*diff(D(2,1),q3));
   H322=simplify(diff(D(3,2),q2)-(1/2)*diff(D(2,2),q3));
   %3DOF
   H313=simplify(diff(D(3,1),q3)-(1/2)*diff(D(1,3),q3));
   H331=simplify(diff(D(3,3),q1)-(1/2)*diff(D(3,1),q3));
   H323=simplify(diff(D(3,2),q3)-(1/2)*diff(D(2,3),q3));
   H332=simplify(diff(D(3,3),q2)-(1/2)*diff(D(3,2),q3));
   H333=simplify(diff(D(3,3),q3)-(1/2)*diff(D(3,3),q3));
   
   syms qd1 qd2 qd3
   
   H1=(H121+H112)*qd1*qd2 +(H131+H113)*qd1*qd3 + (H123+H132)*qd2*qd3 + (H111)*qd1^2+(H122)*qd2^2 +(H133)*qd3^2 ;
   H2=(H221+H212)*qd1*qd2 +(H231+H213)*qd1*qd3 + (H223+H232)*qd2*qd3 + (H211)*qd1^2+(H222)*qd2^2 +(H233)*qd3^2 ;
   H3=(H321+H312)*qd1*qd2 +(H331+H313)*qd1*qd3 + (H323+H332)*qd2*qd3 + (H311)*qd1^2+(H322)*qd2^2 +(H333)*qd3^2 ;
 
   H=[H1; H2; H3]
   %% Inverse Dynamics
qdd = inv(D)*(T - (H + G));
qdd = simplify(qdd);

%Display Matrices
disp('qdd = ');
disp(qdd);