clear;
clc;

home();
gripperhome();
pick('C');
place('A');
home();
pick('A');
place('B');
home();
pick('B');
place('C');
home();

%motion for picking the ball
function pick(a)
global gamma_temp;
global theta_temp;
theta_temp = -90;
gamma_temp = 84;

if (a=='A')
    x = -90;
    h = 70;
elseif (a=='B')
    x = 0;
    h = 0;
elseif (a=='C')
    x = 90;
    h = 0;
end 
[theta, gamma] = inv(cosd(x), sind(x), h);
disp('g')
disp(x)
armrotation(theta);     
armdown(gamma);        
gripperclose();
end

%Motion for placing the ball
function place(a)
global theta_temp;
global gamma_temp;

if (a=='A')
    x = -90;
    h = 70
elseif (a=='B')
    x = 0;
    h = 0;
elseif (a=='C')
    x = 90;
    h = 0;
end
[theta, gamma] = inv(cosd(x), sind(x), 150);
armup(gamma);        
armrotation(theta);    
[theta, gamma] = inv(cosd(x), sind(x), h);
armdown(gamma); 
disp('gamma')
disp('theta')
gripperopen();
end
%move motor B to start position
function home()
myev3 = legoev3('USB');
mytouchsensor = touchSensor(myev3,3);
mymotor = motor(myev3, 'B')
start(mymotor)
resetRotation(mymotor);
while ~readTouch(mytouchsensor)
    mymotor.Speed = -30;
end
readRotation(mymotor)
mymotor.Speed = 0;
%move motor C to start position
mytouchsensor = touchSensor(myev3,1)
mymotor = motor(myev3, 'C')
start(mymotor)
while ~readTouch(mytouchsensor)
    mymotor.Speed = 30;
end
mymotor.Speed = 0;
end

% inverse kinematics
function  [ theta, gamma] = inv(x, y, z)
global theta_temp;                              %store temporary theta
l_3 = 185;                                      %link 3
l_2 = 95;                                        %link 2
l_1 = 50;                                        %link 1
l_4 = 110;                                        %link 5
b_1 = 70;                                         %height from grouund to link 1
theta_1 = atan2d(y,x); 
theta = -round((theta_1 - theta_temp)* 10/3);   %gear ratio 3:1 but we take 10:3
theta_temp = theta_1;                            % because of irregularities
global gamma_temp;                                % store temporary gamma
r_1 = z + 110 - b_1 -l_1-l_2*sind(45);
r = sqrt(r_1^2 + l_3^2);
gamma_1 = 45 + asind(r_1 / l_3); 
gamma = round((gamma_temp-gamma_1) * 533/100);
gamma_temp = gamma_1;
end

%arm rotaion
function armrotation(theta)
myev3 = legoev3('USB');
mymotor = motor(myev3, 'C');
start(mymotor);
resetRotation(mymotor);
uk1=0.0;ek1=0.0;ek2=0.0; c0 = 0.0;c1=0.0;c2=0.0;ek=0.0;uk=0.0;
kp=0.001;kd=0.00;ki=0.00;t=0.01;


while (readRotation(mymotor) ~= theta)
   c0=kp+kd/t;
    c1=ki*t -kp -2*kd/t;
    c2=kd/t;
    ek = theta- double(readRotation(mymotor));
    uk=uk1+c0*ek+c1*ek1+c2*ek2;
    ek2=ek1;
    if ek > 20
        mymotor.Speed = 20;
        uk1=2;
        
    elseif ek<-20
        mymotor.Speed = -20;
        uk1=-2;
        
    else
        mymotor.Speed = uk;
        uk1 = uk;
    end 
end
mymotor.Speed = 0;
rotation = readRotation(mymotor)

end

%Gripper home position

function gripperhome()
myev3 = legoev3('USB');
mymotor = motor(myev3, 'A');
start(mymotor);
mymotor.Speed = 10;
start(mymotor)
pause(1);
stop(mymotor)
start(mymotor)
resetRotation(mymotor)
readRotation(mymotor)
while (readRotation(mymotor) > -90)
    mymotor.Speed = -10;
end
stop(mymotor)
end

%Arm down motion
function armdown(theta)
myev3 = legoev3('USB');
mymotor = motor(myev3, 'B');
start(mymotor);
resetRotation(mymotor);
uk1=0.0;ek1=0.0;ek2=0.0; c0 = 0.0;c1=0.0;c2=0.0;ek=0.0;uk=0.0;
kp=0.0000;kd=0.000;ki=0.0;t=0.01;


while (readRotation(mymotor) < theta-1)
   c0=kp+kd/t;
    c1=ki*t -kp -2*kd/t;
    c2=kd/t;
    ek = theta- readRotation(mymotor);
    uk=uk1+c0*ek+c1*ek1+c2*ek2;
    ek2=ek1;
    if ek > 20
        mymotor.Speed = 10;
        uk1=0.5;
        
    elseif ek<-20
        mymotor.Speed = -10;
        uk1=0.5;
        
    else
        mymotor.Speed = uk;
        uk1 = uk;
    end 
end
mymotor.Speed = 0;
rotation = readRotation(mymotor)
end
%Arm up motion

function armup(theta)
myev3 = legoev3('USB');
mymotor = motor(myev3, 'B');
start(mymotor);
resetRotation(mymotor);
uk1=0.0;ek1=0.0;ek2=0.0; c0 = 0.0;c1=0.0;c2=0.0;ek=0.0;uk=0.0;
kp=0.01;kd=0.0000;ki=0.0;t=0.01;


while (readRotation(mymotor) > theta)
   c0=kp+kd/t;
    c1=ki*t -kp -2*kd/t;
    c2=kd/t;
    ek = theta- double(readRotation(mymotor));
    uk=uk1+c0*ek+c1*ek1+c2*ek2;
    ek2=ek1;
    if ek > 20
        mymotor.Speed = 25;
        uk1=15;
        
    elseif ek<-20
        mymotor.Speed = -25;
        uk1=-15;
        
    else
        mymotor.Speed = uk;
        uk1 = uk;
    end 
end
readRotation(mymotor)
mymotor.Speed = 0;
end

%Gripper  close
function gripperclose()
myev3 = legoev3('USB');
mymotor = motor(myev3, 'A');
start(mymotor);
resetRotation(mymotor)
while (readRotation(mymotor) < 80)
    mymotor.Speed = 10;
end
stop(mymotor)
end
%gripper open
function gripperopen()
myev3 = legoev3('USB');
mymotor = motor(myev3, 'A');
start(mymotor);
resetRotation(mymotor)
while (readRotation(mymotor) > -80)
    mymotor.Speed = -10;
end
stop(mymotor)
end
