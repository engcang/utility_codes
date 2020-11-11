function dx=robot_rk(x,u,T)
k1=robot_plant(x,u)*T;
k2=robot_plant(x+k1*0.5,u)*T;
k3=robot_plant(x+k2*0.5,u)*T;
k4=robot_plant(x+k3,u)*T;
dx=x + ((k1+k4)/6+(k2+k3)/3);