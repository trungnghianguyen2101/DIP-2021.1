clc; clear; warning off;
% img = imgetfile;
% img = imread(img); 
img = imread('E:\MATLAB\Images\10new.png');

if length(size(img))>2
    img = rgb2gray(img);
end 
%%applying prewitt filter in the horizontal direction
fx = [-1 0 1;-1 0 1;-1 0 1];
Ix = filter2(fx,img);
% applying prewitt filter in the vertical direction
fy = [1 1 1;0 0 0;-1 -1 -1];
Iy = filter2(fy,img); 

Ix2 = Ix.^2;
Iy2 = Iy.^2;
Ixy = Ix.*Iy;
clear Ix;
clear Iy;

%applying gaussian filter on the computed value
h= fspecial('gaussian',[7 7],2); 
Ix2 = filter2(h,Ix2);
Iy2 = filter2(h,Iy2);
Ixy = filter2(h,Ixy);
height = size(img,1);
width = size(img,2);
result = zeros(height,width); 
R = zeros(height,width);

Rmax = 0; 
for i = 1:height
    for j = 1:width
        M = [Ix2(i,j) Ixy(i,j);Ixy(i,j) Iy2(i,j)]; 
        R(i,j) = det(M)-0.01*(trace(M))^2;
        if R(i,j) > Rmax
            Rmax = R(i,j);
        end;
    end;
end;
cnt = 0;
for i = 2:height-1
    for j = 2:width-1
        if R(i,j) > 0.3*Rmax && R(i,j) > R(i-1,j-1) && R(i,j) > R(i-1,j) && R(i,j) > R(i-1,j+1) && R(i,j) > R(i,j-1) && R(i,j) > R(i,j+1) && R(i,j) > R(i+1,j-1) && R(i,j) > R(i+1,j) && R(i,j) > R(i+1,j+1)
%         if R(i,j) > 0.1*Rmax
            result(i,j) = 1;
            cnt = cnt+1;
        end;
    end;
end;
[posc, posr] = find(result == 1);
cnt ;
imshow(img);
hold on;
plot(posr,posc,'r.');