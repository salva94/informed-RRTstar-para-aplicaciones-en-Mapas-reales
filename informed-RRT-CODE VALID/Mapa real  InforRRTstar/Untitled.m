clear
clc
close all

A=imread('1.jpg');
figure
imshow(A,[]);

A=double(A);
B=zeros(size(A,1),size(A,2));
for i=1:size(B,1)
    for j=1:size(B,2)
        if A(i,j,1)>250 & A(i,j,2)>250 & A(i,j,2)>250
            B(i,j)=1;
        end
        if abs(A(i,j,1)-A(i,j,2))<8 & abs(A(i,j,1)-A(i,j,3))<8 & abs(A(i,j,3)-A(i,j,2))<8
            B(i,j)=1;
        end
    end
end
imBw = im2bw(B);                        
imLabel = bwlabel(imBw);                
stats = regionprops(imLabel,'Area');    
area = cat(1,stats.Area);
index = find(area == max(area));        
img = ismember(imLabel,index);          
figure
imshow(img,[]);