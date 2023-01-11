close all; clear all; clc;

A = load('coords.txt');

figure(1)
plot(A(:,1), A(:,2), '*r')
hold on
plot(A(:,3), A(:,4), '*r')

plot([A(:,1)'; A(:,3)'], [A(:,2)'; A(:,4)'], 'b') 

plot(A(:,5), A(:,6), '.g')
plot(A(:,7), A(:,8), '.g')

%plot([A(:,5)'; A(:,7)'], [A(:,6)'; A(:,7)'], 'k')
axis equal

B = load('sparsified_coords.txt');

figure;
plot(B(:,1), B(:,2), '*r')
hold on
plot(B(:,3), B(:,4), '*r')

plot([B(:,1)'; B(:,3)'], [B(:,2)'; B(:,4)'], 'b') 