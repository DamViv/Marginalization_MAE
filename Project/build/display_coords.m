close all; clear all; clc;

A = load('coords_including_noise.txt');

figure('name','graph including noise');
plot(A(:,1), A(:,2), '*r')
hold on
plot(A(:,3), A(:,4), '*r')

plot([A(:,1)'; A(:,3)'], [A(:,2)'; A(:,4)'], 'b') 

plot(A(:,5), A(:,6), '.g')
plot(A(:,7), A(:,8), '.g')

plot([A(:,5)'; A(:,7)'], [A(:,6)'; A(:,8)'], 'k')
axis equal

B = load('sparsified_coords.txt');

figure('name','Sparsed graph including noise');
plot(B(:,1), B(:,2), '*r')
hold on
plot(B(:,3), B(:,4), '*r')

plot([B(:,1)'; B(:,3)'], [B(:,2)'; B(:,4)'], 'b')

plot(B(:,5), B(:,6), '.g')
plot(B(:,7), B(:,8), '.g')

plot([B(:,5)'; B(:,7)'], [B(:,6)'; B(:,8)'], 'k')

C = load('coords_without_noise.txt');
figure('name','Graph without noise');
plot(C(:,1), C(:,2), '*r')
hold on
plot(C(:,3), C(:,4), '*r')

plot([C(:,1)'; C(:,3)'], [C(:,2)'; C(:,4)'], 'b')