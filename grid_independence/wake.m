clear all;
close all;
clc;

wake1 = load("1wake.mat").tip;
wake3 = load("3wake.mat").tip;
wake5 = load("5wake.mat").tip;
wake8 = load("8wake.mat").tip;
wake10 = load("10wake.mat").tip;
wake15 = load("15wake.mat").tip;

err1 = rmse(wake1, wake5)/abs(max(wake1))*100;
err2 = rmse(wake5, wake10)/abs(max(wake5))*100;
err3 = rmse(wake10, wake15(1:2916))/abs(max(wake10))*100;
%err4 = rmse(wake8, wake10)/abs(max(wake8))*100;
