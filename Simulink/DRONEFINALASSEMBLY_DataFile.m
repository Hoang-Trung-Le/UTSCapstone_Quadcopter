% Simscape(TM) Multibody(TM) version: 7.2

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(100).translation = [0.0 0.0 0.0];
smiData.RigidTransform(100).angle = 0.0;
smiData.RigidTransform(100).axis = [0.0 0.0 0.0];
smiData.RigidTransform(100).ID = '';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [0.35433070866141769 0.39370078740157488 -3.3858267716535444];  % in
smiData.RigidTransform(1).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(1).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(1).ID = 'B[spider arm-1:-:motor-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [6.9722005946460525e-14 0.21653543307086634 -6.9722005946459124e-14];  % in
smiData.RigidTransform(2).angle = 1.717771517458405;  % rad
smiData.RigidTransform(2).axis = [0.86285620946101393 -0.3574067443365968 0.3574067443365968];
smiData.RigidTransform(2).ID = 'F[spider arm-1:-:motor-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0 0.8661417322834648 0];  % in
smiData.RigidTransform(3).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(3).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(3).ID = 'B[motor-1:-:spinner-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [2.6645352591003907e-15 0.13779527559055127 -2.6645352591003607e-15];  % in
smiData.RigidTransform(4).angle = 2.094395102393193;  % rad
smiData.RigidTransform(4).axis = [-0.5773502691896274 -0.57735026918962484 -0.57735026918962484];
smiData.RigidTransform(4).ID = 'F[motor-1:-:spinner-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [0.62992125984251901 0.30511811023622037 0.62992125984251945];  % in
smiData.RigidTransform(5).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(5).axis = [0.57735026918962584 -0.57735026918962573 0.57735026918962573];
smiData.RigidTransform(5).ID = 'B[:-:spacer-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [0.029527559055117947 0.2362204724409448 1.2303149606299209];  % in
smiData.RigidTransform(6).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(6).axis = [0.57735026918962595 -0.57735026918962573 0.57735026918962562];
smiData.RigidTransform(6).ID = 'F[:-:spacer-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [0.60039370078740106 0.068897637795275593 -0.60039370078740151];  % in
smiData.RigidTransform(7).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(7).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(7).ID = 'B[Distribution board-1:-:spacer-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [0 0 0];  % in
smiData.RigidTransform(8).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(8).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(8).ID = 'F[Distribution board-1:-:spacer-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [0.59055118110236271 0.39370078740157488 -0.15748031496063006];  % in
smiData.RigidTransform(9).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(9).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(9).ID = 'B[spider arm-1:-:Spider top-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [1.2249093847326029 -0.39370078740157471 -0.89084318889643888];  % in
smiData.RigidTransform(10).angle = 2.5935642459694801;  % rad
smiData.RigidTransform(10).axis = [0.28108463771482045 -0.678598344545847 0.678598344545847];
smiData.RigidTransform(10).ID = 'F[spider arm-1:-:Spider top-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(11).translation = [0.1771653543307089 0.23622047244094491 -2.424624630899896];  % in
smiData.RigidTransform(11).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(11).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(11).ID = 'B[spider arm-1:-:piller-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(12).translation = [-1.1153332981600116e-15 -0.082677165354330728 -7.2867597103430539e-16];  % in
smiData.RigidTransform(12).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(12).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(12).ID = 'F[spider arm-1:-:piller-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(13).translation = [0 0.57086614173228345 0];  % in
smiData.RigidTransform(13).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(13).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(13).ID = 'B[spinner-1:-:hex nut style 2_am-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(14).translation = [1.0219082524631773e-13 -1.0144662887512368e-14 0.047244094488189343];  % in
smiData.RigidTransform(14).angle = 1.1102230246251564e-16;  % rad
smiData.RigidTransform(14).axis = [0.94009411656755371 0.3409150216623944 1.7790891322051309e-17];
smiData.RigidTransform(14).ID = 'F[spinner-1:-:hex nut style 2_am-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(15).translation = [0 0.57086614173228345 0];  % in
smiData.RigidTransform(15).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(15).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(15).ID = 'B[spinner-1:-:fan(propeller)-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(16).translation = [-1.399243961986598e-14 0.31496062992125928 -9.9824532595736916e-14];  % in
smiData.RigidTransform(16).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(16).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(16).ID = 'F[spinner-1:-:fan(propeller)-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(17).translation = [0 0.57086614173228345 0];  % in
smiData.RigidTransform(17).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(17).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(17).ID = 'B[spinner-2:-:fan(propeller)-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(18).translation = [1.8777629205716207e-15 0.31496062992126084 1.0242214700989548e-13];  % in
smiData.RigidTransform(18).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(18).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962562];
smiData.RigidTransform(18).ID = 'F[spinner-2:-:fan(propeller)-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(19).translation = [-0.99999999999999845 0 0];  % in
smiData.RigidTransform(19).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(19).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(19).ID = 'B[pan cross head_am-6:-:hex nut style 2_am-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(20).translation = [-1.1102230246251565e-16 -4.4408920985006262e-16 -2.3149606299212584];  % in
smiData.RigidTransform(20).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(20).axis = [1 -2.383427947395772e-33 -5.2406011631744855e-17];
smiData.RigidTransform(20).ID = 'F[pan cross head_am-6:-:hex nut style 2_am-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(21).translation = [-0.99999999999999845 0 0];  % in
smiData.RigidTransform(21).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(21).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(21).ID = 'B[pan cross head_am-7:-:hex nut style 2_am-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(22).translation = [6.8001160258290838e-16 -2.2204460492503131e-16 -1.2712207461977392];  % in
smiData.RigidTransform(22).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(22).axis = [1 1.6973437467400376e-32 1.7820798384534482e-16];
smiData.RigidTransform(22).ID = 'F[pan cross head_am-7:-:hex nut style 2_am-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(23).translation = [0 0.11811023622047241 0];  % in
smiData.RigidTransform(23).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(23).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(23).ID = 'B[fan(propeller)-1:-:hex nut style 2_am-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(24).translation = [1.7555401576885288e-15 -1.4155343563970746e-15 -0.14960629921259752];  % in
smiData.RigidTransform(24).angle = 2.2887833992611187e-16;  % rad
smiData.RigidTransform(24).axis = [0.99128803449916647 0.13171193058633432 1.4941688519598061e-17];
smiData.RigidTransform(24).ID = 'F[fan(propeller)-1:-:hex nut style 2_am-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(25).translation = [-0.99999999999999845 0 0];  % in
smiData.RigidTransform(25).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(25).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(25).ID = 'B[pan cross head_am-7:-:hex nut style 2_am-5]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(26).translation = [1.6653345369377348e-16 4.4408920985006262e-16 -2.3149606299212584];  % in
smiData.RigidTransform(26).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(26).axis = [1 1.6973437467400376e-32 1.7820798384534482e-16];
smiData.RigidTransform(26).ID = 'F[pan cross head_am-7:-:hex nut style 2_am-5]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(27).translation = [0 0.082677165354330756 0];  % in
smiData.RigidTransform(27).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(27).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(27).ID = 'B[Spider top-1:-:battery-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(28).translation = [0 2.2204460492503131e-16 0];  % in
smiData.RigidTransform(28).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(28).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(28).ID = 'F[Spider top-1:-:battery-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(29).translation = [0.89084318889643677 0.16141732283464566 -1.2249093847326011];  % in
smiData.RigidTransform(29).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(29).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(29).ID = 'B[Spider top-1:-:pan cross head_am-6]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(30).translation = [0.094488188976378021 -2.55351295663786e-15 7.8756445809347042e-16];  % in
smiData.RigidTransform(30).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(30).axis = [0.57735026918962595 0.57735026918962584 0.57735026918962562];
smiData.RigidTransform(30).ID = 'F[Spider top-1:-:pan cross head_am-6]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(31).translation = [1.2249093847326009 0.16141732283464566 -0.8908431888964371];  % in
smiData.RigidTransform(31).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(31).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(31).ID = 'B[Spider top-1:-:pan cross head_am-7]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(32).translation = [0.094488188976378035 -2.6090241078691179e-15 3.2265856653168612e-16];  % in
smiData.RigidTransform(32).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(32).axis = [0.57735026918962573 0.57735026918962584 0.57735026918962573];
smiData.RigidTransform(32).ID = 'F[Spider top-1:-:pan cross head_am-7]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(33).translation = [0.60039370078740129 0.068897637795275593 0.60039370078740162];  % in
smiData.RigidTransform(33).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(33).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(33).ID = 'B[Distribution board-1:-:]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(34).translation = [0.60039370078740129 0.068897637795275593 0.60039370078740162];  % in
smiData.RigidTransform(34).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(34).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(34).ID = 'F[Distribution board-1:-:]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(35).translation = [0.6003937007874014 0.30511811023622049 -0.6003937007874014];  % in
smiData.RigidTransform(35).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(35).axis = [-0.57735026918962584 -0.57735026918962584 -0.57735026918962584];
smiData.RigidTransform(35).ID = 'B[:-:pan cross head_am-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(36).translation = [0.16338582677165356 1.3877787807814457e-16 6.2450045135165055e-17];  % in
smiData.RigidTransform(36).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(36).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962562];
smiData.RigidTransform(36).ID = 'F[:-:pan cross head_am-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(37).translation = [0.52136380657949755 0.39370078740157488 -3.2187936737354605];  % in
smiData.RigidTransform(37).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(37).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(37).ID = 'B[spider arm-1:-:pan slot head_am-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(38).translation = [0.45669291338582696 -4.4408920985006262e-16 5.2735593669694936e-16];  % in
smiData.RigidTransform(38).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(38).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(38).ID = 'F[spider arm-1:-:pan slot head_am-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(39).translation = [0.13161991143730739 0.39370078740157488 -3.1631159744294344];  % in
smiData.RigidTransform(39).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(39).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(39).ID = 'B[spider arm-1:-:pan slot head_am-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(40).translation = [0.4566929133858269 6.106226635438361e-16 1.6653345369377348e-16];  % in
smiData.RigidTransform(40).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(40).axis = [-0.57735026918962573 -0.57735026918962584 0.57735026918962573];
smiData.RigidTransform(40).ID = 'F[spider arm-1:-:pan slot head_am-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(41).translation = [0.18729761074333404 0.39370078740157488 -3.5528598695716256];  % in
smiData.RigidTransform(41).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(41).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(41).ID = 'B[spider arm-1:-:pan slot head_am-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(42).translation = [0.45669291338582679 5.5511151231257827e-16 2.0261570199409107e-15];  % in
smiData.RigidTransform(42).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(42).axis = [-0.57735026918962584 -0.57735026918962573 0.57735026918962573];
smiData.RigidTransform(42).ID = 'F[spider arm-1:-:pan slot head_am-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(43).translation = [0 0.39370078740157488 -3.5604342063993863];  % in
smiData.RigidTransform(43).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(43).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(43).ID = 'B[spider arm-1:-:electrical connector-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(44).translation = [-2.9305129465568669 0.27559055118110309 0.039370078740159409];  % in
smiData.RigidTransform(44).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(44).axis = [0.57735026918962595 -0.57735026918962562 0.57735026918962584];
smiData.RigidTransform(44).ID = 'F[spider arm-1:-:electrical connector-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(45).translation = [0.89084318889643677 0.15748031496062992 -1.2249093847326014];  % in
smiData.RigidTransform(45).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(45).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(45).ID = 'B[Spider base-1:-:spider arm-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(46).translation = [0.11811023622047276 -6.9388939039072284e-18 -0.15748031496062764];  % in
smiData.RigidTransform(46).angle = 1.7177715174584018;  % rad
smiData.RigidTransform(46).axis = [0.86285620946101671 -0.35740674433659347 0.35740674433659347];
smiData.RigidTransform(46).ID = 'F[Spider base-1:-:spider arm-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(47).translation = [0 0 0];  % in
smiData.RigidTransform(47).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(47).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(47).ID = 'B[Spider base-1:-:hex nut style 2_am-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(48).translation = [-0.89084318889643876 1.2249093847326029 -0.11417322834645693];  % in
smiData.RigidTransform(48).angle = 1.5700924586837749e-16;  % rad
smiData.RigidTransform(48).axis = [-1 0 -0];
smiData.RigidTransform(48).ID = 'F[Spider base-1:-:hex nut style 2_am-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(49).translation = [0 0 0];  % in
smiData.RigidTransform(49).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(49).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(49).ID = 'B[Spider base-1:-:hex nut style 2_am-5]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(50).translation = [-1.3708450153327323 0.64404078438137946 -0.11417322834645682];  % in
smiData.RigidTransform(50).angle = 1.5700924586837749e-16;  % rad
smiData.RigidTransform(50).axis = [-0.99551038425501526 -0.094652389512530466 7.3972893845989815e-18];
smiData.RigidTransform(50).ID = 'F[Spider base-1:-:hex nut style 2_am-5]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(51).translation = [-0.60039370078740184 0.068897637795275593 0.60039370078740162];  % in
smiData.RigidTransform(51).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(51).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(51).ID = 'B[Distribution board-1:-:Spider base-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(52).translation = [-0.60039370078740184 0.20669291338582679 0.60039370078740162];  % in
smiData.RigidTransform(52).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(52).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(52).ID = 'F[Distribution board-1:-:Spider base-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(53).translation = [0 0 0];  % in
smiData.RigidTransform(53).angle = 0;  % rad
smiData.RigidTransform(53).axis = [0 0 0];
smiData.RigidTransform(53).ID = 'SixDofRigidTransform[Assem2^DRONE FINAL ASSEMBLY-1]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(54).translation = [-0.6003937007874055 0.46850393700787402 0.60039370078739718];  % in
smiData.RigidTransform(54).angle = 2.8681409334758161;  % rad
smiData.RigidTransform(54).axis = [0.70038224236591418 -0.70038224236591429 -0.1375842620250852];
smiData.RigidTransform(54).ID = 'SixDofRigidTransform[pan cross head_am-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(55).translation = [-0.60039370078739507 0.46850393700787402 -0.60039370078740761];  % in
smiData.RigidTransform(55).angle = 2.3311017338252205;  % rad
smiData.RigidTransform(55).axis = [-0.63873604552392771 0.63873604552392782 -0.42899012610654541];
smiData.RigidTransform(55).ID = 'SixDofRigidTransform[pan cross head_am-5]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(56).translation = [0.6003937007873994 0.068897637795275593 0.60039370078740317];  % in
smiData.RigidTransform(56).angle = 1.5707963267949003;  % rad
smiData.RigidTransform(56).axis = [-0 -1 -0];
smiData.RigidTransform(56).ID = 'SixDofRigidTransform[spacer-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(57).translation = [-0.60039370078740517 0.068897637795275593 0.6003937007873974];  % in
smiData.RigidTransform(57).angle = 3.1415926535897865;  % rad
smiData.RigidTransform(57).axis = [0 1 0];
smiData.RigidTransform(57).ID = 'SixDofRigidTransform[spacer-3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(58).translation = [0.60039370078739929 0.46850393700787402 0.60039370078740339];  % in
smiData.RigidTransform(58).angle = 1.8868911265677468;  % rad
smiData.RigidTransform(58).axis = [0.48697064523984657 -0.48697064523984651 -0.7250649497454521];
smiData.RigidTransform(58).ID = 'SixDofRigidTransform[pan cross head_am-3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(59).translation = [-0.60039370078739518 0.068897637795275593 -0.60039370078740728];  % in
smiData.RigidTransform(59).angle = 1.5707963267948861;  % rad
smiData.RigidTransform(59).axis = [0 1 0];
smiData.RigidTransform(59).ID = 'SixDofRigidTransform[spacer-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(60).translation = [0.89084318889643466 1.0629921259842521 1.224909384732606];  % in
smiData.RigidTransform(60).angle = 1.7840699021849158;  % rad
smiData.RigidTransform(60).axis = [0.41795485925891629 -0.41795485925891629 -0.80661482210762703];
smiData.RigidTransform(60).ID = 'SixDofRigidTransform[pan cross head_am-9]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(61).translation = [3.576882430802578 -0.043307086614173172 3.3406619583616513];  % in
smiData.RigidTransform(61).angle = 1.6952447724915312;  % rad
smiData.RigidTransform(61).axis = [0.33229680179855181 0.33229680179855176 0.88269908294328026];
smiData.RigidTransform(61).ID = 'SixDofRigidTransform[pan slot head_am-8]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(62).translation = [3.3406619583617032 0.1968503937007873 3.3406619583615837];  % in
smiData.RigidTransform(62).angle = 3.1415926535897869;  % rad
smiData.RigidTransform(62).axis = [-0 -1 -0];
smiData.RigidTransform(62).ID = 'SixDofRigidTransform[motor-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(63).translation = [3.3406619583616348 -0.043307086614173242 3.0257013284403924];  % in
smiData.RigidTransform(63).angle = 1.6327207324779447;  % rad
smiData.RigidTransform(63).axis = [0.24140897726774346 0.24140897726774346 0.9399167044951825];
smiData.RigidTransform(63).ID = 'SixDofRigidTransform[pan slot head_am-6]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(64).translation = [3.104441485920689 -0.043307086614173242 3.3406619583616499];  % in
smiData.RigidTransform(64).angle = 1.5951262787982619;  % rad
smiData.RigidTransform(64).axis = [0.15410962912152093 0.15410962912152093 0.97596129248247065];
smiData.RigidTransform(64).ID = 'SixDofRigidTransform[pan slot head_am-7]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(65).translation = [1.2249093847325998 -0.25196850393700798 0.89084318889644298];  % in
smiData.RigidTransform(65).angle = 2.0943951023931975;  % rad
smiData.RigidTransform(65).axis = [0.5773502691896244 -0.57735026918962651 0.57735026918962651];
smiData.RigidTransform(65).ID = 'SixDofRigidTransform[hex nut style 2_am-8]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(66).translation = [1.1970705350795869 0.01968503937007874 0.6959712413253466];  % in
smiData.RigidTransform(66).angle = 2.3561944901923475;  % rad
smiData.RigidTransform(66).axis = [-0 -1 -0];
smiData.RigidTransform(66).ID = 'SixDofRigidTransform[spider arm-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(67).translation = [3.3406619583617068 0.92519685039370114 3.3406619583615815];  % in
smiData.RigidTransform(67).angle = 3.1415926535897913;  % rad
smiData.RigidTransform(67).axis = [-0 -1 -0];
smiData.RigidTransform(67).ID = 'SixDofRigidTransform[spinner-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(68).translation = [3.340661958361633 1.4488188976377949 3.340661958361653];  % in
smiData.RigidTransform(68).angle = 1.7512824219271939;  % rad
smiData.RigidTransform(68).axis = [-0.83403989041148408 0.39011374059522669 0.39011374059522669];
smiData.RigidTransform(68).ID = 'SixDofRigidTransform[hex nut style 2_am-7]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(69).translation = [0.8908431888964351 -0.25196850393700798 1.2249093847326062];  % in
smiData.RigidTransform(69).angle = 1.9886202782376143;  % rad
smiData.RigidTransform(69).axis = [0.65015786206529702 -0.53725913412183246 0.53725913412183246];
smiData.RigidTransform(69).ID = 'SixDofRigidTransform[hex nut style 2_am-9]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(70).translation = [1.2249093847325998 1.0629921259842521 0.89084318889644298];  % in
smiData.RigidTransform(70).angle = 1.8690045722689488;  % rad
smiData.RigidTransform(70).axis = [0.4765372947498846 -0.4765372947498846 -0.73879930524122939];
smiData.RigidTransform(70).ID = 'SixDofRigidTransform[pan cross head_am-10]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(71).translation = [-3.3406619583616797 0.1968503937007873 -3.3406619583616073];  % in
smiData.RigidTransform(71).angle = 0;  % rad
smiData.RigidTransform(71).axis = [0 0 0];
smiData.RigidTransform(71).ID = 'SixDofRigidTransform[motor-3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(72).translation = [-3.3406619583616646 1.181102362204725 3.3406619583616228];  % in
smiData.RigidTransform(72).angle = 2.4923422478514103;  % rad
smiData.RigidTransform(72).axis = [-0 -1 -0];
smiData.RigidTransform(72).ID = 'SixDofRigidTransform[fan(propeller)-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(73).translation = [-3.0257013284404044 -0.043307086614173242 3.3406619583616242];  % in
smiData.RigidTransform(73).angle = 1.8328386884064358;  % rad
smiData.RigidTransform(73).axis = [-0.45359973731159003 -0.45359973731158992 0.76713398870191696];
smiData.RigidTransform(73).ID = 'SixDofRigidTransform[pan slot head_am-14]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(74).translation = [-3.3406619583615957 0.1968503937007873 3.3406619583616912];  % in
smiData.RigidTransform(74).angle = 1.5707963267948997;  % rad
smiData.RigidTransform(74).axis = [0 1 0];
smiData.RigidTransform(74).ID = 'SixDofRigidTransform[motor-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(75).translation = [-3.3406619583616832 0.92519685039370114 -3.3406619583616051];  % in
smiData.RigidTransform(75).angle = 0;  % rad
smiData.RigidTransform(75).axis = [0 0 0];
smiData.RigidTransform(75).ID = 'SixDofRigidTransform[spinner-3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(76).translation = [-2.7862642299822489 0.17322834645669297 -2.5357145831051779];  % in
smiData.RigidTransform(76).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(76).axis = [0.47594950435734401 0 0.87947260861382071];
smiData.RigidTransform(76).ID = 'SixDofRigidTransform[piller-3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(77).translation = [1.8373640770989015 0.37401574803149557 0.9465208882024716];  % in
smiData.RigidTransform(77).angle = 2.5935642459694783;  % rad
smiData.RigidTransform(77).axis = [-0.28108463771482123 0.67859834454584667 0.67859834454584689];
smiData.RigidTransform(77).ID = 'SixDofRigidTransform[electrical connector-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(78).translation = [-3.3406619583616135 -0.043307086614173242 -3.0257013284404159];  % in
smiData.RigidTransform(78).angle = 2.7879434670137666;  % rad
smiData.RigidTransform(78).axis = [-0.69572608725003238 -0.69572608725003249 0.17869085885831026];
smiData.RigidTransform(78).ID = 'SixDofRigidTransform[pan slot head_am-9]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(79).translation = [-3.3406619583616095 1.4488188976377949 -3.3406619583616766];  % in
smiData.RigidTransform(79).angle = 2.533067829735006;  % rad
smiData.RigidTransform(79).axis = [-0.31401272144672226 -0.67134045415482868 -0.67134045415482868];
smiData.RigidTransform(79).ID = 'SixDofRigidTransform[hex nut style 2_am-12]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(80).translation = [-0.89084318889644742 -0.25196850393700798 1.2249093847325967];  % in
smiData.RigidTransform(80).angle = 3.1415926535897882;  % rad
smiData.RigidTransform(80).axis = [2.4688501310822653e-15 0.70710678118654746 -0.70710678118654757];
smiData.RigidTransform(80).ID = 'SixDofRigidTransform[hex nut style 2_am-13]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(81).translation = [-3.3406619583616641 -0.043307086614173172 3.576882430802566];  % in
smiData.RigidTransform(81).angle = 1.7419031245497874;  % rad
smiData.RigidTransform(81).axis = [-0.38144277866655141 -0.38144277866655146 0.84202304790681393];
smiData.RigidTransform(81).ID = 'SixDofRigidTransform[pan slot head_am-12]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(82).translation = [-3.3406619583616606 -0.043307086614173242 3.1044414859206766];  % in
smiData.RigidTransform(82).angle = 1.9240325016631974;  % rad
smiData.RigidTransform(82).axis = [-0.50697397533357802 -0.50697397533357802 0.69710456652423181];
smiData.RigidTransform(82).ID = 'SixDofRigidTransform[pan slot head_am-13]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(83).translation = [2.7862642299822662 0.17322834645669297 2.535714583105158];  % in
smiData.RigidTransform(83).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(83).axis = [0.87947260861381915 0 -0.47594950435734701];
smiData.RigidTransform(83).ID = 'SixDofRigidTransform[piller-2]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(84).translation = [-0.89084318889642611 1.0629921259842521 -1.2249093847326122];  % in
smiData.RigidTransform(84).angle = 2.4789017985936375;  % rad
smiData.RigidTransform(84).axis = [-0.66394433423828192 0.66394433423828192 -0.34402883900302478];
smiData.RigidTransform(84).ID = 'SixDofRigidTransform[pan cross head_am-11]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(85).translation = [-3.3406619583616082 1.1811023622047239 -3.3406619583616748];  % in
smiData.RigidTransform(85).angle = 0.84052287721768382;  % rad
smiData.RigidTransform(85).axis = [0 -1 0];
smiData.RigidTransform(85).ID = 'SixDofRigidTransform[fan(propeller)-3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(86).translation = [-1.2249093847326094 -0.25196850393700798 0.89084318889643077];  % in
smiData.RigidTransform(86).angle = 3.0076338211341334;  % rad
smiData.RigidTransform(86).axis = [0.067079758256298874 -0.70551410546929416 0.70551410546929416];
smiData.RigidTransform(86).ID = 'SixDofRigidTransform[hex nut style 2_am-14]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(87).translation = [-2.5357145831051677 0.17322834645669297 2.7862642299822573];  % in
smiData.RigidTransform(87).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(87).axis = [0.95842816745210979 0 0.28533392338519931];
smiData.RigidTransform(87).ID = 'SixDofRigidTransform[piller-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(88).translation = [-1.2249093847325934 -0.25196850393700798 -0.89084318889645164];  % in
smiData.RigidTransform(88).angle = 2.0943951023931895;  % rad
smiData.RigidTransform(88).axis = [0.57735026918962973 0.57735026918962384 -0.57735026918962384];
smiData.RigidTransform(88).ID = 'SixDofRigidTransform[hex nut style 2_am-11]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(89).translation = [-1.2249093847325934 1.0629921259842521 -0.89084318889645164];  % in
smiData.RigidTransform(89).angle = 2.3549016229316231;  % rad
smiData.RigidTransform(89).axis = [-0.64335020725124259 0.64335020725124248 -0.41497110942759202];
smiData.RigidTransform(89).ID = 'SixDofRigidTransform[pan cross head_am-12]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(90).translation = [-1.8373640770988948 0.37401574803149557 -0.94652088820248437];  % in
smiData.RigidTransform(90).angle = 1.7177715174584056;  % rad
smiData.RigidTransform(90).axis = [-0.86285620946101382 -0.35740674433659703 -0.35740674433659675];
smiData.RigidTransform(90).ID = 'SixDofRigidTransform[electrical connector-3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(91).translation = [-3.1044414859206655 -0.043307086614173242 -3.3406619583616717];  % in
smiData.RigidTransform(91).angle = 2.9205639903820684;  % rad
smiData.RigidTransform(91).axis = [-0.70273979708231216 -0.70273979708231216 0.11096645976790184];
smiData.RigidTransform(91).ID = 'SixDofRigidTransform[pan slot head_am-11]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(92).translation = [-0.89084318889642655 -0.25196850393700798 -1.2249093847326125];  % in
smiData.RigidTransform(92).angle = 2.2071010894830034;  % rad
smiData.RigidTransform(92).axis = [0.50450584216116079 0.61052184859563297 -0.61052184859563297];
smiData.RigidTransform(92).ID = 'SixDofRigidTransform[hex nut style 2_am-10]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(93).translation = [-3.5768824308025544 -0.043307086614173172 -3.3406619583616766];  % in
smiData.RigidTransform(93).angle = 2.6380373928929504;  % rad
smiData.RigidTransform(93).axis = [-0.68331159756688398 -0.68331159756688398 0.257236314040583];
smiData.RigidTransform(93).ID = 'SixDofRigidTransform[pan slot head_am-10]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(94).translation = [-1.1970705350795821 0.01968503937007874 -0.69597124132535493];  % in
smiData.RigidTransform(94).angle = 0.78539816339743862;  % rad
smiData.RigidTransform(94).axis = [0 1 0];
smiData.RigidTransform(94).ID = 'SixDofRigidTransform[spider arm-3]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(95).translation = [-0.69597124132535071 0.01968503937007874 1.1970705350795845];  % in
smiData.RigidTransform(95).angle = 2.3561944901923386;  % rad
smiData.RigidTransform(95).axis = [0 1 0];
smiData.RigidTransform(95).ID = 'SixDofRigidTransform[spider arm-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(96).translation = [-3.3406619583615931 0.92519685039370081 3.3406619583616939];  % in
smiData.RigidTransform(96).angle = 1.5707963267948952;  % rad
smiData.RigidTransform(96).axis = [0 1 0];
smiData.RigidTransform(96).ID = 'SixDofRigidTransform[spinner-4]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(97).translation = [-1.2249093847326091 1.0629921259842521 0.89084318889643033];  % in
smiData.RigidTransform(97).angle = 2.7104417443556517;  % rad
smiData.RigidTransform(97).axis = [0.68994512614596815 -0.68994512614596815 -0.21897818570544417];
smiData.RigidTransform(97).ID = 'SixDofRigidTransform[pan cross head_am-13]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(98).translation = [-0.89084318889644742 1.0629921259842521 1.2249093847325967];  % in
smiData.RigidTransform(98).angle = 2.8421636065481901;  % rad
smiData.RigidTransform(98).axis = [0.69901585066092509 -0.6990158506609252 -0.15084323335690758];
smiData.RigidTransform(98).ID = 'SixDofRigidTransform[pan cross head_am-14]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(99).translation = [-3.340661958361665 1.4488188976377949 3.3406619583616206];  % in
smiData.RigidTransform(99).angle = 1.6872826350708008;  % rad
smiData.RigidTransform(99).axis = [-0.88980704742576033 -0.32267895682198627 -0.32267895682198627];
smiData.RigidTransform(99).ID = 'SixDofRigidTransform[hex nut style 2_am-15]';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(100).translation = [-0.94652088820247793 0.37401574803149557 1.8373640770988982];  % in
smiData.RigidTransform(100).angle = 1.7177715174583996;  % rad
smiData.RigidTransform(100).axis = [-0.86285620946101871 0.35740674433659059 0.35740674433659109];
smiData.RigidTransform(100).ID = 'SixDofRigidTransform[electrical connector-4]';


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(16).mass = 0.0;
smiData.Solid(16).CoM = [0.0 0.0 0.0];
smiData.Solid(16).MoI = [0.0 0.0 0.0];
smiData.Solid(16).PoI = [0.0 0.0 0.0];
smiData.Solid(16).color = [0.0 0.0 0.0];
smiData.Solid(16).opacity = 0.0;
smiData.Solid(16).ID = '';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.0022194246296915903;  % kg
smiData.Solid(1).CoM = [0.034002604983655968 0.88060597345497316 0.048578907903083002];  % mm
smiData.Solid(1).MoI = [0.23383093545684036 0.46562945203113432 0.23297894661838872];  % kg*mm^2
smiData.Solid(1).PoI = [-0.00019340596249350258 -0.0011732480335847426 -0.00013534882446308619];  % kg*mm^2
smiData.Solid(1).color = [0.89803921568627454 0.89803921568627454 0.89803921568627454];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = 'Distribution board*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.00013803359471627105;  % kg
smiData.Solid(2).CoM = [6.4056040925546158 0 0];  % mm
smiData.Solid(2).MoI = [0.00027931427810942174 0.0031595462490633772 0.0031595462490633772];  % kg*mm^2
smiData.Solid(2).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(2).color = [0.77647058823529413 0.75686274509803919 0.73725490196078436];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = 'pan cross head_am*:*B18.6.7M - M3 x 0.5 x 13 Type I Cross Recessed PHMS --13N';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.010841044083842972;  % kg
smiData.Solid(3).CoM = [0 1.5826622696003838 0];  % mm
smiData.Solid(3).MoI = [5.0599116978435363 10.092986780740359 5.0599116978435248];  % kg*mm^2
smiData.Solid(3).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(3).color = [0.10196078431372549 0.10196078431372549 0.10196078431372549];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = 'Spider base*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 7.539822368615502e-05;  % kg
smiData.Solid(4).CoM = [0 3 0];  % mm
smiData.Solid(4).MoI = [0.00038641589639154447 0.0003204424506661589 0.00038641589639154447];  % kg*mm^2
smiData.Solid(4).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(4).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = 'spacer*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 0.0076254058577004411;  % kg
smiData.Solid(5).CoM = [6.1321896185844142 6.2009373042728573 -35.459225987747523];  % mm
smiData.Solid(5).MoI = [6.3297976103381011 6.2723250016433498 0.33133572580047188];  % kg*mm^2
smiData.Solid(5).PoI = [-0.50639766100786399 -0.17330534632518863 -0.020315944549367904];  % kg*mm^2
smiData.Solid(5).color = [1 1 1];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = 'spider arm*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(6).mass = 0.011006441211363099;  % kg
smiData.Solid(6).CoM = [0 1.6290403745968893 0];  % mm
smiData.Solid(6).MoI = [5.4466881060332302 10.979735820143024 5.5596743720866115];  % kg*mm^2
smiData.Solid(6).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(6).color = [0.10196078431372549 0.10196078431372549 0.10196078431372549];
smiData.Solid(6).opacity = 1;
smiData.Solid(6).ID = 'Spider top*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(7).mass = 0.00025819933883287389;  % kg
smiData.Solid(7).CoM = [14.547492614488595 0 0];  % mm
smiData.Solid(7).MoI = [0.00041450092187633747 0.025782256391707663 0.025782256391707663];  % kg*mm^2
smiData.Solid(7).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(7).color = [0.77647058823529413 0.75686274509803919 0.73725490196078436];
smiData.Solid(7).opacity = 1;
smiData.Solid(7).ID = 'pan cross head_am*:*B18.6.7M - M3 x 0.5 x 30 Type I Cross Recessed PHMS --30N';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(8).mass = 3.61766194942426e-05;  % kg
smiData.Solid(8).CoM = [2.8245402835792492 -1.428408750321969e-08 1.6994681767525007e-08];  % mm
smiData.Solid(8).MoI = [4.3039856085944897e-05 0.00020467422902736206 0.00020585669318499825];  % kg*mm^2
smiData.Solid(8).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(8).color = [0.77647058823529413 0.75686274509803919 0.73725490196078436];
smiData.Solid(8).opacity = 1;
smiData.Solid(8).ID = 'pan slot head_am*:*B18.6.7M - M2 x 0.4 x 6 Slotted PHMS --6N';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(9).mass = 0.005425862401822374;  % kg
smiData.Solid(9).CoM = [-1.1557376363344803e-10 7.2292546912446127 -2.9251876320949402e-10];  % mm
smiData.Solid(9).MoI = [0.2751218385441746 0.35096264045024317 0.27564962789328029];  % kg*mm^2
smiData.Solid(9).PoI = [0 5.2498826818115768e-09 0];  % kg*mm^2
smiData.Solid(9).color = [0.96862745098039216 0.9882352941176471 1];
smiData.Solid(9).opacity = 1;
smiData.Solid(9).ID = 'motor*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(10).mass = 0.0029588111615642494;  % kg
smiData.Solid(10).CoM = [-0.065916218780186608 3.5020070501706959 0.0060353161476651108];  % mm
smiData.Solid(10).MoI = [0.085887766046528202 0.22422235532227952 0.16044231049668509];  % kg*mm^2
smiData.Solid(10).PoI = [-1.3792734096880839e-06 0.00014435016733073579 5.5914124795389727e-05];  % kg*mm^2
smiData.Solid(10).color = [0.56862745098039214 0.5490196078431373 0.50980392156862742];
smiData.Solid(10).opacity = 1;
smiData.Solid(10).ID = 'electrical connector*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(11).mass = 0.00012695380472341395;  % kg
smiData.Solid(11).CoM = [0 1.2281169841186426e-06 -1.8985799497161076];  % mm
smiData.Solid(11).MoI = [0.00066863141516518786 0.0006686283001832912 0.001039314077621663];  % kg*mm^2
smiData.Solid(11).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(11).color = [0.77647058823529413 0.75686274509803919 0.73725490196078436];
smiData.Solid(11).opacity = 1;
smiData.Solid(11).ID = 'hex nut style 2_am*:*B18.2.4.2M - Hex nut, Style 2,  M4 x 0.7 --D-N';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(12).mass = 6.0954022917407744e-05;  % kg
smiData.Solid(12).CoM = [0 9.6370721200994807e-07 -1.4487828947711949];  % mm
smiData.Solid(12).MoI = [0.00019383179382017737 0.00019383087153577525 0.00030415285090940215];  % kg*mm^2
smiData.Solid(12).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(12).color = [0.77647058823529413 0.75686274509803919 0.73725490196078436];
smiData.Solid(12).opacity = 1;
smiData.Solid(12).ID = 'hex nut style 2_am*:*B18.2.4.2M - Hex nut, Style 2,  M3 x 0.5 --D-N';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(13).mass = 0.00069316100308805225;  % kg
smiData.Solid(13).CoM = [0 3.1033357505438741 0];  % mm
smiData.Solid(13).MoI = [0.017701457780990516 0.019468565284905606 0.017701457780990516];  % kg*mm^2
smiData.Solid(13).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(13).color = [0.77647058823529413 0.75686274509803919 0.73725490196078436];
smiData.Solid(13).opacity = 1;
smiData.Solid(13).ID = 'spinner*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(14).mass = 0.00013514803755870655;  % kg
smiData.Solid(14).CoM = [-1.095556282056858e-07 6.8005219371187833 -4.1748080045260785e-08];  % mm
smiData.Solid(14).MoI = [0.0013469266042481411 0.00038861016002877876 0.0013469263465715933];  % kg*mm^2
smiData.Solid(14).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(14).color = [0.20000000000000001 0.40000000000000002 0.69803921568627447];
smiData.Solid(14).opacity = 1;
smiData.Solid(14).ID = 'piller*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(15).mass = 0.00093498850674554796;  % kg
smiData.Solid(15).CoM = [0 0.31485424212697155 0];  % mm
smiData.Solid(15).MoI = [0.57895152334529054 0.59201178944451371 0.016675330321771226];  % kg*mm^2
smiData.Solid(15).PoI = [0 -0.044621378881340594 0];  % kg*mm^2
smiData.Solid(15).color = [0.10196078431372549 0.10196078431372549 0.10196078431372549];
smiData.Solid(15).opacity = 1;
smiData.Solid(15).ID = 'fan(propeller)*:*Default';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(16).mass = 0.041443671392366752;  % kg
smiData.Solid(16).CoM = [0 8.75 0];  % mm
smiData.Solid(16).MoI = [5.009647716137632 20.823553833260888 17.874928810062499];  % kg*mm^2
smiData.Solid(16).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(16).color = [0.49803921568627452 0.49803921568627452 0.49803921568627452];
smiData.Solid(16).opacity = 1;
smiData.Solid(16).ID = 'battery*:*Default';


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the CylindricalJoint structure array by filling in null values.
smiData.CylindricalJoint(4).Rz.Pos = 0.0;
smiData.CylindricalJoint(4).Pz.Pos = 0.0;
smiData.CylindricalJoint(4).ID = '';

smiData.CylindricalJoint(1).Rz.Pos = -39.865263865094768;  % deg
smiData.CylindricalJoint(1).Pz.Pos = 0;  % in
smiData.CylindricalJoint(1).ID = '[spinner-1:-:hex nut style 2_am-4]';

smiData.CylindricalJoint(2).Rz.Pos = -24.35468159479635;  % deg
smiData.CylindricalJoint(2).Pz.Pos = 0;  % in
smiData.CylindricalJoint(2).ID = '[pan cross head_am-6:-:hex nut style 2_am-1]';

smiData.CylindricalJoint(3).Rz.Pos = -24.35468159479635;  % deg
smiData.CylindricalJoint(3).Pz.Pos = 0;  % in
smiData.CylindricalJoint(3).ID = '[pan cross head_am-7:-:hex nut style 2_am-2]';

smiData.CylindricalJoint(4).Rz.Pos = -24.35468159479635;  % deg
smiData.CylindricalJoint(4).Pz.Pos = 0;  % in
smiData.CylindricalJoint(4).ID = '[pan cross head_am-7:-:hex nut style 2_am-5]';


%Initialize the PlanarJoint structure array by filling in null values.
smiData.PlanarJoint(5).Rz.Pos = 0.0;
smiData.PlanarJoint(5).Px.Pos = 0.0;
smiData.PlanarJoint(5).Py.Pos = 0.0;
smiData.PlanarJoint(5).ID = '';

%This joint has been chosen as a cut joint. Simscape Multibody treats cut joints as algebraic constraints to solve closed kinematic loops. The imported model does not use the state target data for this joint.
smiData.PlanarJoint(1).Rz.Pos = 1.2722218725854064e-14;  % deg
smiData.PlanarJoint(1).Px.Pos = 0;  % in
smiData.PlanarJoint(1).Py.Pos = 0;  % in
smiData.PlanarJoint(1).ID = '[:-:spacer-1]';

%This joint has been chosen as a cut joint. Simscape Multibody treats cut joints as algebraic constraints to solve closed kinematic loops. The imported model does not use the state target data for this joint.
smiData.PlanarJoint(2).Rz.Pos = -167.06457196106049;  % deg
smiData.PlanarJoint(2).Px.Pos = 0;  % in
smiData.PlanarJoint(2).Py.Pos = 0;  % in
smiData.PlanarJoint(2).ID = '[fan(propeller)-1:-:hex nut style 2_am-4]';

smiData.PlanarJoint(3).Rz.Pos = 0;  % deg
smiData.PlanarJoint(3).Px.Pos = 0;  % in
smiData.PlanarJoint(3).Py.Pos = 0;  % in
smiData.PlanarJoint(3).ID = '[Spider top-1:-:battery-1]';

%This joint has been chosen as a cut joint. Simscape Multibody treats cut joints as algebraic constraints to solve closed kinematic loops. The imported model does not use the state target data for this joint.
smiData.PlanarJoint(4).Rz.Pos = -90;  % deg
smiData.PlanarJoint(4).Px.Pos = 0;  % in
smiData.PlanarJoint(4).Py.Pos = 0;  % in
smiData.PlanarJoint(4).ID = '[Spider base-1:-:hex nut style 2_am-1]';

%This joint has been chosen as a cut joint. Simscape Multibody treats cut joints as algebraic constraints to solve closed kinematic loops. The imported model does not use the state target data for this joint.
smiData.PlanarJoint(5).Rz.Pos = -100.8626260896901;  % deg
smiData.PlanarJoint(5).Px.Pos = 0;  % in
smiData.PlanarJoint(5).Py.Pos = 0;  % in
smiData.PlanarJoint(5).ID = '[Spider base-1:-:hex nut style 2_am-5]';


%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(10).Rz.Pos = 0.0;
smiData.RevoluteJoint(10).ID = '';

smiData.RevoluteJoint(1).Rz.Pos = 0;  % deg
smiData.RevoluteJoint(1).ID = '[Distribution board-1:-:spacer-1]';

smiData.RevoluteJoint(2).Rz.Pos = -11.842380359075998;  % deg
smiData.RevoluteJoint(2).ID = '[spider arm-1:-:piller-1]';

smiData.RevoluteJoint(3).Rz.Pos = 127.19930809596572;  % deg
smiData.RevoluteJoint(3).ID = '[spinner-1:-:fan(propeller)-1]';

smiData.RevoluteJoint(4).Rz.Pos = -48.158413448765671;  % deg
smiData.RevoluteJoint(4).ID = '[spinner-2:-:fan(propeller)-2]';

smiData.RevoluteJoint(5).Rz.Pos = -114.35468159479636;  % deg
smiData.RevoluteJoint(5).ID = '[Spider top-1:-:pan cross head_am-6]';

smiData.RevoluteJoint(6).Rz.Pos = -125.21730768448644;  % deg
smiData.RevoluteJoint(6).ID = '[Spider top-1:-:pan cross head_am-7]';

smiData.RevoluteJoint(7).Rz.Pos = 112.22752703112745;  % deg
smiData.RevoluteJoint(7).ID = '[:-:pan cross head_am-1]';

smiData.RevoluteJoint(8).Rz.Pos = -62.946453529767687;  % deg
smiData.RevoluteJoint(8).ID = '[spider arm-1:-:pan slot head_am-1]';

smiData.RevoluteJoint(9).Rz.Pos = -73.809079619457805;  % deg
smiData.RevoluteJoint(9).ID = '[spider arm-1:-:pan slot head_am-2]';

smiData.RevoluteJoint(10).Rz.Pos = -86.258231564634002;  % deg
smiData.RevoluteJoint(10).ID = '[spider arm-1:-:pan slot head_am-4]';

