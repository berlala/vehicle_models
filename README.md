# Vehicle Model
This folder contains the various models which perpare for the autonomous driving simulation.  

## Bicycle Model  
bic_kong.slx  
>Kinematic and Dynamic Vehicle Models for Autonomous Driving
Control Design
>http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf

bic_lego.slx  
>Model Predictive Control of a Mobile Robot Using
Linearization  
>http://www.ece.ufrgs.br/~fetter/mechrob04_553.pdf

bic_yutong.slx  
This model takes the rear wheel steering into account. Assume the velocity is constant.  
>Yutong Li，Junzhi Zhang, Chen Lv and Ye Yuan, Coordinated control of the steering system and the distributed motors for
comprehensive optimization of the dynamics performance and the energy
consumption of an electric vehicle, Part D，2017.  

![image](https://github.com/berlala/vehicle_models/blob/master/bic_yutong.png)

## Comparison
![image](https://github.com/berlala/vehicle_models/blob/master/bic_comp.png)   
The result shows above,  
It is obvious the discrete model by [c2d] with a larger error comparing to the linear version. The heading responces are larger than their linear and nonlinear model;  
\<Kong\> and \<Yutong\> model are steering based on the gravity-center angle, thus the lateral distance is smaller than \<Lego\>;
The linearization almost have no influence on <Kong> model, but for \<Lego\> model is very different. 
