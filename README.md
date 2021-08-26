# Graph Optimization with g2o
---

This repository contains some examples of using graph optimization using the [g2o library](https://github.com/RainerKuemmerle/g2o).

<div align="center">
<img src="/Images/curve_fit.png" width="300" height="250" alt="fit" />
<img src="/Images/robot.gif" width="300" height="250" alt="robot" />
</div>

<div align="center">
<table> 
<tr> 
<td> 
    
+ **Line_fit**: Optimize the parameters of a line equation of type: 

    >                       yi = m*xi + b

    + **inputs**:
        + __*noise_std_dev*__: standard deviation of the measurement noise
        + __*num_obs*__: number of observations
        + __*max_iterations*__: maximum number of iterations
    + **outputs**:
        + __*m*__, and __*b*__: optimized parameters for line equation
 
<tr>
<td> 

+ **Curve_fit**: Optimize the parameters of a curve equation of type: 

    >                       a*exp(b*x_i) + c

    + **inputs**:
        + __*noise_std_dev*__: standard deviation of the measurement noise
        + __*num_obs*__: number of observations
        + __*max_iterations*__: maximum number of iterations
    + **outputs**:
        + __*a*__, __*b*__ and __*c*__: optimized parameters for curve equation
 
<tr>
<td> 

+ **Polinomial_fit**: Optimize the parameters of a polinomial curve of type: 

    >                       yi = a*xi*xi + b*xi + c;

    + **inputs**:
        + __*noise_std_dev*__: standard deviation of the measurement noise
        + __*num_obs*__: number of observations
        + __*max_iterations*__: maximum number of iterations
    + **outputs**:
        + __*a*__, __*b*__ and __*c*__: optimized parameters for polinomial equation
 
<tr>
<td> 

+ **Polinomial_fit**: Optimize the parameters of a polinomial curve of type: 

    >                       yi = a*xi*xi + b*xi + c;

    + **inputs**:
        + __*noise_std_dev*__: standard deviation of the measurement noise
        + __*num_obs*__: number of observations
        + __*max_iterations*__: maximum number of iterations
    + **outputs**:
        + __*a*__, __*b*__ and __*c*__: optimized parameters for polinomial equation
 
<tr>
<td> 

+ **2D_Translation_Optimization**: Optimize a two-dimensional vector position. This vector represents the position of a point in the cartesian plane.

    + **inputs**:
        + __*noise_std_dev*__: standard deviation of the measurement noise
        + __*num_obs*__: number of observations
    + **outputs**:
        + __*x*__ and __*y*__: optimized coordinates

</table>
</div>
    
## How to Use:

+ Dependences (mandatory):
    + **g2o**
    + **Eigen3**
    + **CSparse**

+ Download any folder and open a terminal _`ctrl+t`_:
    ```
    $ cd path 
    $ mkdir build & cd build 
    $ cmake .. 
    $ make
    ```

## NOTE:

| If you find any of these codes helpful, please share my __[GitHub](https://github.com/LuisOrtizF)__ and __*STAR*__ :star: this repository to help other enthusiasts to find these tools. Remember, the knowledge must be shared. Otherwise, it is useless and lost in time. |
| :----------- |
