# ENPM662 Project 2
# Author: Rohit M Patil
# Email: rpatil10@umd.edu


# Import libraries
import sympy as sym
sym.init_printing()
from sympy import *
from sympy.physics.vector import Vector
Vector.simp = True
import numpy as np
from numpy import *
import math

#### Denavit-Hartenberg Table ####

theta1, theta2, theta3, theta4, theta5, theta6 = sym.symbols("theta_1, theta_2, theta_3, theta_4, theta_5, theta_6")
d1, d2, d3, d4, d5, d6 = sym.symbols("d_1,d_2,d_3,d_4,d_5,d_6")
a1, a2, a3, a4, a5, a6 = sym.symbols("a_1,a_2,a_3,a_4,a_5,a_6")

d1 = 400
d2 = 0
d3 = 0
d4 = 620
d5 = 0
d6 = 200

a1 = 180
a2 = 600
a3 = 170
a4 = 0
a5 = 0
a6 = 0

#### Tranformation Matrices of KUKA KR5 Robot ####
#### Calculating Transformation Matrices from DH Parameters ####

A1 = sym.Matrix([[sym.cos(theta1), 0, sym.sin(theta1), sym.cos(theta1)*a1], [sym.sin(theta1), 0, -sym.cos(theta1), sym.sin(theta1)*a1], [0, 1, 0, d1], [0, 0, 0, 1]])
A2 = sym.Matrix([[sym.cos(theta2), -sym.sin(theta2), 0, sym.cos(theta2)*a2], [sym.sin(theta2), sym.cos(theta2), 0, sym.sin(theta2)*a2], [0, 0, 1, d2], [0, 0, 0, 1]])
A3 = sym.Matrix([[sym.cos(theta3), 0, sym.sin(theta3), sym.cos(theta3)*a3], [sym.sin(theta3), 0, -sym.cos(theta3), sym.sin(theta3)*a3], [0, 1, 0, d3], [0, 0, 0, 1]])
A4 = sym.Matrix([[sym.cos(theta4), 0, -sym.sin(theta4), sym.cos(theta4)*a4], [sym.sin(theta4), 0, sym.cos(theta4), sym.sin(theta4)*a4], [0, -1, 0, d4], [0, 0, 0, 1]])
A5 = sym.Matrix([[sym.cos(theta5), 0, sym.sin(theta5), sym.cos(theta5)*a5], [sym.sin(theta5), 0, -sym.cos(theta5), sym.sin(theta5)*a5], [0, 1, 0, d5], [0, 0, 0, 1]])
A6 = sym.Matrix([[sym.cos(theta6), -sym.sin(theta6), 0, sym.cos(theta6)*a6], [sym.sin(theta6), sym.cos(theta6), 0, sym.sin(theta6)*a6], [0, 0, 1, d6], [0, 0, 0, 1]])

#### Final Transformation Matrix ####

A = A1*A2*A3*A4*A5*A6

#### Transformation Matrix Values ####

V1 = N(A1.subs(theta1, sym.degree(90)))
V2 = N(A2.subs(theta2, sym.degree(0)))
V3 = N(A4.subs(theta4, sym.degree(0)))
V4 = N(A5.subs(theta5, sym.degree(0)))
V5 = N(A6.subs(theta6, sym.degree(0)))
#### Robot's Configurations ####

X1 = N(A.subs(theta1, sym.degree(90)))
X2 = N(X1.subs(theta2, sym.degree(0)))
X3 = N(X2.subs(theta4, sym.degree(-90)))
X4 = N(X3.subs(theta5, sym.degree(0)))
X5 = N(X4.subs(theta6, sym.degree(0)))
#### Jacobian ####

##### Calculate Z ####

Z0 = sym.Matrix([0,0,1])
Z1 = A1[:3,2]
A12 = A1*A2
Z2 = A12[:3,2]
A23 = A12*A3
Z3 = A23[:3,2]
A34 = A23*A4
A24 = A12*A3*A4
Z4 = A24[:3,2]
A45 = A24*A5
Z5 = A45[:3,2]
A56 = A45*A6
Z6 = A56[:3,2]
sym.simplify(Z6)


#### Calculate O ####

O0 = sym.Matrix([0, 0, 0])
O1 = A1[:3,3]
O2 = A12[:3,3]
O3 = A23[:3,3]
O4 = A34[:3,3]
O5 = A45[:3,3]
O6 = A56[:3,3]

#### Calculate Jacobian ####

from sympy.vector import CoordSys3D

xp = A[0,3]; yp = A[1,3]; zp = A[2,3];

a11 = sym.diff(xp, theta1)
a12 = sym.diff(xp, theta2)
a13 = sym.diff(xp, theta3)
a14 = sym.diff(xp, theta4)
a15 = sym.diff(xp, theta5)
a16 = sym.diff(xp, theta6)

a21 = sym.diff(yp, theta1)
a22 = sym.diff(yp, theta2)
a23 = sym.diff(yp, theta3)
a24 = sym.diff(yp, theta4)
a25 = sym.diff(yp, theta5)
a26 = sym.diff(yp, theta6)

a31 = sym.diff(zp, theta1)
a32 = sym.diff(zp, theta2)
a33 = sym.diff(zp, theta3)
a34 = sym.diff(zp, theta4)
a35 = sym.diff(zp, theta5)
a36 = sym.diff(zp, theta6)

vl = sym.Matrix([[a11, a12, a13, a14, a15, a16], [a21, a22, a23, a24, a25, a26],[a31, a32, a33, a34, a35, a36],[Z1,Z2,Z3,Z4,Z5,Z6]]) # assemble into matix form
vl_s = sym.simplify(vl)
from sympy.physics.vector import *
N = ReferenceFrame('N')
N.x.cross(N.y)

J1 = sym.Matrix([[Z0.cross(O6-O0)],[Z0]])
J2 = sym.Matrix([[Z1.cross(O6-O1)],[Z1]])
J3 = sym.Matrix([[Z2.cross(O6-O2)],[Z2]])
J4 = sym.Matrix([[Z4.cross(O6-O3)],[Z3]])
J5 = sym.Matrix([[Z5.cross(O6-O4)],[Z4]])
J6 = sym.Matrix([[Z6.cross(O6-O5)],[Z5]])
J = sym.Matrix([[J1, J2, J3, J4, J5, J6]])

#### Find Inverse Jacobian ####

theta_joint = sym.Matrix([20,-100,0,0,0,0])*(pi/180)
N = 60
ta = linspace(float(pi/2), float((5*pi)/2),num=N)
J_inverse = vl.evalf(3, subs={theta1:theta_joint[0],theta2:theta_joint[1],theta3:theta_joint[2],theta4:theta_joint[3],theta5:theta_joint[4],theta6:theta_joint[5]}).inv()
T = A.evalf(3, subs={theta1:theta_joint[0],theta2:theta_joint[1],theta3:theta_joint[2],theta4:theta_joint[3],theta5:theta_joint[4],theta6:theta_joint[5]})

import matplotlib.pyplot as plt

for i in range(0,N):
  y_dot = -100.0 * (2*pi/5)* sin(ta[i])  
  z_dot = 100.0 * (2*pi/5)* cos(ta[i])
  V = Matrix([y_dot,0.0, z_dot, 0.0, 0.0, 0.0])
  J_inverse = vl.evalf(3, subs={theta1:theta_joint[0],theta2:theta_joint[1],theta3:theta_joint[2],theta4:theta_joint[3],theta5:theta_joint[4],theta6:theta_joint[5]}).inv()
  theta_dot = J_inverse*V

  #### q_current= q_pervious + 𝑞̇ _current . ∆t ####
  theta_joint = theta_joint + (theta_dot*(5/N))

  T = A.evalf(3, subs={theta1:theta_joint[0],theta2:theta_joint[1],theta3:theta_joint[2],theta4:theta_joint[3],theta5:theta_joint[4],theta6:theta_joint[5]})
  plt.title( 'KUKA KR5: Project 2: Validation: Draw a circle ' )
  plt.scatter(T[2,3],T[0,3], color = 'blue')
plt.show()
