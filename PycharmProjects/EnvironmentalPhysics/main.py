from vpython import *
from scipy.constants import G, pi
import numpy as np
from math import exp
import os, sys

file = open("data.txt", "w")
#----------------------------------------------Is scene launched ? -----------------------------------------------------

is_launched = False
#-------------------------------------------- Initialize scene ---------------------------------------------------------
if(is_launched):
    scene = canvas( width=1800, height=1100)

#------------------------------------------ Conversion functions -------------------------------------------------------

def sec2day( seconds ):
    return seconds / 86400

def day2sec( days ):
    return days * 86400

def angVel( time ):
    return 2 * pi / time

def norm(vector):
    if mag(vector) != 0:
        return vector / mag(vector)
    else:
        return vector

#-------------------------------------------- Main debris class --------------------------------------------------------

class Debris():
    def __init__(self,radius, mass, position, velocity):
        self.radius = radius
        self.mass = mass
        self.position = position
        self.velocity = velocity
        self.gravity_force = - G * self.mass * self.mass / mag2( self.position ) * self.position
        self.acceleration = ( self.gravity_force / self.mass  )
        self.past_position = self.position
        self.add_force = vec(0.0,0.0,0.0)


    def launchCamera(self):
         self.obj = sphere( pos=self.position,
               vel=self.velocity,
               acc=self.acceleration,
               texture='https://st3.depositphotos.com/1035886/15256/i/1600/depositphotos_152562626-stock-photo-old-steel-texture.jpg',
               radius=100000,
               make_trail=True,
               trail_radius=50000,
               retain=500,
               interval=500 )
    
    def move(self, dt):
        if(mag(self.position) > RE):
            #   Forces   #
            self.gravity_force =  - G * self.mass * ME / mag2( self.position ) * self.position / mag(self.position)
            self.acceleration = (self.gravity_force - self.drag() - add_force) / self.mass
            #   vectors   #
            self.velocity = self.velocity + self.acceleration * dt
            Debris_past_pos = self.position
            self.past_position = self.position
            self.position = self.position + self.velocity * dt
            if is_launched:
                self.obj.pos = self.position
            return 0
        else:
            print("The debris has landed!")
            return 1


    def drag( self ):
        Cd = 0.47
        g = G * ME / mag2( self.position )

        if mag(self.position) - RE > 25000:
            temp = - 131.21 + 0.0299 * (mag(self.position) - RE)
            p = 2.488 * ((temp + 273.1)/ 216.6) ** -11.388
        elif 25000 > mag(self.position) - RE > 11000:
            temp = -56.46
            p = 22.65 * exp(1.73 - 0.000157 * (mag(self.position) - RE))
        else:
            temp = 15.04 - 0.00649 * ((mag(self.position) - RE))
            p = 101.29 * ((temp + 273.1)/288.08) ** 5.256

        #   Change pressure from kPa to Pa and temperature from C to K   #
        temp += 272.15
        p *= 1000

        rho = p/(temp * 287.058 )
        A = 3.14 * (self.radius ** 2)
        F = Cd * rho * A * 0.5 * mag2(self.velocity)
        F *= norm(self.velocity)
        return F

    def print_val(self, val):
        if val == 0:
            print("position:")
            print(self.position)
        elif val == 1:
            print("velocity: ")
            print(self.velocity)
        elif val == 2:
            print("gravity force: ")
            print(self.gravity_force)
        elif val == 3:
            print("drag force: ")
            print(self.drag())
        elif val == 4:
            print("height:")
            print(mag(self.position)- RE)


#------------------------------------------------ Constant values ------------------------------------------------------

ME = 5.9736e24
RE = 6.371e6

t = 0
dt = 0.01
T = 27.321

rE = vec( 0, 0, 0 )

vE = vec( 0, 0, 0 )
vM = vec( 962, 0, 0 )

aE = vec( 0, 0, 0 )

theta = angVel( 86400 ) * dt
phi = angVel( 2360534 ) * dt

if is_launched:
    Earth = sphere( pos=rE,
                     vel=vE,
                    acc=aE,
                    texture=textures.earth,
                    radius=RE )
    
    Atmosphere = sphere( pos=Earth.pos,
                         vel=Earth.vel,
                         acc=Earth.acc,
                         color=color.cyan,
                         opacity=0.2,
                         radius=RE+2e5 )
    
    Position_plot = graph( x=0, y=0, width=600, height=600,
                           xmin=-4.5e8, xmax=4.5e8,
                           ymin=-4.5e8, ymax=4.5e8,
                           foreground=color.black,
                           background=color.white,
                           title='X vs. Y Position',
                           xtitle='x(t) [m]',
                           ytitle='y(t) [m]' )

if is_launched:
    scene.camera.pos = vec( 8e6, 1.37829e6, 2e7 )
    scene.camera.axis = vec( -1.91006e8, -1.60134e8, -6.90157e8 )



height_list = [-10000]#, 45000,50000]
debris_list = []
speed_list = [5875]#,5750,5800, 5810, 5820, 5830, 5840, 5850, 5860, 5870, 5880, 5890]
force_list[vec(0,0,10)]
#for h in height_list:
h = 3000000
v_first = (G * ME / (RE + h) )**(1 / 2)
print(v_first)

for v in force_list:
    h_1 = vec(0.0, 0.0, float(RE + h))
    v_2 = vec(v_first , 0.0, 0.0)
    debris_list.append(Debris(10, 3e4, h_1, v_2))
    debris_list[-1].add_force = v
    if is_launched:
        debris_list[-1].launchCamera()
while True:
  if len(debris_list) == 0:
        break
  for d in debris_list:
    if d.move(dt):
        file.write(str(t) + "\n")
        debris_list.remove(d)
    if is_launched:
        Earth.rotate( angle=theta, origin=Earth.pos, axis=vec( 0, 1, 0 ) )
    t += dt

file.close()
